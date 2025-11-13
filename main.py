import cv2
import numpy as np
import time
import ntcore
import argparse

S = np.array([
    [ 0,  0,  1],   # forward  = +Z_cv
    [-1,  0,  0],   # left     = -X_cv
    [ 0, -1,  0],   # up       = -Y_cv
], dtype=np.float64)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-t",
        "--team",
        type=int,
        help="Team Number",
    )

    parser.add_argument(
        "-s",
        "--sim",
        action="store_true",
        help="should setup for sim"
    )

    args = parser.parse_args()

    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4("vizion")

    vision_table = inst.getTable("vision")

    tag_pose_publishers = {}

    if args.sim:
        inst.setServer("localhost")
    else:
        inst.setServerTeam(args.team)

    cap = cv2.VideoCapture(0)

    aruco_dict: cv2.aruco.Dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    cam_mat = np.array([
        [942.2778458640017, 0, 636.898111776291],
        [0, 946.896116804522, 405.45681464476786],
        [0, 0, 1],
    ], dtype=np.float32)

    dist_coeff = np.array([
        -0.007574449506918484,
        -0.16696507835814586,
        0.005954991219207065,
        0.0016744647761098121,
        0.3191633376366791], dtype=np.float32)

    win_cam_mat = np.array([
        [975.6914800562727, 0, 657.1922238570544],
        [0, 976.3474157317961, 351.54103305664995],
        [0, 0, 1]
    ], dtype=np.float32)

    win_dist_coeff = np.array([
        0.04389964856406952,
        -0.17211539337258833,
        -0.002305299309464663,
        0.0016484276591536293,
        0.13158928243393753 
    ], dtype=np.float32)

    marker_size = 0.071

    coord_system = np.array([
        [-marker_size / 2,  marker_size / 2, 0],
        [ marker_size / 2,  marker_size / 2, 0],
        [ marker_size / 2, -marker_size / 2, 0],
        [-marker_size / 2, -marker_size / 2, 0]
    ], dtype=np.float32).reshape(-1, 1, 3)

    prev_frame_time, new_frame_time = 0, 0

    while True:
        ret, frame = cap.read()

        if not ret:
            print("Error: recieve frame")
            break

        new_frame_time = time.time()
        fps = 1 / (new_frame_time - prev_frame_time)
        prev_frame_time = new_frame_time
        fps_text = "fps: " +  str(int(fps))

        if len(frame.shape) == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        elif len(frame.shape) == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = detector.detectMarkers(gray)
        rvecs, tvecs = [], []

        vis = frame.copy()

        cv2.putText(
            vis, 
            fps_text,
            (10, 70), 
            cv2.FONT_HERSHEY_SIMPLEX,
            2,
            (0, 255, 0),
            3,
            cv2.LINE_AA
        )

        tag_str = "tag ids: "

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(vis, corners, ids)

            for id in ids:
                tag_str += str(id[0]) + " "


            for corner, tag_id in zip(corners, ids.flatten()):
                img_points = corner.reshape(-1, 1, 2).astype(np.float32)

                success, rvec, tvec = cv2.solvePnP(
                    coord_system.astype(np.float32),
                    img_points,
                    win_cam_mat.astype(np.float32),
                    win_dist_coeff.astype(np.float32)
                )

                if not success:
                    continue

                rvecs.append(rvec)
                tvecs.append(tvec)

                x, y, z, roll, pitch, yaw = opencv_pnp_to_wpilib_pose(rvec, tvec)

                topic_name = f"tag_{int(tag_id)}_pose_cam"
                if tag_id not in tag_pose_publishers:
                    tag_pose_publishers[tag_id] = vision_table.getDoubleArrayTopic(topic_name).publish()

                pub = tag_pose_publishers[tag_id]

                pub.set([x, y, z, roll, pitch, yaw])

            inst.flush()

            for i in range(len(ids)):
                cv2.drawFrameAxes(vis, win_cam_mat, win_dist_coeff, rvecs[i], tvecs[i], marker_size)

        cv2.putText(
            vis,
            tag_str,
            (10, 140),
            cv2.FONT_HERSHEY_SIMPLEX,
            2,
            (0, 255, 0),
            3,
            cv2.LINE_AA
        )

        cv2.imshow("Live Feed", vis)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


# TODO: FIGURE OUT WHAT THIS DOES ITS GPT
def opencv_pnp_to_wpilib_pose(rvec, tvec):
    """
    rvec, tvec from cv2.solvePnP for a TAG.
    Returns (x, y, z, roll, pitch, yaw) of the TAG in the CAMERA frame,
    expressed in WPILib's coordinate system.
    """
     # 1) Rotation matrix in OpenCV camera frame (tag -> cam_cv)
    R_cv, _ = cv2.Rodrigues(rvec)      # (3,3)
    t_cv = tvec.reshape(3, 1)          # (3,1)

    # 2) Change basis: OpenCV camera -> WPILib camera
    R_wp = S @ R_cv                    # (3,3)
    t_wp = S @ t_cv                    # (3,1)

    # 3) Translation in WPILib camera frame
    x, y, z = t_wp.flatten().tolist()

    # 4) Extract roll, pitch, yaw from R_wp (Rotation3d convention)
    R = R_wp
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    singular = sy < 1e-6

    if not singular:
        roll  = np.arctan2(R[2, 1], R[2, 2])       # about X
        pitch = np.arctan2(-R[2, 0], sy)           # about Y
        yaw   = np.arctan2(R[1, 0], R[0, 0])       # about Z
    else:
        roll  = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw   = 0.0

    return x, y, z, roll, pitch, yaw


if __name__ == "__main__":
    main()
