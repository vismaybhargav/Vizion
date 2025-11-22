from typing import List

from cv2.typing import MatLike
from config import marker_size
from numpy.typing import NDArray

import cv2
import numpy as np
import time
import ntcore
import argparse
import math
import platform

from wpimath.geometry import Pose3d, Translation3d, Rotation3d

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

coord_system = np.array([
    [-marker_size / 2,  marker_size / 2, 0],
    [ marker_size / 2,  marker_size / 2, 0],
    [ marker_size / 2, -marker_size / 2, 0],
    [-marker_size / 2, -marker_size / 2, 0]
], dtype=np.float32).reshape(-1, 1, 3)

def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "-t",
        "--team",
        type=int,
        help="Team Number",
        default=2473
    )

    parser.add_argument(
        "-s",
        "--sim",
        action="store_true",
        help="should setup for sim"
    )

    parser.add_argument(
        "-n",
        "--networktable",
        action="store_true",
        help="push to network tables"
    )

    parser.add_argument(
        "-d",
        "--debug",
        action="store_false",
        help="show debug information on output"
    )

    os_name = platform.system() # TODO: Log this onto the debug output
    print(os_name)

    args = parser.parse_args()

    print("Starting NT Client")
    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4("vizion")

    vision_table = inst.getTable("fid-pipeline")

    print("Generating Camera Calibrations")

    tag_pose_publishers = {}

    print("Starting NT Server")
    if args.sim:
        inst.setServer("localhost")
    else:
        inst.setServerTeam(args.team)

    print("Starting Video Capture")
    cap = cv2.VideoCapture(0)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    params = cv2.aruco.DetectorParameters() # TODO: This needs to be exposed to users
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)

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

        frame = make_safe_frame(frame)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = detector.detectMarkers(gray)
        rvecs_arr, tvecs_arr = [], []

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

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(vis, corners, ids)

            tag_ids = ids.flatten()

            for id in ids:
                tag_str += str(id) + " "

            for corner, tag_id in zip(corners, ids.flatten()):
                img_points = corner.reshape(-1, 1, 2).astype(np.float32)

                _, rvecs, tvecs, errors = cv2.solvePnPGeneric(
                    coord_system.astype(np.float32),
                    img_points,
                    cam_mat,
                    dist_coeff,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )

                rvecs_arr.append(rvecs)
                tvecs_arr.append(tvecs)

                if errors is not None:
                    continue

                # x, y, z, roll, pitch, yaw = opencv_pnp_to_wpilib_pose(rvec, tvec)
                print(tvecs)
                print(rvecs)
                pose = opencv_to_wpilib(tvecs[0], rvecs[0])

                if args.networktable:
                    print("Hello")
                    if tag_id not in tag_pose_publishers:
                        topic_name = f"tag_{int(tag_id)}_pose_cam"
                        tag_pose_publishers[tag_id] = vision_table.getDoubleArrayTopic(topic_name).publish()

                    pub = tag_pose_publishers[tag_id]

                    pub.set([
                        pose.X(),
                        pose.Y(),
                        pose.Z(),
                        pose.rotation().X(),
                        pose.rotation().Y(),
                        pose.rotation().Z()
                    ])


            if args.networktable:
                inst.flush()

            for i in range(len(ids)):
                cv2.drawFrameAxes(vis, cam_mat, dist_coeff, rvecs_arr[i][0], tvecs_arr[i][0], marker_size)

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

def make_safe_frame(frame: NDArray) -> NDArray:
    safe_frame: NDArray

    if len(frame.shape) == 2:
        safe_frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    elif len(frame.shape) == 4:
        safe_frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    else:
        safe_frame = frame

    return safe_frame

# ALL OF MECH ADV CODE HERE
def opencv_to_wpilib(tvec: MatLike, rvec: MatLike) -> Pose3d:
    return Pose3d(
        Translation3d(tvec[2][0], -tvec[0][0], -tvec[1][0]),
        Rotation3d(
            np.array([rvec[2][0], -rvec[0][0], -rvec[1][0]]),
            math.sqrt(math.pow(rvec[0][0], 2) + math.pow(rvec[1][0], 2) + math.pow(rvec[2][0], 2)),
        ),
    )


def wpilibTranslationToOpenCv(translation: Translation3d) -> List[float]:
    return [-translation.Y(), -translation.Z(), translation.X()]
# END OF MECH ADV CODE

if __name__ == "__main__":
    main()
