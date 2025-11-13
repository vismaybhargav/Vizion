import cv2
import numpy as np
import time

def main():
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

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(vis, corners, ids)

            for id in ids:
                tag_str += str(id[0]) + " "


            for corner in corners:
                img_points = corner.reshape(-1, 1, 2).astype(np.float32)

                success, rvec, tvec = cv2.solvePnP(
                    coord_system.astype(np.float32), 
                    img_points, cam_mat.astype(np.float32),
                    dist_coeff.astype(np.float32)
                )

                if success:
                    rvecs.append(rvec)
                    tvecs.append(tvec)

                print(tvecs)
                print(rvecs)



            for i in range(len(ids)):
                cv2.drawFrameAxes(vis, cam_mat, dist_coeff, rvecs[i], tvecs[i], marker_size)

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


if __name__ == "__main__":
    main()
