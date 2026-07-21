import cv2
import numpy as np
import time
import ntcore
import argparse
import math
import platform
import logging

from typing import List
from cv2.typing import MatLike
from config import (
    MARKER_SIZE,
    CameraCalibration,
    ConfigManager,
    SINGLE_FID_COORD_SYSTEM,
)
from numpy.typing import NDArray
from wpimath.geometry import Pose3d, Transform3d, Translation3d, Rotation3d
from sim.sim import Simulation
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    setup_parser(parser)

    args = parser.parse_args()

    logging.basicConfig(level=(logging.DEBUG if args.debug else logging.INFO))
    logger = logging.getLogger("[Vizion] [Fiducial Pipeline]")

    os_name = platform.system()
    logger.info(os_name)

    logger.info("Starting NT Client")
    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4("vizion")
    logger.info("Started NT Client")

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    sim = (
        Simulation((1920, 1080), Path("../maps/2026-rebuilt-welded.json"), aruco_dict)
        if args.sim
        else None
    )
    if sim is not None:
        sim.begin()

    vision_table = inst.getTable("fid-pipeline")

    logger.info("Generating Camera Calibrations")
    config_manager = ConfigManager(
        "../maps/2026-rebuilt-welded.json",
        "../calibrations/mac_calib.json"
        if os_name == "Darwin"
        else "../calibrations/OV9821_calib.json",
    )
    if sim is not None:
        config_manager.calibration_data = CameraCalibration.ideal_pinhole(
            sim.window_size, sim.camera.fovy
        )
    logger.info("Generated Camera Calibrations")

    tag_pose_publishers = {}

    # logger.info("Starting NT Server")
    # inst.setServer("localhost") if args.sim else inst.setServerTeam(args.team)

    cap = None
    if sim is None:
        logger.info("Starting Video Capture")
        cap = cv2.VideoCapture(0)
        logger.info("Started Video Capture")

    params = cv2.aruco.DetectorParameters()  # TODO: This needs to be exposed to users
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    prev_frame_time, new_frame_time = 0, 0

    while True:
        if sim is not None:
            if sim.should_close():
                break

            sim.update(np.array([10, 10, 0], dtype=np.uint8))
            frame = sim.raylib_screen_to_bgr_np()
        else:
            assert cap is not None
            ret, frame = cap.read()

            if not ret:
                logger.error("Error: recieve frame")
                break

        new_frame_time = time.time()
        fps = 1 / (new_frame_time - prev_frame_time)
        prev_frame_time = new_frame_time
        fps_text = "fps: " + str(int(fps))

        frame = make_safe_frame(frame)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = detector.detectMarkers(gray)
        vis = frame.copy()

        cv2.putText(
            vis,
            fps_text,
            (10, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            2,
            (0, 255, 0),
            3,
            cv2.LINE_AA,
        )

        tag_str = "tag ids: "

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(vis, corners, ids)

            obsv_points = []
            coords = []
            tag_poses = []
            tag_ids = []
            tag_corners = []

            for corner, tag_id in zip(corners, ids.flatten()):
                tag_pose = None
                tag = config_manager.fiducial_map.get_tag_by_id(int(tag_id))

                if tag is not None:
                    tag_pose = tag.pose

                    obsv_points += [
                        wpilibTranslationToOpenCv(
                            (
                                tag_pose
                                + Transform3d(
                                    0,
                                    MARKER_SIZE / 2.0,
                                    -MARKER_SIZE / 2.0,
                                    Rotation3d(),
                                )
                            ).translation()
                        ),
                        wpilibTranslationToOpenCv(
                            (
                                tag_pose
                                + Transform3d(
                                    0,
                                    -MARKER_SIZE / 2.0,
                                    -MARKER_SIZE / 2.0,
                                    Rotation3d(),
                                )
                            ).translation()
                        ),
                        wpilibTranslationToOpenCv(
                            (
                                tag_pose
                                + Transform3d(
                                    0,
                                    -MARKER_SIZE / 2.0,
                                    MARKER_SIZE / 2.0,
                                    Rotation3d(),
                                )
                            ).translation()
                        ),
                        wpilibTranslationToOpenCv(
                            (
                                tag_pose
                                + Transform3d(
                                    0,
                                    MARKER_SIZE / 2.0,
                                    MARKER_SIZE / 2.0,
                                    Rotation3d(),
                                )
                            ).translation()
                        ),
                    ]

                    coords += [
                        [corner[0][0][0], corner[0][0][1]],
                        [corner[0][1][0], corner[0][1][1]],
                        [corner[0][2][0], corner[0][2][1]],
                        [corner[0][3][0], corner[0][3][1]],
                    ]

                    tag_ids.append(tag_id)
                    tag_poses.append(tag_pose)
                    tag_corners.append(corner)

            # Multi-tag PnP estimates the pose of the field coordinate system,
            # not an individual tag.  Estimate every visible tag in its local
            # coordinate system so the debug axes stay attached to that tag.
            """
            for corner in tag_corners:
                success, rvec, tvec = cv2.solvePnP(
                    SINGLE_FID_COORD_SYSTEM,
                    corner.reshape(-1, 1, 2).astype(np.float32),
                    config_manager.calibration_data.cam_mat,
                    config_manager.calibration_data.dist_coeff,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE,
                )
                if success:
                    cv2.drawFrameAxes(
                        vis,
                        config_manager.calibration_data.cam_mat,
                        config_manager.calibration_data.dist_coeff,
                        rvec,
                        tvec,
                        MARKER_SIZE,
                    )
            """

            if len(tag_ids) == 1:
                img_points = tag_corners[0].reshape(-1, 1, 2).astype(np.float32)

                _, rvecs, tvecs, _ = cv2.solvePnPGeneric(
                    SINGLE_FID_COORD_SYSTEM.astype(np.float32),
                    img_points,
                    config_manager.calibration_data.cam_mat,
                    config_manager.calibration_data.dist_coeff,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE,
                )

                pose = opencv_to_wpilib(tvecs[0], rvecs[0])

                if args.network_table:
                    if tag_ids[0] not in tag_pose_publishers:
                        topic_name = f"tag_{int(tag_ids[0])}_pose_cam"
                        tag_pose_publishers[tag_ids[0]] = (
                            vision_table.getDoubleArrayTopic(topic_name).publish()
                        )

                    pub = tag_pose_publishers[tag_ids[0]]

                    pub.set(
                        [
                            pose.X(),
                            pose.Y(),
                            pose.Z(),
                            pose.rotation().X(),
                            pose.rotation().Y(),
                            pose.rotation().Z(),
                        ]
                    )

            elif len(tag_ids) >= 2:
                _, rvecs, tvecs, _ = cv2.solvePnPGeneric(
                    np.array(obsv_points),
                    np.array(coords),
                    config_manager.calibration_data.cam_mat,
                    config_manager.calibration_data.dist_coeff,
                    flags=cv2.SOLVEPNP_SQPNP,
                )

                camera_to_field_pose = opencv_to_wpilib(tvecs[0], rvecs[0])
                camera_to_field = Transform3d(
                    camera_to_field_pose.translation(), camera_to_field_pose.rotation()
                )
                field_to_camera = camera_to_field.inverse()
                pose = Pose3d(field_to_camera.translation(), field_to_camera.rotation())

                if args.network_table:
                    if tag_ids[0] not in tag_pose_publishers:
                        topic_name = f"tag_{int(tag_ids[0])}_pose_cam"
                        tag_pose_publishers[tag_ids[0]] = (
                            vision_table.getDoubleArrayTopic(topic_name).publish()
                        )

                    pub = tag_pose_publishers[tag_ids[0]]

                    pub.set(
                        [
                            pose.X(),
                            pose.Y(),
                            pose.Z(),
                            pose.rotation().X(),
                            pose.rotation().Y(),
                            pose.rotation().Z(),
                        ]
                    )

                    print(pose.X(), pose.Y(), pose.Z())

            if args.network_table:
                inst.flush()

        cv2.putText(
            vis,
            tag_str,
            (10, 140),
            cv2.FONT_HERSHEY_SIMPLEX,
            2,
            (0, 255, 0),
            3,
            cv2.LINE_AA,
        )

        cv2.imshow("Live Feed", vis)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    if sim is not None:
        sim.end_simulation()

    if cap is not None:
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
            math.sqrt(
                math.pow(rvec[0][0], 2)
                + math.pow(rvec[1][0], 2)
                + math.pow(rvec[2][0], 2)
            ),
        ),
    )


def wpilibTranslationToOpenCv(translation: Translation3d) -> List[float]:
    return [-translation.Y(), -translation.Z(), translation.X()]


# END OF MECH ADV CODE


def setup_parser(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("-t", "--team", type=int, help="Team Number", default=2473)

    parser.add_argument("-s", "--sim", action="store_true", help="should setup for sim")

    parser.add_argument(
        "-n", "--network-table", action="store_true", help="push to network tables"
    )

    parser.add_argument(
        "-mt",
        "--multi-tag",
        action="store_true",
        help="use multitag targeting in the fid pipeline",
    )

    parser.add_argument(
        "-f", "--fps", type=int, default=60, help="FPS to run the camera at"
    )

    parser.add_argument("-d", "--debug", action="store_true", help="Show debug logs")


if __name__ == "__main__":
    main()
