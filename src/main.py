import cv2
import numpy as np
import time
import ntcore
import argparse
import platform
import logging

from config import (
    CameraCalibration,
    ConfigManager,
)
from numpy.typing import NDArray
from pipelines.pipelines import FiducialPipeline
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

    # logger.info("Starting NT Server")
    # inst.setServer("localhost") if args.sim else inst.setServerTeam(args.team)

    cap = None
    if sim is None:
        logger.info("Starting Video Capture")
        cap = cv2.VideoCapture(0)
        logger.info("Started Video Capture")

    prev_frame_time, new_frame_time = 0, 0

    fid_pipeline = FiducialPipeline(True, config_manager)

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

        vis = frame.copy()

        pose = fid_pipeline.run(gray, vis)

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
