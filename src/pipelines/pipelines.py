from abc import ABC, abstractmethod
from cv2.typing import MatLike
from numpy.typing import NDArray
from wpimath.geometry import Pose3d, Transform3d, Rotation3d, Translation3d
from config import ConfigManager, MARKER_SIZE, SINGLE_FID_COORD_SYSTEM

import cv2
import numpy as np
import math


class BasePipeline(ABC):
    """Base abstract class for various pipelines that can be run"""

    @abstractmethod
    def run(self, frame: NDArray, vis: NDArray) -> Pose3d:
        pass


class FiducialPipeline(BasePipeline):
    """Fiducial Pipline for use with 36h11 fiducial markers"""

    multitag: bool
    detector: cv2.aruco.ArucoDetector
    config_manager: ConfigManager

    def __init__(self, multitag: bool, config_manager: ConfigManager) -> None:
        self.multitag = multitag

        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
            cv2.aruco.DetectorParameters(),
        )

        self.config_manager = config_manager

    def run(self, frame: NDArray, vis: NDArray) -> Pose3d:
        corners, ids, _ = self.detector.detectMarkers(frame)
        pose = Pose3d()

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(vis, corners, ids)

            obsv_points = []
            coords = []
            tag_poses = []
            tag_ids = []
            tag_corners = []
            for corner, tag_id in zip(corners, ids.flatten()):
                tag_pose = None
                tag = self.config_manager.fiducial_map.get_tag_by_id(int(tag_id))

                if tag is not None:
                    tag_pose = tag.pose

                    obsv_points += [
                        self._wpilibTranslationToOpenCv(
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
                        self._wpilibTranslationToOpenCv(
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
                        self._wpilibTranslationToOpenCv(
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
                        self._wpilibTranslationToOpenCv(
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

            if len(tag_ids) == 1:
                img_points = tag_corners[0].reshape(-1, 1, 2).astype(np.float32)

                _, rvecs, tvecs, _ = cv2.solvePnPGeneric(
                    SINGLE_FID_COORD_SYSTEM.astype(np.float32),
                    img_points,
                    self.config_manager.calibration_data.cam_mat,
                    self.config_manager.calibration_data.dist_coeff,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE,
                )

                pose = self._opencv_to_wpilib(tvecs[0], rvecs[0])

            elif len(tag_ids) >= 2:
                _, rvecs, tvecs, _ = cv2.solvePnPGeneric(
                    np.array(obsv_points),
                    np.array(coords),
                    self.config_manager.calibration_data.cam_mat,
                    self.config_manager.calibration_data.dist_coeff,
                    flags=cv2.SOLVEPNP_SQPNP,
                )

                camera_to_field_pose = self._opencv_to_wpilib(tvecs[0], rvecs[0])
                camera_to_field = Transform3d(
                    camera_to_field_pose.translation(), camera_to_field_pose.rotation()
                )
                field_to_camera = camera_to_field.inverse()
                pose = Pose3d(field_to_camera.translation(), field_to_camera.rotation())

        return pose

    def _wpilibTranslationToOpenCv(self, translation: Translation3d) -> list[float]:
        return [-translation.Y(), -translation.Z(), translation.X()]

    def _opencv_to_wpilib(self, tvec: MatLike, rvec: MatLike) -> Pose3d:
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
