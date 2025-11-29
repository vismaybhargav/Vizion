from numpy.typing import NDArray
from wpimath.geometry import Pose3d, Quaternion, Rotation3d, Translation3d
from viz_types import Fiducial

import numpy as np
import json

MARKER_SIZE = 0.071 # meters

SINGLE_FID_COORD_SYSTEM = np.array([
    [-MARKER_SIZE / 2,  MARKER_SIZE / 2, 0],
    [ MARKER_SIZE / 2,  MARKER_SIZE / 2, 0],
    [ MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
    [-MARKER_SIZE / 2, -MARKER_SIZE / 2, 0]
], dtype=np.float32).reshape(-1, 1, 3)

class CameraCalibration:
    """
    Calibration of the camera. (Currently) only supports CalibDB formats
    """

    dist_coeff: NDArray
    """
    Distortion Coefficients. Must be a 1x5 matrix
    """

    cam_mat: NDArray
    """
    Camera Matrix. Must be 3x3
    """

    resolution_width: int
    """
    Max resolution width of the camera
    """

    resolution_height: int
    """
    Max resolution height of the camera
    """

    def __init__(self, calibration: dict) -> None:
        camera_matrix = calibration["camera_matrix"]
        distortion_coefficients = calibration["distortion_coefficients"]

        if len(camera_matrix) != 3 and len(camera_matrix[0]) != 3:
            raise ValueError("Camera Matrix is not of size 3x3")

        if len(distortion_coefficients) != 5:
            raise ValueError("Distortion Coefficients are not of size 1x5")

        self.dist_coeff = np.array(distortion_coefficients, dtype=np.float32)
        self.cam_mat = np.array(camera_matrix, dtype=np.float32)
        self.resolution_width = calibration["img_size"][0]
        self.resolution_height = calibration["img_size"][1]

class FiducialMap:
    """
    Map of all poses of the fiducials in the map for multitag targeting
    """

    tags: list[Fiducial] = []
    """
    List of all the tags
    """

    field_width: float
    """
    Width of the field
    """

    field_length: float
    """
    Length of the field
    """

    def __init__(self, wpi_tag_data: dict) -> None:
        self.field_length = wpi_tag_data["field"]["length"]
        self.field_width = wpi_tag_data["field"]["width"]

        for tag in wpi_tag_data["tags"]:
            translation = tag["pose"]["translation"]
            rotation = tag["pose"]["rotation"]["quaternion"]

            self.tags.append(
                Fiducial(
                    tag["ID"],
                    Pose3d(
                        Translation3d(
                            translation["x"],
                            translation["y"],
                            translation["z"]
                        ),
                        Rotation3d(
                            Quaternion(
                                rotation["W"],
                                rotation["X"],
                                rotation["Y"],
                                rotation["Z"]
                            )
                        )
                    )
                )
            )

    def get_tag_by_id(self, tag_id: int) -> Fiducial | None:
        for tag in self.tags:
            if tag.id == tag_id:
                return tag
        return None

class ConfigManager:
    """
    Container for all hardware and spatial configs
    """

    fiducial_map: FiducialMap
    calibration_data: CameraCalibration

    def __init__(self, tag_file_path: str, calibration_path: str) -> None:
        self.fiducial_map = FiducialMap(self.__load_json(tag_file_path))
        self.calibration_data = CameraCalibration(self.__load_json(calibration_path))

    def __load_json(self, file_path: str) -> dict:
        """
        Loads json

        Args:
            file_path (str): path of the json file

        Returns:
            dict: The deserialized json dict
        """
        with open(file_path, 'r') as file:
            return json.load(file)


