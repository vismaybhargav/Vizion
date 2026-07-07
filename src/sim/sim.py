from dataclasses import dataclass
from pathlib import Path
from raylib.enums import PixelFormat

import cv2
import pyray as pr
import numpy as np
import numpy.typing as npt
import json

@dataclass
class Pose:
    pos: pr.Vector3
    rot: pr.Vector4 | pr.Vector3 # can either be a rotation with deg offset/quaternion


@dataclass
class SimFiducial:
    pose: Pose
    id: int
    img: Path


class Simulation:
    """
    Manages the simulation
    """

    window_size: tuple[int, int]  # width, height (pixels)
    camera: pr.Camera3D
    map_path: Path
    map_size: pr.Vector2  # width, length (meters)
    fiducials: list[SimFiducial]

    def __init__(self, window_size: tuple[int, int], map_path: Path):
        self.window_size = window_size
        # TODO: This needs to be initialized to the robot's initial camera offset transform
        self.camera = pr.Camera3D(
            pr.Vector3(10.0, 10.0, 10.0),
            pr.Vector3(0.0, 0.0, 0.0),
            pr.Vector3(0.0, 1.0, 0),
            45.0,
            pr.CameraProjection.CAMERA_PERSPECTIVE,
        )
        self.map_path = map_path
        self._configure_environment()

    def begin(self) -> None:
        pr.init_window(self.window_size[0], self.window_size[1], "Vizion Sim")

    def update(self, camera_pos: npt.NDArray[np.uint8]) -> None:
        self.camera.position = self._to_vector3(camera_pos)

        pr.begin_drawing()
        pr.begin_mode_3d(self.camera)
        pr.clear_background(pr.WHITE)
        pr.draw_plane(
            pr.Vector3(0, 0, 0),
            self.map_size,
            pr.BLACK,
        )
        pr.end_mode_3d()
        pr.end_drawing()

    def should_close(self) -> bool:
        return pr.window_should_close()

    def _to_vector3(self, value: npt.NDArray[np.uint8]) -> pr.Vector3:
        return pr.Vector3(value[0], value[1], value[2])

    # TODO: Generate the marker with the padding in the marker gen script
    def _create_padded_marker_texture(self) -> Path:
        marker_path = Path("../marker.png")
        output_path = Path(__file__).with_name("marker_padded.png")

        marker = cv2.imread(str(marker_path), cv2.IMREAD_GRAYSCALE)
        if marker is None:
            raise FileNotFoundError(f"Could not load marker texture at {marker_path}")

        padding = max(marker.shape) // 5
        padded_marker = cv2.copyMakeBorder(
            marker, padding, padding, padding, padding, cv2.BORDER_CONSTANT, value=255
        )

        cv2.imwrite(str(output_path), padded_marker)
        return output_path

    def _configure_environment(self) -> None:
        with open(self.map_path, "r") as file:
            data = json.load(file)

            self.map_size = pr.Vector2(data["field"]["width"], data["field"]["length"])

            for tag in data["tags"]:
                translation = tag["pose"]["translation"]
                quaternion = tag["pose"]["rotation"]["quaternion"]

                self.fiducials.append(
                    SimFiducial(
                        Pose(
                            pr.Vector3(
                                float(translation["x"]),
                                float(translation["y"]), 
                                float(translation["z"])
                            ),
                            pr.Vector4(
                                float(quaternion["W"),

                            )
                        ),
                            pr.Vector3()),
                            int(tag["ID"]),
                    ),
                )

    # Gets the raylib pixel buffer in OpenCV format.
    def raylib_screen_to_bgr_np(self) -> npt.NDArray[np.uint8]:
        img = pr.load_image_from_screen()

        try:
            pr.image_format(img, int(PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8A8))
            w, h = img.width, img.height

            raw = pr.ffi.buffer(img.data, w * h * 4)

            rgba = np.frombuffer(raw, dtype=np.uint8).reshape((h, w, 4))

            bgr = rgba[:, :, [2, 1, 0]].copy()

            return bgr
        finally:
            pr.unload_image(img)

    def end_simulation(self) -> None:
        pr.close_window()
