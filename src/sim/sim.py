from dataclasses import dataclass
from pathlib import Path
from raylib.enums import PixelFormat

import typing
import cv2
import pyray as pr
import numpy as np
import numpy.typing as npt
import json


# Basis change from WPILib/FRC (X forward, Y left, Z up) to Raylib
# rX horizontal, Y up, Z forward): (x, y, z) -> (y, z, x).
_FRC_TO_RAYLIB_BASIS = pr.Vector4(-0.5, -0.5, -0.5, 0.5)

# Move only the rendered tag surface off the field/wall to avoid coplanar
# depth-buffer artifacts.  Pose estimation still uses the map's true pose.
_TAG_RENDER_SURFACE_OFFSET_METERS = 0.001


@dataclass
class Pose:
    pos: pr.Vector3
    rot: pr.Vector4


@dataclass(frozen=True)
class SimFiducial:
    pose: Pose
    render_pose: Pose
    id: int
    img: npt.NDArray[np.uint8]
    model: pr.Model


class Simulation:
    camera: pr.Camera3D
    map_path: Path
    map_size: pr.Vector2  # width, length (meters)
    field_center: pr.Vector2  # width, length (meters)
    fids: list[SimFiducial]
    fid_models: list[pr.Model]
    fid_textures: list[pr.Texture]
    field_model: pr.Model
    aruco_dictionary: cv2.aruco.Dictionary
    window_size: tuple[int, int]

    def __init__(
        self,
        window_size: tuple[int, int] = (1920, 1080),
        map_path: Path = Path("../maps/2026-rebuilt-welded.json"),
        aruco_dict: cv2.aruco.Dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_APRILTAG_36h11
        ),
    ):
        self.camera = pr.Camera3D(
            pr.Vector3(0.0, 0.3, 0.0),
            pr.Vector3(0.0, 0.0, -1.0),
            pr.Vector3(0.0, 1.0, 0.0),
            45.0,
            pr.CameraProjection.CAMERA_PERSPECTIVE,
        )
        self.camera.target = pr.vector3_add(self.camera.position, pr.Vector3(0, 0, -1))
        self.map_path = map_path
        self.aruco_dictionary = aruco_dict
        self.fids = []
        self.window_size = window_size

        # These are solely used for resource storage, use actual SimFiducial
        self.fid_models = []
        self.fid_textures = []

    def begin(self) -> None:
        pr.set_trace_log_level(pr.TraceLogLevel.LOG_ERROR)
        # TODO: Simulation is running mad slow, need to multithread? Could be a python issue too?
        pr.set_target_fps(120)
        pr.init_window(*self.window_size, "Vizion Sim")
        self._configure_environment()

    def update(self, camera_pos: pr.Vector3) -> None:
        pr.begin_drawing()
        self.camera.position = pr.vector3_add(self.camera.position, camera_pos)
        pr.begin_mode_3d(self.camera)
        pr.clear_background(pr.BLACK)
        # pr.draw_cube((0, 0, 0), 1, 1, 1, pr.RED)
        pr.draw_model_ex(
            self.field_model,
            pr.Vector3(0, 0, 0),
            pr.Vector3(0, 1, 0),
            90,
            pr.vector3_one(),
            pr.WHITE,
        )

        for fid in self.fids:
            pr.draw_model(fid.model, fid.render_pose.pos, 1, pr.WHITE)
        pr.end_mode_3d()
        pr.end_drawing()

    def should_close(self) -> bool:
        """Whether or not the simulation window should close

        Returns:
            Whether or not the simulation window should close
        """
        return pr.window_should_close()

    def _to_vector3(self, value: npt.NDArray[np.uint8]) -> pr.Vector3:
        """Converts a numpy equivalent to a pr.Vector3 into a pr.Vector3

        Args:
            value (npt.NDArray[np.uint8]): the original array to convert

        Returns:
            The converted vector as a pr.Vector3
        """
        return pr.Vector3(value[0], value[1], value[2])

    def _generate_marker_with_border(self, id: int) -> npt.NDArray[np.uint8]:
        """Generates a numpy array representing the fiducial.

        Args:
            id (int): id of the tag

        Returns:
            Array of the bits representing the fiducial with a 1 bit border
        """
        raw_fid = self.aruco_dictionary.generateImageMarker(id, 8)

        fin = cv2.copyMakeBorder(raw_fid, 1, 1, 1, 1, cv2.BORDER_CONSTANT, None, 255)

        return typing.cast(npt.NDArray[np.uint8], fin)

    def _frc_position_to_raylib(self, position: dict[str, float]) -> pr.Vector3:
        """Convert an FRC field position into Raylib's world coordinates.

        Args:
            position (dict[str, float]): frc Pose's transform

        Returns:
            A pr.Vector3 conversion of the frc position
        """
        # WPILib maps use the blue-wall corner as their origin.  The
        # AdvantageScope model is centered at the world origin, so translate
        # into that centered field frame before changing axes to Raylib.
        centered_x = position["x"] - self.field_center.y
        centered_y = position["y"] - self.field_center.x
        return pr.Vector3(centered_y, position["z"], centered_x)

    def _frc_rotation_to_raylib(self, quaternion: dict[str, float]) -> pr.Vector4:
        """Express an FRC field rotation in the Raylib coordinate basis.

        Args:
            quaternion (dict[str, float]): tag rotation quaternion

        Returns:
            A pr.Vector4 quaternion conversion of the frc quaternion
        """
        frc_rotation = pr.Vector4(
            quaternion["X"],
            quaternion["Y"],
            quaternion["Z"],
            quaternion["W"],
        )
        inverse_basis = pr.quaternion_invert(_FRC_TO_RAYLIB_BASIS)
        return pr.quaternion_multiply(
            pr.quaternion_multiply(_FRC_TO_RAYLIB_BASIS, frc_rotation),
            inverse_basis,
        )

    def _configure_environment(self) -> None:
        with open(self.map_path, "r") as file:
            data = json.load(file)

            self.map_size = pr.Vector2(data["field"]["width"], data["field"]["length"])
            self.field_center = pr.Vector2(self.map_size.x / 2.0, self.map_size.y / 2.0)

            for tag in data["tags"]:
                translation = tag["pose"]["translation"]
                quaternion = tag["pose"]["rotation"]["quaternion"]

                fid_pose: Pose = Pose(
                    pos=self._frc_position_to_raylib(translation),
                    rot=self._frc_rotation_to_raylib(quaternion),
                )

                fid_img = self._generate_marker_with_border(tag["ID"])

                fid_model: pr.Model = self._generate_model_for_tag(fid_img, fid_pose)

                fid = SimFiducial(
                    pose=fid_pose,
                    render_pose=self._get_render_pose(fid_pose, fid_model.transform),
                    model=fid_model,
                    id=tag["ID"],
                    img=self._generate_marker_with_border(tag["ID"]),
                )

                self.fids.append(fid)

            self.field_model = pr.load_model(
                "/Users/vismayb/Library/Application Support/AdvantageScope/autoAssets/Field3d_2026FRCFieldV1/model.glb"
            )

    def _get_render_pose(self, fid_pose: Pose, model_transform: pr.Matrix) -> Pose:
        # GenMeshPlane's local face normal is +Y.  Offset the display
        # mesh along its transformed normal so it does not z-fight
        # with the field surface behind it.
        face_normal = pr.vector3_normalize(
            pr.vector3_transform(pr.Vector3(0.0, 1.0, 0.0), model_transform)
        )
        return Pose(
            pr.vector3_add(
                fid_pose.pos,
                pr.vector3_scale(face_normal, _TAG_RENDER_SURFACE_OFFSET_METERS),
            ),
            fid_pose.rot,
        )

    def _generate_model_for_tag(
        self, fid_img: npt.NDArray[np.uint8], fid_pose: Pose
    ) -> pr.Model:
        height, width = fid_img.shape

        image = pr.Image(
            fid_img,
            width,
            height,
            1,
            int(pr.PixelFormat.PIXELFORMAT_UNCOMPRESSED_GRAYSCALE),
        )

        tex = pr.load_texture_from_image(image)
        pr.set_texture_filter(tex, pr.TextureFilter.TEXTURE_FILTER_POINT)

        mesh: pr.Mesh = pr.gen_mesh_plane(0.2667, 0.2667, 1, 1)
        model: pr.Model = pr.load_model_from_mesh(mesh)

        self.fid_models.append(model)
        self.fid_textures.append(tex)

        model.materials.maps[pr.MaterialMapIndex.MATERIAL_MAP_ALBEDO].texture = tex

        # GenMeshPlane is horizontal (XZ plane).  Rotate it upright so
        # its normal is +Z, then orient it using the converted tag pose.
        upright_plane = pr.matrix_rotate_x(np.pi / 2)
        model.transform = pr.matrix_multiply(
            upright_plane, pr.quaternion_to_matrix(fid_pose.rot)
        )

        return model

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

        for model in self.fid_models:
            pr.unload_model(model)
        for texture in self.fid_textures:
            pr.unload_texture(texture)

        pr.unload_model(self.field_model)
