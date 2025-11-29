from dataclasses import dataclass
from numpy import float64
from numpy.typing import NDArray 
from wpimath.geometry import Pose3d

@dataclass(frozen=True)
class Fiducial:
    """
    Fiducial Tag. Represented by a id and a pose
    """

    tag_id: int
    """
    ID of the tag
    """

    pose: Pose3d
    """
    Pose of the tag in field space
    """

    type: str = "36h11" # TODO: Need a system for this to change accordingly
    """
    Fiducial Family of the tag
    """

@dataclass(frozen=True)
class FiducialObservation:
    tag: Fiducial
    corners: NDArray[float64]
