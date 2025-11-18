from dataclasses import dataclass
from numpy import float64
from numpy.typing import NDArray 

@dataclass(frozen=True)
class Fiducial:
    id: int
    corners: NDArray[float64]
