# from pathlib import Path

import numpy as np

from sim import Simulation


def main() -> None:
    sim = Simulation(window_size=(640, 480))
    sim.begin()

    try:
        while True:
            sim.update(np.array(0))

    except KeyboardInterrupt:
        sim.end_simulation()


if __name__ == "__main__":
    main()
