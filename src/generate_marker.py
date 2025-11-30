import cv2
import argparse

def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "-f",
        "--filename",
        type=str,
    )

    parser.add_argument(
        "-i",
        "--id",
        type=int
    )

    parser.add_argument(
        "-s",
        "--size",
        type=int
    )

    args = parser.parse_args()

    aruco_dict: cv2.aruco.Dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)

    marker_id = args.id
    marker_size = args.size
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)

    cv2.imwrite(args.filename, marker_img)

if __name__ == "__main__":
    main()
