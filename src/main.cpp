#include "opencv2/core/mat.hpp"
#include "opencv2/objdetect/aruco_detector.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"

//#define GENERATE_MARKER
bool validate_file_extension(const std::string& file_name) {
}

void generate_marker(const cv::aruco::Dictionary& dictionary, const std::string& file_name) {
    if (!validate_file_extension(file_name)) {
        throw std::invalid_argument("File extension must be .png or .jpg");
    }

    cv::Mat marker_image;
    cv::aruco::generateImageMarker(dictionary, 23, 200, marker_image, 1);
    cv::imwrite(file_name, marker_image);
}

int main(int argc, char* argv[]) {
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    for (int i = 1; i < argc; i++) {
        if (argv[i] == std::string("--generate-marker")) {
            if (argv[i + 1 ] != )
            generate_marker(dictionary, argv[i + 1]);
        }
    }

    cv::Mat input_image = cv::imread("singlemarkersoriginal.jpg", cv::IMREAD_COLOR_BGR);

    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;
    auto detector_parameters = cv::aruco::DetectorParameters();
    cv::aruco::ArucoDetector detector(dictionary, detector_parameters);

    detector.detectMarkers(input_image, marker_corners, marker_ids, rejected_candidates);

    cv::Mat output_image = input_image.clone();
    cv::aruco::drawDetectedMarkers(output_image, marker_corners, marker_ids);
    cv::imshow("singlemarker", output_image);
    cv::waitKey(0);
    return 0;
}