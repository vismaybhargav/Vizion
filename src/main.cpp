#include "cmdparser.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/objdetect/aruco_detector.hpp"
#include "opencv2/imgproc/imgproc.hpp"
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

static inline std::string typeStr(const cv::Mat& m) {
    if (m.empty()) return "<empty>";
    int t = m.type(); int depth = t & CV_MAT_DEPTH_MASK; int ch = 1 + (t >> CV_CN_SHIFT);
    const char* d = depth == CV_8U ? "8U" : depth == CV_8S ? "8S" :
                    depth == CV_16U ? "16U" : depth == CV_16S ? "16S" :
                    depth == CV_32S ? "32S" : depth == CV_32F ? "32F" : "64F";
    return std::string(d) + "C" + std::to_string(ch);
}

void calibrate_camera() {

}

void configure_parser(cli::Parser& parser) {
    parser.set_optional<std::string>("gm", "generate marker", "");
    parser.set_optional<bool>("cc", "calibrate", false);
}

int main(int argc, char* argv[]) {
    cli::Parser parser(argc, argv);
    configure_parser(parser);
    parser.run_and_exit_if_error();

    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    if (auto marker_file = parser.get<std::string>("gm"); !marker_file.empty()) {
        generate_marker(dictionary, marker_file);
        return EXIT_SUCCESS;
    }

    if (auto should_calibrate = parser.get<bool>("cc"); should_calibrate) {
        calibrate_camera();
        return EXIT_SUCCESS;
    }

    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        throw std::runtime_error("Can't open camera!");
    }

    cap.set(cv::CAP_PROP_CONVERT_RGB, 1);

    cv::Mat frame, frame_gray, frame_bgr, frame_vis;
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;

    auto detector_parameters = cv::aruco::DetectorParameters();
    cv::aruco::ArucoDetector detector(dictionary, detector_parameters);

    while (true) {
        cap >> frame;

        if (frame.empty()) {
            std::cout << "End of video or frame" << std::endl;
            break;
        }

        std::cout << typeStr(frame) << std::endl;

        if (frame.type() == CV_8UC4) {
            cv::cvtColor(frame, frame_bgr, cv::COLOR_BGRA2BGR);
        } else if (frame.type() == CV_8UC3) {
            frame_bgr = frame.clone();
        } else if (frame.type() == CV_8UC1) {
            cv::cvtColor(frame, frame_bgr, cv::COLOR_GRAY2BGR);
        } else {
            frame.convertTo(frame_bgr, CV_8U);
            if (frame_bgr.channels() == 1) cv::cvtColor(frame_bgr, frame_bgr, cv::COLOR_GRAY2BGR);
        }

        cv::cvtColor(frame_bgr, frame_gray, cv::COLOR_BGR2GRAY);

        marker_ids.clear();
        marker_corners.clear();
        rejected_candidates.clear();

        detector.detectMarkers(frame_gray, marker_ids, marker_corners, rejected_candidates);

        // TODO: Log everything to an endpoint or something maybe through websockets
        frame_vis = frame_bgr.clone();
        if (!marker_ids.empty()) {
            cv::aruco::drawDetectedMarkers(frame_vis, marker_corners, marker_ids);
        }

        if (!rejected_candidates.empty()) {
            const std::vector<int> empty;
            cv::aruco::drawDetectedMarkers(frame_vis, rejected_candidates, empty, cv::Scalar(100, 0, 255));
        }

        cv::imshow("Processed Feed", frame_vis);

        if (cv::waitKey(1) == 'q' || cv::waitKey(1) == 27) {
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();

    return 0;
}