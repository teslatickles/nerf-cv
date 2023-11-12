#include <opencv2/opencv.hpp>

class StereoVision
{
public:
    StereoVision(float baseline, float alpha, float focal_length)
        : baseline(baseline), alpha(alpha), focal_length(focal_length)
    {
        // this->get_camera_parameters(false);
    }

    void get_camera_parameters(bool is_stereo);
    cv::Mat undistort_frame(cv::Mat &frame);
    cv::Mat add_hsv_filter(cv::Mat &frame, int camera);
    cv::Point find_dart(cv::Mat &frame, cv::Mat &mask);
    void detect_objects(cv::Mat &frame);
    float find_depth(cv::Point circle_left, cv::Point circle_right, cv::Mat &frame_left, cv::Mat &frame_right);

    std::string CAM_PARAMS_FILE = "../camera_params.yml";

    cv::Mat CAMERA_MATRIX;
    cv::Mat DISTORTION_COEFFICIENTS;

    cv::Mat CAMERA_MATRIX_LEFT;
    cv::Mat DISTORTION_COEFFICIENTS_LEFT;

    cv::Mat CAMERA_MATRIX_RIGHT;
    cv::Mat DISTORTION_COEFFICIENTS_RIGHT;

private:
    float baseline = 7.0;
    float focal_length = 6.0;
    float alpha = 56.6;

    bool _DEBUG_ = false;
};
