#include "src/StereoVision.cpp"
#include <opencv2/highgui/highgui.hpp>

void debug_mono_run()
{
    cv::VideoCapture cap(cv::CAP_V4L2);

    float baseline = 7.0;
    float focal_length = 6.0;
    float alpha = 56.6;

    StereoVision stereo_vision(baseline, alpha, focal_length);

    if (!cap.isOpened())
    {
        std::cerr << "Failed to open Camera" << std::endl;
        exit(EXIT_FAILURE);
    }

    cv::Point circle;

    cv::Mat frame;
    while (true)
    {
        cap.read(frame);

        // NEED TO CALIBRATE CAMERA FIRST!
        // cv::Mat ud_frame;
        // ud_frame = stereo_vision.undistort_frame(frame);

        cv::Mat mask;
        mask = stereo_vision.add_hsv_filter(frame, 0);

        cv::Mat res_frame;

        bitwise_and(frame, frame, res_frame, mask);

        circle = stereo_vision.find_dart(frame, mask);

        // cv::Point object_rect;
        // object_rect = stereo_vision.detect_objects(frame);

        float dart_depth = 0;
        // float object_depth = 0;

        if (!circle.x)
        {
            putText(frame, "tracking lost!", {75, 50}, cv::FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2);
        }
    }
}

void run_stereo()
{
    cv::VideoCapture cap_left(0);
    cv::VideoCapture cap_right(1);

    float baseline = 7.0;
    float focal_length = 6.0;
    float alpha = 56.6;

    StereoVision stereo_vision(baseline, alpha, focal_length);

    if (!cap_left.isOpened())
    {
        std::cerr << "Failed to open LEFT Camera" << std::endl;
        exit(EXIT_FAILURE);
    }
    if (!cap_right.isOpened())
    {
        std::cerr << "Failed to open RIGHT Camera!" << std::endl;
        exit(EXIT_FAILURE);
    }

    cv::Point circle_left;
    cv::Point circle_right;

    cv::Mat frame_left;
    cv::Mat frame_right;

    cv::Mat ud_frame_left;
    cv::Mat ud_frame_right;

    cap_left.read(frame_left);
    cap_right.read(frame_right);

    ud_frame_left = stereo_vision.undistort_frame(frame_left);
    ud_frame_right = stereo_vision.undistort_frame(frame_right);

    cv::Mat mask_left;
    cv::Mat mask_right;

    mask_left = stereo_vision.add_hsv_filter(ud_frame_left, 0);
    mask_right = stereo_vision.add_hsv_filter(ud_frame_right, 1);

    cv::Mat res_frame_left;
    cv::Mat res_frame_right;

    bitwise_and(ud_frame_left, ud_frame_left, res_frame_left, mask_left);
    bitwise_and(ud_frame_right, ud_frame_right, res_frame_right, mask_right);

    circle_left = stereo_vision.find_dart(ud_frame_left, mask_left);
    circle_right = stereo_vision.find_dart(ud_frame_right, mask_right);

    // cv::Point rect_left;
    // cv::Point rect_right;
    // rect_left = stereo_vision.detect_objects(frame_left);
    // rect_right = stereo_vision.detect_objects(frame_right);

    float dart_depth = 0;
    float object_depth = 0;

    if (!circle_left.x)
    {
        putText(ud_frame_left, "tracking lost!", {75, 50}, cv::FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2);
    }
    else if (!circle_right.x)
    {
        putText(ud_frame_right, "tracking lost!", {75, 75}, cv::FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2);
    }
    else
    {
        dart_depth = stereo_vision.find_depth(circle_left, circle_right, ud_frame_left, ud_frame_right);
        // object_depth = stereo_vision.find_depth(rect_left, rect_right, frame_left, frame_right);
        // calculate_orientation();
    }
}

// g++ -ggdb main.cpp -o main `pkg-config --cflags --libs opencv4`
int main()
{
    // run_stereo();
    debug_mono_run();
    return 0;
}
