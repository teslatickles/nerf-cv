#include "../include/StereoVision.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <cstdlib>

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/all_layers.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/persistence.hpp>

cv::Mat StereoVision::add_hsv_filter(cv::Mat &frame, int camera)
{
    // reduce noise
    GaussianBlur(frame, frame, {5, 5}, 0);
    cvtColor(frame, frame, cv::COLOR_BGR2HSV);

    cv::Mat mask;

    // nerf dart orange (rgb) = 255,117,0
    std::vector<int> nerfdart_lower_limit_left = {0, 117, 255};
    std::vector<int> nerfdart_upper_limit_left = {255, 255, 255};
    std::vector<int> nerfdart_lower_limit_right = {0, 117, 255};
    std::vector<int> nerfdart_upper_limit_right = {255, 255, 255};

    camera == 1 ? inRange(frame, nerfdart_lower_limit_right, nerfdart_upper_limit_right, mask)
                : inRange(frame, nerfdart_lower_limit_left, nerfdart_upper_limit_left, mask);

    erode(mask, mask, (3, 3));
    dilate(mask, mask, (3, 3));

    return mask;
}

cv::Point StereoVision::find_dart(cv::Mat &frame, cv::Mat &mask)
{
    std::vector<std::vector<cv::Point>> contours;

    sort(contours.begin(), contours.end(), [](const std::vector<cv::Point> &c1, const std::vector<cv::Point> &c2)
         { return cv::contourArea(c1, false) < cv::contourArea(c2, false); });

    if (contours.size() > 0)
    {
        std::vector<cv::Point> largestContour = contours[contours.size() - 1];
        cv::Point2f center;
        float radius;
        minEnclosingCircle(largestContour, center, radius);
        cv::Moments m = moments(largestContour);
        cv::Point centerPoint(m.m10 / m.m00, m.m01 / m.m00);

        if (radius > 10)
        {
            circle(frame, center, int(radius), (0, 255, 255), 2);
            circle(frame, centerPoint, 5, (0, 0, 255), -1);
        }

        return centerPoint;
    }
    return {0, 0};
}

void StereoVision::detect_objects(cv::Mat &frame)
{
    std::vector<std::string> class_names;
    std::ifstream ifs(std::string("../dnn/obj_detection_coco.txt").c_str());
    std::string line;

    while (getline(ifs, line))
    {
        class_names.push_back(line);
    }

    auto nn_model = cv::dnn::readNet("../dnn/frozen_inference_graph.pb",
                                     "../dnn/ssd_mobilenet_v2_coco_2018_03_29.pbtxt.txt", "TensorFlow");

    int frame_height = frame.cols;
    int frame_width = frame.rows;

    cv::Mat blob = cv::dnn::blobFromImage(frame, 1.0, cv::Size(300, 300), cv::Scalar(127.5, 127.5, 127.5), true, false);
    nn_model.setInput(blob);
    cv::Mat output = nn_model.forward();

    cv::Mat detectionMat(output.size[2], output.size[3], CV_32F, output.ptr<float>());

    for (int i = 0; i < detectionMat.rows; i++)
    {
        int class_id = detectionMat.at<float>(i, 1);
        float confidence = detectionMat.at<float>(i, 2);

        // Check if the detection is of good quality
        if (confidence > 0.4)
        {
            int box_x = static_cast<int>(detectionMat.at<float>(i, 3) * frame.cols);
            int box_y = static_cast<int>(detectionMat.at<float>(i, 4) * frame.rows);
            int box_width = static_cast<int>(detectionMat.at<float>(i, 5) * frame.cols - box_x);
            int box_height = static_cast<int>(detectionMat.at<float>(i, 6) * frame.rows - box_y);

            rectangle(frame, cv::Point(box_x, box_y), cv::Point(box_x + box_width, box_y + box_height), cv::Scalar(255, 255, 255), 2);
            putText(frame, class_names[class_id - 1].c_str(), cv::Point(box_x, box_y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
        }
    }
}

float StereoVision::find_depth(cv::Point circle_left, cv::Point circle_right, cv::Mat &frame_left, cv::Mat &frame_right)
{
    int focal_pixels = 0;

    if (frame_right.cols == frame_left.cols)
    {
        focal_pixels = (frame_right.cols * 0.5) / tan(alpha * 0.5 * CV_PI / 180.0);
    }
    else
    {
        std::cout << "Left and Right camera frames do not have the same pixel width" << std::endl;
    }

    int disparity = circle_left.x - circle_right.x;
    float z_depth = (baseline * float(focal_pixels)) / float(disparity); // depth in cm

    return abs(z_depth);
}

cv::Mat StereoVision::undistort_frame(cv::Mat &frame)
{
    cv::Mat undistorted_frame;
    cv::undistort(frame, undistorted_frame, CAMERA_MATRIX, DISTORTION_COEFFICIENTS);
    return undistorted_frame;
}

void StereoVision::get_camera_parameters(bool is_stereo)
{
    cv::FileStorage fs;
    fs.open(CAM_PARAMS_FILE, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cerr << "Failed to open CAM_PARAMS_FILE!" << std::endl;
        exit(EXIT_FAILURE);
    }

    if (is_stereo)
    {
        // Left Camera
        fs["camera_matrix_left"] >> CAMERA_MATRIX_LEFT;
        fs["distortion_coefficients_left"] >> DISTORTION_COEFFICIENTS_LEFT;

        // Right Camera
        fs["camera_matrix_right"] >> CAMERA_MATRIX_RIGHT;
        fs["distortion_coefficients_right"] >> DISTORTION_COEFFICIENTS_RIGHT;
    }
    else
    {
        // single camera
        fs["camera_matrix"] >> CAMERA_MATRIX;
        fs["distortion_coefficients"] >> DISTORTION_COEFFICIENTS;
    }

    fs.release();
}
