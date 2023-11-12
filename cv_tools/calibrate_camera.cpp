#include <iostream>
#include <fstream>
#include <stdio.h>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/*
 * This is a helper script to calibrate intrinsic camera parameters.
 * For each camera to be used in a system, this script should be called ONLY after
 * taking multiple images of a calibration object (i.e. Chess board) with the camera to be
 * calibrated.
 *
 * After taking multiple (15 - 50) images from varying perspectives of the same Chess board, make sure
 * to save them in the directory included in this project (`./cv_tools/calibration_images/`).
 * Saving the images here allows this script to read in each image and perform the necessary steps
 * to calibrate the camera used.
 *
 * The end result is a file generated in the root directory of this project:
 * `./camera_params.yml`
 *
 * This yml file is a configuration file with two important values now included:
 *
 * `camera_matrix` values.
 * `distortion_coefficients` values.
 *
 * These parameters will be read in when running the main file of this project (the actual application).
 * When a frame is read in from the camera, the `undistort_image` method is called in order to retrieve an
 * undistorted version of that frame. All subsequent processing is performed on this more accurate representation
 * of the camera's input (the scene in which the camera is capturing frames from).
 *
 * !!!
 * To run this script, simply run the pre-compiled `./calibrate_camera` script from the command-line,
 * but REMEMBER:
 * You can only run this after the prerequisite step of taking multiple images with the camera to be calibrated
 * and those images placed in the calibration_images directory.
 * !!!
 *
 */

int write_params_to_file(cv::Mat camera_matrix, cv::Mat distortion_coefficients, bool is_stereo)
{
    std::ofstream outfile("../camera_params.yml");

    if (!is_stereo)
    {
        outfile << "camera_matrix: " << camera_matrix << std::endl;
        outfile << "distortion_coefficients: " << distortion_coefficients << std::endl;
        outfile.close();

        return 0;
    }

    // outfile << "camera_matrix_left: " << camera_matrix_left << std::endl;
    // outfile << "distortion_coefficients_left: " << distortion_coefficients_left << std::endl;

    // outfile << "camera_matrix_right: " << camera_matrix_right << std::endl;
    // outfile << "distortion_coefficients_right: " << distortion_coefficients_right << std::endl;
}

// Defining the dimensions of checkerboard
int CHECKERBOARD[2]{6, 9};

int main()
{
    // Creating vector to store vectors of 3D points for each checkerboard image
    std::vector<std::vector<cv::Point3f>> objpoints;

    // Creating vector to store vectors of 2D points for each checkerboard image
    std::vector<std::vector<cv::Point2f>> imgpoints;

    // Defining the world coordinates for 3D points
    std::vector<cv::Point3f> objp;
    for (int i{0}; i < CHECKERBOARD[1]; i++)
    {
        for (int j{0}; j < CHECKERBOARD[0]; j++)
            objp.push_back(cv::Point3f(j, i, 0));
    }

    // Extracting path of individual image stored in a given directory
    std::vector<cv::String> images;
    // Path of the folder containing checkerboard images
    std::string path = "./calibration_images/*.jpg";

    cv::glob(path, images);

    cv::Mat frame, gray;
    // vector to store the pixel coordinates of detected checker board corners
    std::vector<cv::Point2f> corner_pts;
    bool success;

    // Looping over all the images in the directory
    for (int i{0}; i < images.size(); i++)
    {
        frame = cv::imread(images[i]);
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // Finding checker board corners
        // If desired number of corners are found in the image then success = true
        success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

        /*
         * If desired number of corner are detected,
         * we refine the pixel coordinates and display
         * them on the images of checker board
         */
        if (success)
        {
            cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);

            // refining pixel coordinates for given 2d points.
            cv::cornerSubPix(gray, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);

            // Displaying the detected corner points on the checker board
            cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

            objpoints.push_back(objp);
            imgpoints.push_back(corner_pts);
        }

        cv::imshow("Image", frame);
        cv::waitKey(0);
    }

    cv::destroyAllWindows();

    cv::Mat cameraMatrix, distCoeffs, R, T;

    /*
     * Performing camera calibration by
     * passing the value of known 3D points (objpoints)
     * and corresponding pixel coordinates of the
     * detected corners (imgpoints)
     */
    cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows, gray.cols), cameraMatrix, distCoeffs, R, T);

    std::cout << "camera_matrix: " << cameraMatrix << std::endl;
    std::cout << "distortion_coeffecients: " << distCoeffs << std::endl;
    std::cout << "rVecs: " << R << std::endl;
    std::cout << "tVecs: " << T << std::endl;

    write_params_to_file(cameraMatrix, distCoeffs, false);

    return 0;
}