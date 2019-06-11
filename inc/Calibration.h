#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

enum CalibrationType
{
    SINGLE,
    STEREO
};

class Calibration
{
public:
    struct Settings
    {
        std::string outfile_name;
        CalibrationType type;

        int flags = 0;
        bool bUseCalibrated = false;
    };

private:
    struct Input
    {
        cv::Size grid_size;
        float grid_square_size;

        cv::Size image_size;
        std::vector<std::string> images[2];
        std::vector<std::vector<cv::Point3f>> object_points;
        std::vector<std::vector<cv::Point2f>> image_points[2];
    };

    struct Result
    {
        cv::Mat CameraMatrix[2];
        cv::Mat DistCoeffs[2];
        cv::Mat OptimalMatrix[2];
        int n_image_pairs;
        std::vector<std::string> good_images;
        std::vector<std::vector<cv::Point3f>> object_points;
        std::vector<std::vector<cv::Point2f>> undistorted_points[2];

        cv::Mat R, T;
        cv::Mat R1, R2, Q, P1, P2, E, F;
    };

public:
    Calibration(Settings);

    void RunCalibration();
    void ShowRectifiedImage();

private:
    void SingleCalibrate();
    void StereoCalibrate();
    void GetImagePoints();
    void UndistortPoints();
    void TriangulatePoints();
    void TransformLocalToWorld();

private:
    Input input;
    Result result;
    Settings settings;
};
