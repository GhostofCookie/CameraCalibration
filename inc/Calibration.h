#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

struct CalibrationSettings
{
    std::string outfile_name;

    int flags = 0;
    bool bUseCalibrated = false;
};

struct CalibrationInput
{
    int grid_rows;
    int grid_cols;
    float grid_square_size;
    float grid_gap;

    cv::Size image_size;
    std::vector<std::string> images;
    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points[2];
};

struct CalibrationResult
{
    cv::Mat CameraMatrix[2];
    cv::Mat DistCoeffs[2];
    std::vector<std::vector<cv::Point3f>> object_points;
    cv::Mat R, T;
    cv::Mat R1, R2, Q, P1, P2;
};

class Calibration
{
public:
    Calibration();
    ~Calibration();

    void GetImagePoints();
    void RunCalibration();
private:
    CalibrationInput input;
    CalibrationResult result;
    CalibrationSettings settings;

};
