#include "includes/Calibration.h"

#include <opencv2/stereo.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/ximgproc.hpp>

#include <iostream>

cv::Point3f CalculateCentroid(std::vector<cv::Point3f>);
std::vector<cv::Point3f> TranslatePoints(std::vector<cv::Point3f>, cv::Point3f);

Calibration::Calibration(Input& in, CalibrationType type, std::string outfile)
{
    for(int i = 0; i < 2; i++)
    {
        this->input.images[i] = in.images[i];
        this->input.image_points[i] = in.image_points[i];
    }

    this->input.image_size        = in.image_size;
    this->input.grid_size         = in.grid_size != cv::Size() ? in.grid_size : cv::Size(19, 11);
    this->input.grid_dot_size     = in.grid_dot_size != 0.f ? in.grid_dot_size : 12.1f; // mm

    this->type              = type;
    this->outfile_name      = outfile;
    this->out_dir           = "config/";
}

void Calibration::ReadImages(std::string dir1, std::string dir2)
{
    cv::glob(dir1, input.images[0], false);
    cv::glob(dir2, input.images[1], false);
}

void Calibration::RunCalibration()
{
    if (type == CalibrationType::STEREO)
    {
        SingleCalibrate();
        StereoCalibrate();
        UndistortPoints();
        TriangulatePoints();
    }
    else SingleCalibrate();
}

void Calibration::ReadCalibration()
{
    if(input.image_points[0].empty()) std::cout << "No left image points copied over!\n";
    if(input.image_points[1].empty()) std::cout << "No right image points copied over!\n";
    if(type == CalibrationType::STEREO)
    {
        if(!input.image_points[0].empty() &&!input.image_points[1].empty())
            result.n_image_pairs = (input.image_points[0].size() + input.image_points[1].size()) / 2;
        else GetImagePoints();
        cv::FileStorage fs(out_dir + outfile_name, cv::FileStorage::READ);
        fs["K1"] >> result.CameraMatrix[0];
        fs["D1"] >> result.DistCoeffs[0];
        fs["K2"] >> result.CameraMatrix[1];
        fs["D2"] >> result.DistCoeffs[0];
        fs["K1"] >> result.CameraMatrix[0];
        fs["D1"] >> result.DistCoeffs[0];
        fs["K2"] >> result.CameraMatrix[1];
        fs["D2"] >> result.DistCoeffs[1];
        fs["E"] >> result.E;
        fs["F"] >> result.F;
        fs["R"] >> result.R;
        fs["T"] >> result.T;
        fs["P1"] >> result.P1;
        fs["R1"] >> result.R1;
        fs["P2"] >> result.P2;
        fs["R2"] >> result.R2;

        UndistortPoints();
        TriangulatePoints();
    }
}

void Calibration::GetImagePoints()
{
    std::cout << "  > Finding image points..." << std::endl;
    int k = 0;
    for (auto images : input.images)
    {
        for (auto image : images)
        {
            cv::Mat frame = cv::imread(image, cv::IMREAD_GRAYSCALE);

            std::vector<cv::Point2f> buffer;

            bool found = cv::findCirclesGrid(frame, cv::Size(input.grid_size.width, input.grid_size.height), buffer);

            if (!buffer.empty() && found)
            {
                //drawChessboardCorners(frame, Size(input.grid_size.width, input.grid_size.height), Mat(buffer), true);

                std::vector<cv::Point3f> objs;
                for (int i = 0; i < input.grid_size.height; i++)
                    for (int j = 0; j < input.grid_size.width; j++)
                        objs.push_back(cv::Point3f((float)j * input.grid_dot_size, (float)i * input.grid_dot_size, 0));

                input.image_points[k].push_back(buffer);
                if (input.object_points.size() != input.image_points[0].size())
                    input.object_points.push_back(objs);

                if (input.image_size == cv::Size())
                    input.image_size = frame.size();

                result.good_images.push_back(image);
            }
        }
        result.n_image_pairs = result.good_images.size() / 2;
        k++;
    }
}

void Calibration::SingleCalibrate()
{
    std::cout << "=== Starting Camera Calibration ===\n";

    GetImagePoints();

    int i = 0;
    for (auto images : input.images)
    {
        std::cout << "  > Calibrating Camera " << std::to_string(i) << "...\n";

        if (input.image_points[i].empty())
        {
            std::cout << " !> No image points found for this camera!\n";
            i++;
            continue;
        }

        std::vector<cv::Mat> rvecs, tvecs;
        flags |= cv::CALIB_FIX_ASPECT_RATIO;
        flags |= cv::CALIB_FIX_PRINCIPAL_POINT;
        flags |= cv::CALIB_ZERO_TANGENT_DIST;
        flags |= cv::CALIB_SAME_FOCAL_LENGTH;
        flags |= cv::CALIB_FIX_K3;
        flags |= cv::CALIB_FIX_K4;
        flags |= cv::CALIB_FIX_K5;
        //flags |= cv::CALIB_TILTED_MODEL;

        double calib = calibrateCamera(input.object_points, input.image_points[i], input.image_size, result.CameraMatrix[i], result.DistCoeffs[i], rvecs, tvecs, flags);
        std::cout << "  > Calibration Result: " << calib << std::endl;

        cv::FileStorage fs(out_dir + "calib_camera_" + std::to_string(i) + ".yaml", cv::FileStorage::WRITE);
        fs << "K" << result.CameraMatrix[i];
        fs << "D" << result.DistCoeffs[i];
        fs << "grid_size" << input.grid_size;
        fs << "grid_dot_size" << input.grid_dot_size;
        fs << "resolution" << input.image_size;
        std::cout << "  > Finished Camera " << std::to_string(i) << " calibration\n";

        i++;
    }
    std::cout << "=== Finished Calibration ===" << std::endl;
}

void Calibration::StereoCalibrate()
{
    std::cout << "=== Starting Stereo Calibration ===" << std::endl;
    std::cout << "  > Calibrating stereo..." << std::endl;

    flags = 0;
    flags |= cv::CALIB_FIX_INTRINSIC;
    flags |= cv::CALIB_SAME_FOCAL_LENGTH;

    double rms = cv::stereoCalibrate(input.object_points,
                                     input.image_points[0],
                                     input.image_points[1],
                                     result.CameraMatrix[0],// Intrinsic Matrix 1
                                     result.DistCoeffs[0],
                                     result.CameraMatrix[1],// Intrinsic Matrix 2
                                     result.DistCoeffs[1],
                                     input.image_size,
                                     result.R,              // 3x3 Rotation Matrix
                                     result.T,              // 3x1 Translation Vector (Column Vector)
                                     result.E,              // Essential Matrix
                                     result.F,              // Fundamental Matrix
                                     flags,
                                     cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-5));

    std::cout << "  > Stereo Calibration Result: " << rms << std::endl;

    std::cout << "  > Rectifying stereo..." << std::endl;
    cv::stereoRectify(result.CameraMatrix[0],
                      result.DistCoeffs[0],
                      result.CameraMatrix[1],
                      result.DistCoeffs[1],
                      input.image_size,
                      result.R,
                      result.T,
                      result.R1,
                      result.R2,
                      result.P1,
                      result.P2,
                      result.Q,
                      cv::CALIB_ZERO_DISPARITY, -1, input.image_size);

    // Save calibration to yaml file to be used elswhere.
    cv::FileStorage fs(out_dir + outfile_name, cv::FileStorage::WRITE);
    fs << "K1" << result.CameraMatrix[0];
    fs << "D1" << result.DistCoeffs[0];
    fs << "K2" << result.CameraMatrix[1];
    fs << "D2" << result.DistCoeffs[1];
    fs << "E" << result.E;
    fs << "F" << result.F;
    fs << "R" << result.R;
    fs << "T" << result.T;
    fs << "P1" << result.P1;
    fs << "R1" << result.R1;
    fs << "P2" << result.P2;
    fs << "R2" << result.R2;
    fs << "grid_size" << input.grid_size;
    fs << "grid_dot_size" << input.grid_dot_size;
    fs << "image_size" << input.image_size;
    std::cout << "=== Finished Stereo Calibration ===" << std::endl;
}

void Calibration::GetUndistortedImage()
{   
    for (size_t i = 0; i < result.good_images.size(); i++)
    {
        cv::Mat img, undist_img;
        img = cv::imread(result.good_images[i]);

        if(i < result.good_images.size()/2)
            cv::undistort(img, undist_img, result.CameraMatrix[0], result.DistCoeffs[0]);
        else cv::undistort(img, undist_img, result.CameraMatrix[1], result.DistCoeffs[1]);

        cv::imshow("Rectified (Undistorted) Image", undist_img); 
        char c = cv::waitKey(0);
        if(c==27) continue;
    
    }
}

void Calibration::UndistortImage(cv::Mat& img, int index)
{
    cv::Mat uimg;
    cv::undistort(img, uimg, result.CameraMatrix[index], result.DistCoeffs[index]);
    img = uimg;
}

cv::Mat Calibration::CalculateDisparity(cv::Mat& left_img, cv::Mat& right_img)
{
    UndistortImage(left_img, 0);
    UndistortImage(right_img, 1);

    cv::Ptr<cv::StereoBM> SM = cv::StereoBM::create(160, 5);
    cv::Mat disparity;

    cv::Mat left_grey, right_grey;
    cv::cvtColor(left_img, left_grey, cv::COLOR_BGR2GRAY);
    cv::cvtColor(right_img, right_grey, cv::COLOR_BGR2GRAY);
    SM->compute(left_grey, right_grey, disparity);

    
    cv::Mat D;
    float b = 65.f; // mm
    float f = 17.f; // mm

    D = (b * f) / disparity;
    
    // TODO: Maybe replace with disparity.
    return D;
}

void Calibration::UndistortPoints()
{
    result.undistorted_points[0].resize(input.image_points[0].size());
    result.undistorted_points[1].resize(input.image_points[1].size());

    for (int i = 0; i < result.n_image_pairs; i++)
    {
        cv::undistortPoints(input.image_points[0][i],
                            result.undistorted_points[0][i],
                            result.CameraMatrix[0],
                            result.DistCoeffs[0],
                            result.R1,
                            result.P1);

        cv::undistortPoints(input.image_points[1][i],
                            result.undistorted_points[1][i],
                            result.CameraMatrix[1],
                            result.DistCoeffs[1],
                            result.R2,
                            result.P2);
    }
}

void Calibration::TriangulatePoints()
{
    std::cout << "=== Starting Triangulation ===" << std::endl;
    
    cv::Mat P1_32FC1, P2_32FC1;
    result.P1.convertTo(P1_32FC1, CV_32FC1);
    result.P2.convertTo(P2_32FC1, CV_32FC1);

    std::cout << "  > Triangulating points..." << std::endl;
    cv::Mat homogeneous_points[result.n_image_pairs];
    for (int i = 0; i < result.n_image_pairs; i++)
        cv::triangulatePoints(P1_32FC1, P2_32FC1,
                              result.undistorted_points[0][i], // Left
                              result.undistorted_points[1][i], // Right
                              homogeneous_points[i]);
    
    result.object_points.resize(result.n_image_pairs);
    for(int i = 0; i < result.n_image_pairs; i++)
        cv::convertPointsFromHomogeneous(homogeneous_points[i].t(), result.object_points[i]);

    // TODO: Add code to check between point pairs, and get cv::norm(p1 - p2) to get distance between them.

    // FIXME: Might not even need this step.
    //for(int i = 0; i < result.n_image_pairs; i++)
    //    result.object_points[i]  = TranslatePoints(result.object_points[i], -CalculateCentroid(result.object_points[i]));

    cv::FileStorage fs(out_dir + "ObjectPoints.yaml", cv::FileStorage::WRITE);
    fs << "object_points" << result.object_points;

    std::cout << "=== Finished Triangulation ===" << std::endl;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Helper Functions
/////////////////////////////////////////////////////////////////////////////////////////////

cv::Point3f CalculateCentroid(std::vector<cv::Point3f> points)
{
    cv::Point3f sum;
    for(auto p : points)
        sum += p;

    return (sum / (int)points.size());    
}

std::vector<cv::Point3f> TranslatePoints(std::vector<cv::Point3f> points, cv::Point3f point)
{
    std::vector<cv::Point3f> result(points.size());
    for(size_t i = 0 ; i < points.size(); i++)
        result[i] = points[i] + point;

    return result;
}