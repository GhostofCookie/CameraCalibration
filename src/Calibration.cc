#include <Calibration.h>

#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>

using namespace cv;

Calibration::Calibration()
{
}

Calibration::~Calibration()
{
}

void Calibration::GetImagePoints()
{
    for (auto image : input.images)
    {
        Mat frame = imread(image, IMREAD_GRAYSCALE);

        std::vector<Point2f> buffer;

        bool found = findCirclesGrid(frame, Size(input.grid_cols, input.grid_rows), buffer, CALIB_CB_SYMMETRIC_GRID);

        if (!buffer.empty() && found)
        {
            drawChessboardCorners(frame, Size(input.grid_cols, input.grid_rows), Mat(buffer), true);

            std::vector<Point3f> objs;
            for (int i = 0; i < input.grid_rows; i++)
                for (int j = 0; j < input.grid_cols; j++)
                    objs.push_back(Point3f((float)j * (input.grid_square_size + input.grid_gap), (float)i * (input.grid_square_size + input.grid_gap), 0));

            //input.image_points.push_back(buffer);
            input.object_points.push_back(objs);
        }
    }
}

void Calibration::RunCalibration()
{
    std::cout << "=== Starting Calibration ===" << std::endl;
    std::cout << "  > Searching frames for targets..." << std::endl;

    std::cout << "  > Calibrating..." << std::endl;

    Mat K, D;
    std::vector<Mat> rvecs, tvecs;
    settings.flags |= fisheye::CALIB_FIX_K2;
    settings.flags |= fisheye::CALIB_FIX_K3;
    settings.flags |= fisheye::CALIB_FIX_K4;
    settings.flags |= fisheye::CALIB_RECOMPUTE_EXTRINSIC;

    double calib = fisheye::calibrate(input.object_points, input.image_points[0], input.image_size, K, D, rvecs, tvecs, settings.flags);
    std::cout << "  > Calibration Result: " << calib << std::endl;

    Mat opt_matrix;
    fisheye::estimateNewCameraMatrixForUndistortRectify(K, D, input.image_size, noArray(), opt_matrix, 0.5, input.image_size, 0.9);

    FileStorage fs("", FileStorage::WRITE);
    fs << "K" << K;
    fs << "D" << D;
    fs << "OptimalMatrix" << opt_matrix;
    fs << "grid_cols" << input.grid_cols;
    fs << "grid_rows" << input.grid_rows;
    fs << "grid_square_size" << input.grid_square_size;
    fs << "resolution" << input.image_size;
    std::cout << "=== Finished Calibration ===" << std::endl;
}