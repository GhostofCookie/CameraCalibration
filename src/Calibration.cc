#include <Calibration.h>
#include <iostream>

using namespace cv;

Calibration::Calibration(Calibration::Settings s)
{
    settings = s;
    input.grid_size = Size(19, 11);
    input.grid_square_size = 6.f;
    glob("./LeftImages/*.JPG", input.images[0], false);
    glob("./RightImages/*.JPG", input.images[1], false);
}

void Calibration::GetImagePoints()
{
    int k = 0;
    for (auto images : input.images)
    {
        for (auto image : images)
        {
            Mat frame = imread(image, IMREAD_GRAYSCALE);

            std::vector<Point2f> buffer;

            bool found = findCirclesGrid(frame, Size(input.grid_size.width, input.grid_size.height), buffer);

            if (!buffer.empty() && found)
            {
                //drawChessboardCorners(frame, Size(input.grid_size.width, input.grid_size.height), Mat(buffer), true);

                std::vector<Point3f> objs;
                for (int i = 0; i < input.grid_size.height; i++)
                    for (int j = 0; j < input.grid_size.width; j++)
                        objs.push_back(Point3f((float)j * input.grid_square_size, (float)i * input.grid_square_size, 0));

                input.image_points[k].push_back(buffer);
                if (input.object_points.size() != input.image_points[0].size())
                    input.object_points.push_back(objs);

                if (input.image_size == Size())
                    input.image_size = frame.size();

                result.good_images.push_back(image);
            }
        }
        result.n_image_pairs = result.good_images.size() / 2;
        k++;
    }
}

void Calibration::RunCalibration()
{
    if (settings.type == CalibrationType::STEREO)
    {
        if (settings.bUseCalibrated)
            SingleCalibrate();
        else
            GetImagePoints();
        StereoCalibrate();
        UndistortPoints();
        TriangulatePoints();
    }
    else
        SingleCalibrate();
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

        std::vector<Mat> rvecs, tvecs;
        settings.flags |= CALIB_FIX_K4;
        settings.flags |= CALIB_FIX_K5;
        settings.flags |= CALIB_FIX_K6;
        settings.flags |= CALIB_FIX_PRINCIPAL_POINT;
        settings.flags |= CALIB_TILTED_MODEL;

        double calib = calibrateCamera(input.object_points, input.image_points[i], input.image_size, result.CameraMatrix[i], result.DistCoeffs[i], rvecs, tvecs, settings.flags);
        std::cout << "  > Calibration Result: " << calib << std::endl;

     //   fisheye::estimateNewCameraMatrixForUndistortRectify(result.CameraMatrix[i], result.DistCoeffs[i], input.image_size, noArray(), result.OptimalMatrix[i], 0.5, input.image_size, 0.9);
     //   result.object_points = input.object_points;

        FileStorage fs("calib_camera_" + std::to_string(i) + ".yaml", FileStorage::WRITE);
        fs << "K" << result.CameraMatrix[i];
        fs << "D" << result.DistCoeffs[i];
        fs << "OptimalMatrix" << result.OptimalMatrix[i];
        fs << "grid_size" << input.grid_size;
        fs << "grid_square_size" << input.grid_square_size;
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

    if (settings.bUseCalibrated)
    {
        settings.flags = 0;
        settings.flags |= CALIB_FIX_INTRINSIC;
        settings.flags |= CALIB_CB_ADAPTIVE_THRESH;
    }
    else
    {
        settings.flags |= CALIB_FIX_ASPECT_RATIO;
        settings.flags |= CALIB_FIX_PRINCIPAL_POINT;
        settings.flags |= CALIB_ZERO_TANGENT_DIST;
        settings.flags |= CALIB_SAME_FOCAL_LENGTH;
        settings.flags |= CALIB_TILTED_MODEL;
        settings.flags |= CALIB_FIX_K2;
        settings.flags |= CALIB_FIX_K3;
        settings.flags |= CALIB_FIX_K4;
        settings.flags |= CALIB_FIX_K5;
    }

    if (!settings.bUseCalibrated)
    {
        result.CameraMatrix[0] = cv::initCameraMatrix2D(input.object_points, input.image_points[0], input.image_size, 0);
        result.CameraMatrix[1] = cv::initCameraMatrix2D(input.object_points, input.image_points[1], input.image_size, 0);
    }

    double res = cv::stereoCalibrate(input.object_points,
                                     input.image_points[0],
                                     input.image_points[1],
                                     result.CameraMatrix[0],
                                     result.DistCoeffs[0],
                                     result.CameraMatrix[1],
                                     result.DistCoeffs[1],
                                     input.image_size,
                                     result.R,
                                     result.T,
                                     result.E,
                                     result.F,
                                     settings.flags,
                                     TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));

    std::cout << "  > Stereo Calibration Result: " << res << std::endl;

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
                      CALIB_ZERO_DISPARITY, -1, input.image_size);

    FileStorage fs(settings.outfile_name, FileStorage::WRITE);
    fs << "K1" << result.CameraMatrix[0];
    fs << "D1" << result.DistCoeffs[0];
    fs << "K2" << result.CameraMatrix[1];
    fs << "D2" << result.DistCoeffs[1];
    fs << "E" << result.E;
    fs << "F" << result.F;
    fs << "grid_size" << input.grid_size;
    fs << "grid_square_size" << input.grid_square_size;
    fs << "image_size" << input.image_size;
    std::cout << "=== Finished Stereo Calibration ===" << std::endl;
}

void Calibration::ShowRectifiedImage()
{
    cv::Mat rectify_map[2][2];

    initUndistortRectifyMap(result.CameraMatrix[0],
                            result.DistCoeffs[0],
                            result.R1,
                            result.P1,
                            input.image_size,
                            CV_16SC2,
                            rectify_map[0][0],
                            rectify_map[0][1]);

    initUndistortRectifyMap(result.CameraMatrix[1],
                            result.DistCoeffs[1],
                            result.R2,
                            result.P2,
                            input.image_size,
                            CV_16SC2,
                            rectify_map[1][0],
                            rectify_map[1][1]);
                            
    for (int i = 0; i < result.good_images.size(); i++)
    {
        for(int j = 0; j < 2; j++)
        {
            cv::Mat img, undist_img, rect_img;
            img = imread(result.good_images[i]);

            remap(img, rect_img, rectify_map[j][0], rectify_map[j][1], INTER_LINEAR);
            imshow("Rectified", rect_img); 
            char c = waitKey(25);
            if(c=27) continue;
        }
    }
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
    cv::Mat PP1, PP2;
    
    result.P1.convertTo(PP1, CV_32FC1);
    result.P2.convertTo(PP2, CV_32FC1);

    std::cout << "  > Triangulating points..." << std::endl;
    cv::Mat object_points_4D[result.n_image_pairs];
    for (int i = 0; i < result.n_image_pairs; i++)
    {
        cv::triangulatePoints(PP1, PP2,
                            result.undistorted_points[0][i],
                            result.undistorted_points[1][i],
                            object_points_4D[i]);
    }
 
    // Projection
    // <x, y, z, w> -> <x/w, y/w, z/w> 
    result.object_points.resize(result.n_image_pairs);
    for (int i = 0; i < result.n_image_pairs; i++)
        for(int j = 0; j < result.undistorted_points[0][0].size(); j++)
        {
            Point3f newPoint(
                object_points_4D[i].at<float>(0, j) / object_points_4D[i].at<float>(3,j),
                object_points_4D[i].at<float>(1, j) / object_points_4D[i].at<float>(3,j),
                object_points_4D[i].at<float>(2, j) / object_points_4D[i].at<float>(3,j)
            );
            result.object_points[i].push_back(newPoint);
        }
  
    std::cout << "  > Transforming points to world coordinates..." << std::endl;
    TransformLocalToWorld();

    FileStorage fs("ObjectPoints.yaml", FileStorage::WRITE);
    fs << "object_points" << result.object_points;

    std::cout << "=== Finished Triangulation ===" << std::endl;
}

void Calibration::TransformLocalToWorld()
{
    // Rotate to world
    {
        float x_rot = 0.f; // Degrees
        float x_rot_radians = x_rot * CV_PI / 180.f;

        // Create a 3x3 X-Rotation matrix.
        // |1   0   0 |
        // |0  cos sin|
        // |0 -sin cos|
        cv::Mat rot_matrix = (Mat_<float>(3, 3) << 1, 0, 0, 0, cos(x_rot_radians), sin(x_rot_radians), 0, -sin(x_rot_radians), cos(x_rot_radians));

        for(int i = 0; i < result.object_points.size(); i++)
            for(int j = 0; j < result.object_points[i].size(); j++)
            {
                cv::Point3f newPoint(
                    rot_matrix.at<float>(0) * result.object_points[i][j].x +
                    rot_matrix.at<float>(3) * result.object_points[i][j].y +
                    rot_matrix.at<float>(6) * result.object_points[i][j].z,
                    rot_matrix.at<float>(1) * result.object_points[i][j].x +
                    rot_matrix.at<float>(4) * result.object_points[i][j].y +
                    rot_matrix.at<float>(7) * result.object_points[i][j].z,
                    rot_matrix.at<float>(2) * result.object_points[i][j].x +
                    rot_matrix.at<float>(5) * result.object_points[i][j].y +
                    rot_matrix.at<float>(8) * result.object_points[i][j].z
                );
                result.object_points[i][j] = newPoint;
            }
    }

    // Translate to world.
    {
        // TODO: Find out why [0][199] is best.
        cv::Point3f offset(
            result.object_points[0][199].x,
            result.object_points[0][199].y,
            result.object_points[0][199].z);

        // TODO: This should be determined.
        cv::Point3f translation(0, 0, 0);

        for(int i = 0; i < result.object_points.size(); i++)
            for(int j = 0; j < result.object_points[i].size(); j++)
            {
                cv::Point3f newPoint(
                    result.object_points[i][j].x - offset.x + translation.x,
                    result.object_points[i][j].y - offset.y + translation.y,
                    result.object_points[i][j].z - offset.z + translation.z
                );
                result.object_points[i][j] = newPoint;
            }
    }
}