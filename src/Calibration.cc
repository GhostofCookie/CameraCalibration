#include <Calibration.h>
#include <iostream>

Calibration::Calibration(CalibrationType type, std::string outfile)
{
    this->type              = type;
    this->outfile_name      = outfile;
    input.grid_size         = cv::Size(19, 11);
    input.grid_square_size  = 6.f;

    // TODO: Make this more dynamic.
    cv::glob("./LeftImages/*.JPG", input.images[0], false);
    cv::glob("./RightImages/*.JPG", input.images[1], false);
}

void Calibration::GetImagePoints()
{
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
                        objs.push_back(cv::Point3f((float)j * input.grid_square_size, (float)i * input.grid_square_size, 0));

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
        flags |= cv::CALIB_TILTED_MODEL;

        double calib = calibrateCamera(input.object_points, input.image_points[i], input.image_size, result.CameraMatrix[i], result.DistCoeffs[i], rvecs, tvecs, flags);
        std::cout << "  > Calibration Result: " << calib << std::endl;

        cv::FileStorage fs("calib_camera_" + std::to_string(i) + ".yaml", cv::FileStorage::WRITE);
        fs << "K" << result.CameraMatrix[i];
        fs << "D" << result.DistCoeffs[i];
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

    flags = 0;
    flags |= cv::CALIB_FIX_INTRINSIC;
    /*    
    {
        flags |= CALIB_FIX_ASPECT_RATIO;
        flags |= CALIB_FIX_PRINCIPAL_POINT;
        flags |= CALIB_ZERO_TANGENT_DIST;
        flags |= CALIB_SAME_FOCAL_LENGTH;
        flags |= CALIB_FIX_K3;
        flags |= CALIB_FIX_K4;
        flags |= CALIB_FIX_K5;
        flags |= CALIB_USE_INTRINSIC_GUESS;
    } 
    */

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
                                     flags,
                                     cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-5));

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
                      cv::CALIB_ZERO_DISPARITY, -1, input.image_size);

    // Save calibration to yaml file to be used elswhere.
    cv::FileStorage fs(outfile_name, cv::FileStorage::WRITE);
    fs << "K1" << result.CameraMatrix[0];
    fs << "D1" << result.DistCoeffs[0];
    fs << "K2" << result.CameraMatrix[1];
    fs << "D2" << result.DistCoeffs[1];
    fs << "E" << result.E;
    fs << "F" << result.F;
    fs << "P1" << result.P1;
    fs << "R1" << result.R1;
    fs << "P2" << result.P2;
    fs << "R2" << result.R2;
    fs << "grid_size" << input.grid_size;
    fs << "grid_square_size" << input.grid_square_size;
    fs << "image_size" << input.image_size;
    std::cout << "=== Finished Stereo Calibration ===" << std::endl;
}

void Calibration::ShowRectifiedImage()
{
    for (int i = 0; i < result.good_images.size(); i++)
    {
        for(int j = 0; j < 2; j++)
        {
            cv::Mat img, undist_img, rect_img;
            img = cv::imread(result.good_images[i]);

            undistort(img, rect_img, result.CameraMatrix[j], result.DistCoeffs[j]);
            cv::imshow("Rectified", rect_img); 
            char c = cv::waitKey(25);
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
    cv::Mat P1_32FC1, P2_32FC1;
    
    result.P1.convertTo(P1_32FC1, CV_32FC1);
    result.P2.convertTo(P2_32FC1, CV_32FC1);

    std::cout << "  > Triangulating points..." << std::endl;
    cv::Mat projection_points[result.n_image_pairs];
    for (int i = 0; i < result.n_image_pairs; i++)
        cv::triangulatePoints(P1_32FC1, P2_32FC1,
                              result.undistorted_points[0][i],
                              result.undistorted_points[1][i],
                              projection_points[i]);

    // Projection:
    // <x, y, z, w> -> <x/w, y/w, z/w> 
    result.object_points.resize(result.n_image_pairs);
    for (int i = 0; i < result.n_image_pairs; i++)
        for(int j = 0; j < result.undistorted_points[0][0].size(); j++)
        {
            result.object_points[i].push_back(cv::Point3f(
                projection_points[i].at<float>(0, j) / projection_points[i].at<float>(3,j),
                projection_points[i].at<float>(1, j) / projection_points[i].at<float>(3,j),
                projection_points[i].at<float>(2, j) / projection_points[i].at<float>(3,j)
            ));
        }
  
    std::cout << "  > Transforming points to world coordinates..." << std::endl;
    TransformLocalToWorld();

    cv::FileStorage fs("ObjectPoints.yaml", cv::FileStorage::WRITE);
    fs << "object_points" << result.object_points;

    std::cout << "=== Finished Triangulation ===" << std::endl;
}

void Calibration::TransformLocalToWorld()
{
    // Rotate to world
    {
        float x_rot_degrees = 0.f;
        float x_rot_radians = x_rot_degrees * CV_PI / 180.f;

        // Create a 3x3 X-Rotation matrix.
        // |1   0   0 |
        // |0  cos sin|
        // |0 -sin cos|
        cv::Mat rot_matrix = (cv::Mat_<float>(3, 3) << 1, 0, 0, 0, cos(x_rot_radians), sin(x_rot_radians), 0, -sin(x_rot_radians), cos(x_rot_radians));

        for(int i = 0; i < result.object_points.size(); i++)
            for(int j = 0; j < result.object_points[i].size(); j++)
            {
                result.object_points[i][j] = cv::Point3f(
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
            }
    }

    // Translate to world.
    {
        cv::Point3f offset(
            result.object_points[0][0].x,
            result.object_points[0][0].y,
            result.object_points[0][0].z);

        // TODO: This should be determined.
        cv::Point3f translation(0, 0, 0);

        for(int i = 0; i < result.object_points.size(); i++)
            for(int j = 0; j < result.object_points[i].size(); j++)
            {
                result.object_points[i][j] = cv::Point3f(
                    result.object_points[i][j].x - offset.x + translation.x,
                    result.object_points[i][j].y - offset.y + translation.y,
                    result.object_points[i][j].z - offset.z + translation.z
                );
            }
    }
}