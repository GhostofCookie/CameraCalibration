#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>


#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

#define CALIBRATION_FILE_NAME "CameraCalibration"

//#define CALIBRATE_VIDEO
//#define STEREO_CALIBRATE


int grid_cols  = 19;   //number of points
int grid_rows  = 11;   //number of points
float dot_size = 6.f;  //mm
float grid_gap = 6.1f; //mm

void Calibrate(vector<Mat>&, vector<vector<Point3f>>&, vector<vector<Point2f>>&);
void FixImage(Mat&);
void StereoCalibrate(vector<Mat>&, vector<Mat>&, vector<vector<Point3f>>&, vector<vector<Point2f>>&, vector<vector<Point2f>>&);

int main(int argc, char** argv)
{
     setUseOptimized(true);

     // Holds the real world target point info.
     vector<vector<Point3f>> object_points;

#ifdef STEREO_CALIBRATE
     // For stereo calibration.
     vector<Mat> left_frames;
     vector<Mat> right_frames;
     vector<vector<Point2f>> left_image_points;
     vector<vector<Point2f>> right_image_points;
#else
     // For regular singular calibration.
     vector<Mat> frames;
     vector<vector<Point2f>> image_points;
#endif  


#ifdef CALIBRATE_VIDEO
     VideoCapture cap(argv[1]);

     auto totalFrames = cap.get(cv::CAP_PROP_FRAME_COUNT);

     cout << "  > Saving frames to buffer..." <<endl;
     for(int i = 0; i < totalFrames; i++)
     {
	  Mat frame;
	  cap.read(frame);
	       
	  if(frame.empty())
	  {
	       i--;
	       continue;
	  }

     #ifdef STEREO_CALIBRATE
          if(!frame.empty())
          {
               left_frames.push_back(frame);
               right_frames.push_back(frame);
          }
     #else
         if(!frame.empty()) frames.push_back(frame);
     #endif
     }
     cap.release();
#elif defined STEREO_CALIBRATE
     vector<String> left_files, right_files;
     glob("./LeftImages/*.JPG", left_files, false);
     glob("./RightImages/*.JPG", right_files, false);
     if(left_files.size() == right_files.size())
          for(int i = 0; i < left_files.size() && i < right_files.size(); i++)
          {
               Mat left_frame = imread(left_files[i], IMREAD_COLOR);
               left_frames.push_back(left_frame);	  
               Mat right_frame = imread(right_files[i], IMREAD_COLOR);
               right_frames.push_back(right_frame);
          }
#else
     vector<String> files;
     glob("./RightImages/*.JPG", files, false);

     for(auto file : files)
     {
	  Mat frame = imread(file, IMREAD_COLOR);
	  frames.push_back(frame);	  
     }
#endif
     
#ifdef STEREO_CALIBRATE
     StereoCalibrate(left_frames, right_frames, object_points, left_image_points, right_image_points);
#else
     Calibrate(frames, object_points, image_points);
#endif

#ifdef CALIBRATE_VIDEO
     cap = VideoCapture(argv[1]);
     int currentFrame = 0;
     while(true)
     {
          Mat frame;

          cap.read(frame);
               
          if(frame.empty()) continue;

          currentFrame++;
          if(currentFrame >= totalFrames) break;

          FixImage(frame);
          imshow("Video", frame);
          
          char c = waitKey(25);
          if(c == 27) break;	 
     }
     cap.release();
#else  
     Mat image = imread(argv[1]);
     FixImage(image);
     imshow("Fixed", image);
     	  
#endif
     
     waitKey(0);
     destroyAllWindows();

     return 0;
}

void Calibrate(vector<Mat>& frames, vector<vector<Point3f>>& object_points, vector<vector<Point2f>>& image_points)
{
     Size frame_resolution;

     cout << "=== Starting Calibration ===" << endl;
     cout << "  > Searching frames for targets..." <<endl;
     for(auto frame : frames)
     {
	  if(!frame.empty())
	  {
	       Mat grey;
	       cvtColor(frame, grey, COLOR_BGR2GRAY);

	       vector<Point2f> buffer;

	       bool found = findCirclesGrid(grey, Size(grid_cols, grid_rows), buffer, CALIB_CB_SYMMETRIC_GRID);

	       if(!buffer.empty() && found)
	       {
		    drawChessboardCorners(frame, Size(grid_cols, grid_rows), Mat(buffer), true);

		    vector<Point3f> objs;
		    for(int i = 0; i < grid_rows; i++)
			 for(int j = 0; j < grid_cols; j++)
			      objs.push_back(Point3f((float)j * (dot_size + grid_gap), (float)i * (dot_size + grid_gap), 0));

		    image_points.push_back(buffer);
		    object_points.push_back(objs);
	       }
	  }
       frame_resolution = frame.size();
     }

     cout << "  > Calibrating..." << endl;
     
     Mat K, D;
     vector< Mat > rvecs, tvecs;
     int flag = 0;
     flag |= fisheye::CALIB_FIX_K2;
     flag |= fisheye::CALIB_FIX_K3;
     flag |= fisheye::CALIB_FIX_K4;
     flag |= fisheye::CALIB_RECOMPUTE_EXTRINSIC;

     double calib = fisheye::calibrate(object_points, image_points, frame_resolution, K, D, rvecs, tvecs, flag);
     cout<< "  > Calibration Result: " << calib << endl;

     Mat opt_matrix;
     fisheye::estimateNewCameraMatrixForUndistortRectify(K, D, frame_resolution, noArray(), opt_matrix, 0.5, frame_resolution, 0.9);

     FileStorage fs(CALIBRATION_FILE_NAME, FileStorage::WRITE);
     fs << "K" << K;
     fs << "D" << D;
     fs << "OptimalMatrix" << opt_matrix;
     fs << "grid_cols" << grid_cols;
     fs << "grid_rows" << grid_rows;
     fs << "dot_size" << dot_size;
     fs << "resolution" << frame_resolution;
     cout << "=== Finished Calibration ===" <<endl;
}

void FixImage(Mat& frame)
{
     Mat K, D;
     
     FileStorage fs(CALIBRATION_FILE_NAME, FileStorage::READ);
     if(!fs.isOpened())
     {
	  cout << "*** Could not open file..." <<endl;
	  return;
     }
     
     Mat opt_matrix;

     fs["K"] >> K;
     fs["D"] >> D;
     fs["OptimalMatrix"] >> opt_matrix;
     
     Mat undist_matrix;
     fisheye::undistortImage(frame, undist_matrix, K, D, opt_matrix);

     frame = undist_matrix;
}

void StereoCalibrate(    vector<Mat>& left_frames, 
                         vector<Mat>& right_frames, 
                         vector<vector<Point3f>>& ops, 
                         vector<vector<Point2f>>& lips, 
                         vector<vector<Point2f>>& rips)
{
     // We need to have frames from both cameras.
     if(left_frames.empty() && right_frames.empty()) return;

     Size frame_resolution;

     cout << "=== Starting Stereo Calibration ===" << endl;
     cout << "  > Searching frames for targets..." <<endl;
     for(int i =0; i < left_frames.size() && i < right_frames.size(); i++)
     {
          Mat left_frame = left_frames[i], right_frame = right_frames[i];

          Mat left_grey, right_grey;
          cvtColor(left_frame, left_grey, COLOR_BGR2GRAY);
          cvtColor(right_frame, right_grey, COLOR_BGR2GRAY);

          vector<Point2f> left_buffer, right_buffer;

          bool found_left  = findCirclesGrid(left_grey, Size(grid_cols, grid_rows), left_buffer, CALIB_CB_SYMMETRIC_GRID);
          bool found_right = findCirclesGrid(right_grey, Size(grid_cols, grid_rows), right_buffer, CALIB_CB_SYMMETRIC_GRID);
          
          drawChessboardCorners(left_frame, Size(grid_cols, grid_rows), left_buffer, true);
          drawChessboardCorners(right_frame, Size(grid_cols, grid_rows), right_buffer, true);

          if((!left_buffer.empty() || !right_buffer.empty()) && found_left == found_right)
          {
               vector<Point3f> objs;
               for(int i = 0; i < grid_rows; i++)
                    for(int j = 0; j < grid_cols; j++)
                    objs.push_back(Point3f((float)j * (dot_size + grid_gap), (float)i * (dot_size + grid_gap), 0));

               
               lips.push_back(left_buffer);
               rips.push_back(right_buffer);
               ops.push_back(objs);
          }

         frame_resolution = left_frame.size();
     }

     if(  ops.size()  != lips.size() || ops.size()   != rips.size() ||
          lips.size() != rips.size() || lips.empty() || rips.empty())
          return;
     
     cout << "  > Calibrating stereo..." << endl;

     // Set the flags for calibration.
     int flag = 0;
     flag |= fisheye::CALIB_RECOMPUTE_EXTRINSIC;
     flag |= fisheye::CALIB_CHECK_COND;
     flag |= fisheye::CALIB_FIX_SKEW;
     flag |= fisheye::CALIB_FIX_K1;

     Mat K1, K2, D1, D2;
     vector< Mat > R, T;
     
     // TODO: This won't need to stay if we can figure out the stereo image problem.
     FileStorage fs(CALIBRATION_FILE_NAME, FileStorage::READ);
     if(!fs.isOpened()) 
     {
          cout << "*** Could not open file..." <<endl;
          return;
     }
     
     // Get calibration data from previous file.
     // TODO: This is more of a test than anything. This should either be removed completely or should be given more reasonable values.
     fs["K"] >> K1;
     fs["D"] >> D1;
     fs["K"] >> K2;
     fs["D"] >> D2;

     double result = fisheye::stereoCalibrate(ops, lips, rips, K1, D1, K2, D2, frame_resolution, R, T, flag);
     cout << "  > Calibration Result: " << result << endl;
     
     cout << "  > Rectifying stereo..." << endl;
     Mat R1, R2, P1, P2, Q;
     fisheye::stereoRectify(K1, D1, K2, D2, frame_resolution, R, T, R1, R2, P1, P2, Q, flag);

     fs = FileStorage(CALIBRATION_FILE_NAME, FileStorage::WRITE);
     fs << "K1" << K1;
     fs << "D1" << D1;
     fs << "K2" << K2;
     fs << "D2" << D2;
     fs << "grid_cols" << grid_cols;
     fs << "grid_rows" << grid_rows;
     fs << "dot_size" << dot_size;
     fs << "resolution" << frame_resolution;
     cout << "=== Finished Stereo Calibration ===" <<endl;
}
