#include <iostream>
#include <Calibration.h>

int main(int argc, char** argv)
{
     Calibration::Input input;

     Calibration* calib = new Calibration(input, CalibrationType::STEREO, "StereoCalibration.yaml");
     calib->ReadImages("Hero7-1/", "Hero7-2/");

     //calib->RunCalibration();
     calib->ReadCalibration();
     calib->GetUndistortedImage();
     
     delete calib;
     return 0;
}