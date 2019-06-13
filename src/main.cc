#include <iostream>
#include <Calibration.h>

int main(int argc, char** argv)
{
     Calibration::Input input;

     Calibration* calib = new Calibration(input, CalibrationType::STEREO, "StereoCalibration.yaml");
     calib->ReadImages("Archive/", "Archive/");

     calib->RunCalibration();
     calib->ShowRectifiedImage();
     
     delete calib;
     return 0;
}