#include <iostream>
#include <Calibration.h>

int main(int argc, char** argv)
{
     Calibration::Settings settings;
     settings.bUseCalibrated  = false;
     settings.type            = CalibrationType::STEREO;
     settings.outfile_name    = "StereoCalibration.yaml";

     Calibration* calib = new Calibration(settings);

     calib->RunCalibration();
     calib->ShowRectifiedImage();
     
     delete calib;
     return 0;
}