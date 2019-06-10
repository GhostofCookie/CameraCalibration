#include <iostream>
#include <Calibration.h>

int main(int argc, char** argv)
{
     Calibration* calib = new Calibration(CalibrationType::STEREO);

     calib->RunCalibration();
     calib->ShowRectifiedImage();
     
     delete calib;
     return 0;
}