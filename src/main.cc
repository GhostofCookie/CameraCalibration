#include <iostream>
#include <Calibration.h>

int main(int argc, char** argv)
{
     Calibration* calib = new Calibration();

     calib->RunCalibration();

     delete calib;
     return 0;
}