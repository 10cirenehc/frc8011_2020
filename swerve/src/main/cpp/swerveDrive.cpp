/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "swerveDrive.h"
#include <cmath>

using namespace std;

    void swerveDrive::drive(double x1, double y1, double x2){
         double r = sqrt(length*length + width*width);
         y1*=-1;
         
    double a = x1 - x2 * (length / r);
    double b = x1 + x2 * (length / r);
    double c = y1 - x2 * (width/ r);
    double d = y1 + x2 * (width/ r);


    double backRightSpeed = sqrt ((a * a) + (d * d));
    double backLeftSpeed = sqrt ((a * a) + (c * c));
    double frontRightSpeed = sqrt ((b * b) + (d * d));
    double frontLeftSpeed = sqrt ((b * b) + (c * c));

    double backRightAngle = atan2 (a, d) *180/ M_PI;
    double backLeftAngle = atan2 (a, c) *180/ M_PI;
    double frontRightAngle = atan2 (b, d) *180/ M_PI;
    double frontLeftAngle = atan2 (b, c) *180/ M_PI;
}
