#pragma once

#include "swerveDrive/swerveDriveMode.h"
#include "limeLight/limeLightManage.h"
#include "gyro/pigeonIMU.h"
#include "util/robotStatus.h"
 #include <iostream>
  #include <vector>
using namespace frc_8011;


/**
 * 自动模式方案
 * autoMode=firstAuto
 */
enum  autoMODE{
    firstAuto,
    secondAuto,
    thirdAuto,
};

/**
 * 自动模式
 */
void autoAction(robotStatus *status,gyroPig* gyro_Pig,limeLightManage limeMan,autoMODE mode,double *YXR);


double  goForward(double error);

void move(double error, double maxSpeed, double heading);




