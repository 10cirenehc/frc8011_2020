#pragma once

#include "swerveDrive/swerveDriveMode.h"
#include "limeLight/limeLightManage.h"
#include "gyro/pigeonIMU.h"
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
void autoAction(double error_gyro,int *error_drive,autoMODE mode,double *YXR);