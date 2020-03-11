#pragma once

#include "util/robotStatus.h"
#include "swerveDrive/swerveDriveMode.h"
#include "limeLight/limeLightManage.h"
#include "gyro/pigeonIMU.h"

void updateStatus(frc_8011::swerveDriveMode driveMode,gyroPig* gyro_Pig,limeLightManage lime,robotStatus *status);