#include "util/robotUtil.h"

void updateStatus(frc_8011::swerveDriveMode driveMode,gyroPig* gyro_Pig,limeLightManage lime,robotStatus *status){
  
  driveMode.getMotorStatus(status);
  status->gyro_angle=gyro_Pig->getCurrentAngle();
  status->gyro_angle = gyro_Pig->getCurrentRate();
  lime.getLimeStatus(status);

}