#include "limeLight/limeLightManage.h"
#include <cmath>
#include<frc/smartdashboard/SmartDashboard.h>

double spin_error_sum=0.0;
double dis_error_sum=0.0;
double hor_error_sum=0.0;

double limeLightManage::aim_distance(double targetDis){
    double driving_adjust=0.0;
    double driving_error=0.0;
    if(lime.hasTarget()){
        driving_error=lime.getDistance()-targetDis;
        driving_adjust=kpDistance*driving_error+kiDistance*dis_error_sum;
        if(driving_error<0.08&&driving_error>-0.08){
            driving_adjust=0.0;
            dis_error_sum=0.0;
        }
    }
    
    frc::SmartDashboard::PutNumber("driving_adjust",driving_adjust);
    frc::SmartDashboard::PutNumber("driving_error",driving_error);
    return driving_adjust;
}

double limeLightManage::aim_hor_angle(){
    double horizontal_adjust=0.0;
    double horAngle_error=0.0;
    if(lime.hasTarget()){
      horAngle_error=lime.getHorAngle();
      horizontal_adjust=kpHor*horAngle_error+kiHor*hor_error_sum;
      hor_error_sum=hor_error_sum+horAngle_error;

      if(horAngle_error<0.5&&horAngle_error>-0.5){
        horizontal_adjust=0.0;
        hor_error_sum=0.0;
    }
    }
    frc::SmartDashboard::PutNumber("horizontal_adjust",horizontal_adjust);
    frc::SmartDashboard::PutNumber("horAngle_error",horAngle_error);
    return horizontal_adjust;
}

double limeLightManage::aim_spin_angle(){
    double spin_adjust=0.0;
    double horAngle_error=0.0;
    if(lime.hasTarget()){
    horAngle_error=lime.getHorAngle();
    spin_error_sum=spin_error_sum+horAngle_error;

    spin_adjust=kpSpin*horAngle_error+kiSpin*spin_error_sum;
    if(horAngle_error<0.5&&horAngle_error>-0.5){
        spin_adjust=0.0;
        spin_error_sum=0.0;
    }
    }
    frc::SmartDashboard::PutNumber("spin_adjust",spin_adjust);
    frc::SmartDashboard::PutNumber("horAngle_error",horAngle_error);
    return spin_adjust;
}

void limeLightManage::setLimeLed(limeLightLedMode mode){
    switch (mode)
    {
    case mGetBall:
        lime.setLed(1);
        break;
    case mShootBall:
        lime.setLed(3);
        break;
    case mDisable:
        lime.setLed(1);
        break;
    default:
        break;
    }
}