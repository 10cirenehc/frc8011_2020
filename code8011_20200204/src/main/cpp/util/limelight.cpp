/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "util/limelight.h"
#include <cmath>

limelight::limelight() {}

double limelight::getDistance(){
    return (TARGET_HEIGHT-MOUNT_HEIGHT)/tan((MOUNT_ANGLE+limelight::getVertAngle())*pi/180);
}
double limelight::getVertAngle(){
    return table->GetNumber("ty",0.0);
}
double limelight::getHorAngle(){
    return table->GetNumber("tx",0.0);
}
double limelight::getArea(){
    return table->GetNumber("ta",0.0);
}
double limelight::getSkew(){
    return table-> GetNumber("ts",0.0);
}
bool limelight::hasTarget(){
    int num =  table->GetNumber("tv",0.0);
    if (num==1){
        return true;
    }
    else{
        return false;
    }
}
void limelight::printValues(){
    frc::SmartDashboard::PutBoolean("Target found", hasTarget());
    frc::SmartDashboard::PutNumber("Vert Ang", getVertAngle());
    frc::SmartDashboard::PutNumber("Hori Angle", getHorAngle());
    frc::SmartDashboard::PutNumber("Area", getArea());
    frc::SmartDashboard::PutNumber("Skew", getSkew());
    frc::SmartDashboard::PutNumber("Distance", getDistance());
}

std::vector<double> limelight::aim_range (){
    std::vector<double> vect;
    double heading_error = getHorAngle();
    double distance_error = targetDist - getDistance();
    double driving_adjust = 0.0;
    double horizontal_adjust = 0.0;
    double spin_adjust =0.0;

    driving_adjust = kpDistance*distance_error;
    
    if (abs(heading_error) > 1.0){
        horizontal_adjust = kpHor*heading_error;
        spin_adjust = kpSpin*heading_error;
    }
    else if (abs(heading_error) < 1.0){
        if (heading_error<0){
            horizontal_adjust = kpHor*heading_error - min_aim_command_hor;
            spin_adjust = kpSpin*heading_error - min_aim_command_spin;
        }
        else if(heading_error>0){
            horizontal_adjust = kpHor*heading_error + min_aim_command_hor;
            spin_adjust = kpSpin*heading_error + min_aim_command_spin;
        }
    }
    vect.push_back(driving_adjust);
    vect.push_back(horizontal_adjust);
    vect.push_back(spin_adjust);

    return vect;
}


