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

boolean limelight::hasTarget(){
    int num =  table->GetNumber("tv",0.0);
    if (num==1){
        return true;
    }
    else{
        return false;
    }
}



