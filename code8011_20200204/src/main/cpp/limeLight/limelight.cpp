
#include "limeLight/limelight.h"
#include <cmath>
#include<frc/Timer.h>

using namespace frc_8011;
using namespace std;

limelight::limelight() {
    table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
}

double limelight::getDistance(){
    return (TARGET_HEIGHT-MOUNT_HEIGHT)/tan((MOUNT_ANGLE+limelight::getVertAngle())*PI/180);
}

double limelight::getVertAngle(){
    return table->GetNumber("ty",0.0);
    frc::Wait(0.05);
}

double limelight::getHorAngle(){
    return table->GetNumber("tx",0.0);
    frc::Wait(0.05);
}

double limelight::getArea(){
    return table->GetNumber("ta",0.0);
    frc::Wait(0.05);
}

double limelight::getSkew(){
    return table-> GetNumber("ts",0.0);
    frc::Wait(0.05);
}

void limelight::printValues(){
    frc::SmartDashboard::PutBoolean("Target found", hasTarget());
    frc::SmartDashboard::PutNumber("Vert Ang", getVertAngle());
    frc::SmartDashboard::PutNumber("Hori Angle", getHorAngle());
    frc::SmartDashboard::PutNumber("Area", getArea());
    frc::SmartDashboard::PutNumber("Skew", getSkew());
    frc::SmartDashboard::PutNumber("Distance", getDistance());
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

void limelight::setLed(int modeNum){
    table->PutNumber("ledMode",modeNum);
}
