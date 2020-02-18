#include "gyro/pigeonIMU.h"
#include <frc/smartdashboard/SmartDashboard.h>
double kPgain = 0.03; 
double kDgain = 0.00004; 

gyroPig::gyroPig(int canID){
    _pidgey=new PigeonIMU(canID);
}

void gyroPig::gyroInit(){
    //_pidgey->ConfigFactoryDefault();
    const int kTimeoutMs = 10;
	_pidgey->SetFusedHeading(0.0, kTimeoutMs);
  
}

bool gyroPig::gyroReady(){
    return (_pidgey->GetState() == PigeonIMU::Ready) ? true : false;
}

double gyroPig::getCurrentAngle(){
    PigeonIMU::FusionStatus *stat=new PigeonIMU::FusionStatus();
    _pidgey->GetFusedHeading(*stat);
    return stat->heading;
}

double gyroPig::getGyroSpin(double targetAngle){
    double spin_adjust=0.0;

    double currentAngle=getCurrentAngle();
    double angle_error=targetAngle-currentAngle;//角度误差
    frc::SmartDashboard::PutNumber("currentAngle",currentAngle);
    frc::SmartDashboard::PutNumber("angle_error",angle_error);
    PigeonIMU::GeneralStatus genStatus;
	double xyz_dps[3];
	_pidgey->GetGeneralStatus(genStatus);
	_pidgey->GetRawGyro(xyz_dps);
    double currentAngularRate = xyz_dps[2];

    spin_adjust=angle_error*kPgain-currentAngularRate*kDgain;
    if(spin_adjust>0.3){
        spin_adjust=0.3;
    }
    else if(spin_adjust<-0.3){
        spin_adjust=-0.3;
    }

    return spin_adjust;
}