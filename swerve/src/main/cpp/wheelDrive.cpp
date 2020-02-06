/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "wheelDrive.h"

wheelDrive::wheelDrive (int drive, int turn){
    wheelDrive::driveMotor = new rev::CANSparkMax(drive, rev::CANSparkMax::MotorType::kBrushless);
    wheelDrive::turnMotor = new TalonSRX(turn);
    turnMotor->ConfigFactoryDefault;

    //configure sensors (turning encoder)
    turnMotor ->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder,0,10);
    turnMotor->SetSensorPhase(true);

    //initialize pid control? 
}

void wheelDrive::drive(double speed, double angle){
    
}
