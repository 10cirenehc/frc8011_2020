/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/auto_commands.h"
#include <cmath>
#include "frc/smartdashboard/SmartDashboard.h"

//input gyro and drive objects into this class
void auto_commands::initAutoCommands(gyroPig* gyro_pig, frc_8011::swerveDriveMode driveMode, int mode){
    gyro = gyro_pig; 
    drive = driveMode;
    position = new double[3];
    rotationsTravelled =0;

    switch(mode){
        case 1:
            position[0] = 0; //X position
            position[1] = 0; //Y position
            position[2] = 0; // direction
            break;
        default:
            break;
    }

    printPosition();
}

//heading in degrees, absolute 
void auto_commands::move(double maxSpeed, double targetDist, double heading){

    //adjust for previous direction
    heading -= getPreviousDirection();

    double headingRadians = heading*(3.1416/180);
    double xTargetDist = targetDist * cos(headingRadians);
    double yTargetDist = targetDist * sin(headingRadians);
    double currentX, currentY = 0;
    double errorX, errorY= 0;
    double sum_errorX, sum_errorY = 0;
    double xCorrection, yCorrection, angleCorrection = 0;

    //PID loop below
    while (((leftFront_encoder.GetPosition()*0.33 + rightRear_encoder.GetPosition()*0.33)/2) < targetDist-drive_error_margin || 
            ((leftFront_encoder.GetPosition()*0.33 + rightRear_encoder.GetPosition()*0.33)/2)> targetDist+drive_error_margin)
    {
        currentX = ((getCurrentEncoderDist(leftFront_encoder)+getCurrentEncoderDist(rightRear_encoder))/2)*cos(headingRadians);
        currentY = ((getCurrentEncoderDist(leftFront_encoder)+getCurrentEncoderDist(rightRear_encoder))/2)*sin(headingRadians);

        errorX = xTargetDist - currentX;
        errorY = yTargetDist - currentY;

        angleCorrection = gyro->getGyroSpin(getPreviousDirection());

        xCorrection = kpDist*errorX + kiDist*sum_errorX;
        yCorrection = kpDist*errorY + kiDist*sum_errorY;

        sum_errorX += errorX;
        sum_errorY += errorY;

        //cap input values
        if (abs(xCorrection) > maxSpeed){
            xCorrection =maxSpeed;
        }
        if (abs(yCorrection) >maxSpeed){
            yCorrection =maxSpeed;
        }

        drive.execute(xCorrection, yCorrection, angleCorrection);

        updatePosition(currentX, currentY);
    }
        updateRotations();
        printPosition();
}

void auto_commands::spinTo(double direction){
    double correction = gyro->getGyroSpin(direction);
    drive.execute(0,0,correction);
    updateDirection(direction);
    updateRotations();
}

void auto_commands::updatePosition(double addX, double addY){
    position[0] += addX;
    position[1] += addY;
}

void auto_commands::updateDirection(double newDirection){
    position[3] = newDirection;
}

void auto_commands::updateRotations(){
    rotationsTravelled = (leftFront_encoder.GetPosition()+rightRear_encoder.GetPosition())/2;
}

double auto_commands::getCurrentEncoderDist(rev::CANEncoder encoder){
    return (encoder.GetPosition()-rotationsTravelled)*0.33;
}

double auto_commands::getPreviousDirection(){
    return position[3];
}

void auto_commands::printPosition(){
    frc::SmartDashboard::PutNumber("X", position[0]);
    frc::SmartDashboard::PutNumber("Y", position[1]);
    frc::SmartDashboard::PutNumber("Direction", position[3]);
}









