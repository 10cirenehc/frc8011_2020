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
    //driveMotor encoder
    int p[4];
    //drive.getDrivePosition(p);
    double rightFront_encoder=p[0];
    double leftFront_encoder=p[1];
    double leftRear_encoder=p[2];
    double rightRear_encoder=p[3];
    double headingRadians = heading*(3.1416/180);
    double xTargetDist = targetDist * cos(headingRadians);
    double yTargetDist = targetDist * sin(headingRadians);
    double currentX, currentY = 0;
    double errorX, errorY= 0;
    double sum_errorX, sum_errorY = 0;
    double xCorrection, yCorrection, angleCorrection = 0;

    updateRotations(leftFront_encoder,rightRear_encoder);

    //PID loop below
    while (((getCurrentEncoderDist(leftFront_encoder) + getCurrentEncoderDist(rightRear_encoder))/2) <(targetDist-drive_error_margin) || 
          ((getCurrentEncoderDist(leftFront_encoder) + getCurrentEncoderDist(rightRear_encoder))/2)>(targetDist+drive_error_margin))
    {
        
        currentX = ((getCurrentEncoderDist(leftFront_encoder)+getCurrentEncoderDist(rightRear_encoder))/2)*cos(headingRadians);
        currentY = ((getCurrentEncoderDist(leftFront_encoder)+getCurrentEncoderDist(rightRear_encoder))/2)*sin(headingRadians);
        
        errorX = xTargetDist - currentX;
        errorY = yTargetDist - currentY;
        
        angleCorrection = 0.0;//gyro->getGyroSpin(getPreviousDirection());
        
        xCorrection = kpDist*errorX ;
        yCorrection = kpDist*errorY;

        sum_errorX += errorX;
        sum_errorY += errorY;

        //cap input values
        if (abs(xCorrection) > maxSpeed){
            if(xCorrection>maxSpeed)
            xCorrection =maxSpeed;
            else{
                xCorrection=-1*maxSpeed;
            }
        }
        if (abs(yCorrection) >maxSpeed){
            if(yCorrection>maxSpeed)
            yCorrection =maxSpeed;
            else
            {
                yCorrection=-1*maxSpeed;
            }
            
        }

        drive.execute(xCorrection, yCorrection, angleCorrection);

        updatePosition(currentX, currentY);
         //drive.getDrivePosition(p);
         rightFront_encoder=p[0];
         leftFront_encoder=p[1];
         leftRear_encoder=p[2];
         rightRear_encoder=p[3];
          printPosition();
    }
        printPosition();
}

// void auto_commands::spinTo(double direction){
//     double correction = gyro->getGyroSpin(direction);
//     drive.execute(0,0,correction);
//     updateDirection(direction);
//     updateRotations();
// }

void auto_commands::updatePosition(double addX, double addY){
    position[0] += addX;
    position[1] += addY;
}

void auto_commands::updateDirection(double newDirection){
    position[2] = newDirection;
}

void auto_commands::updateRotations(double leftFront_encoder,double rightRear_encoder){
    rotationsTravelled = (leftFront_encoder+rightRear_encoder)/2;
}

double auto_commands::getCurrentEncoderDist(double  position){
    return (position-rotationsTravelled)*0.33;
}

double auto_commands::getPreviousDirection(){
    return position[2];
}

void auto_commands::printPosition(){
    frc::SmartDashboard::PutNumber(" position0", position[0]);
    frc::SmartDashboard::PutNumber(" position1", position[1]);
    frc::SmartDashboard::PutNumber("Direction", position[2]);
}

