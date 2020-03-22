#include <auto/autoMode.h>
#include "util/commonUtil.h"
#include <cmath>
#include<frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <vector>

double kp_drive=0.2;
double ki_drive=1e-4;
double kd_drive=0.0;
double errorSum_drive=0.0;
int actionStep=0;
double drive_status=0.0;

void autoAction(robotStatus * status,gyroPig* gyro_Pig,limeLightManage limeMan,autoMODE mode,double *YXR){

     frc::SmartDashboard::PutNumber("actionStep",actionStep);
    switch (mode)
    {
    case firstAuto:
    {
        double drive_error=(status->rFDriveEnc+status->lRDriveEnc)/2;
        //直行走向摆球位置，直行3米
        if(actionStep==0){  //第一步
            YXR[0]=goForward(40.0-drive_error);
            YXR[1]=0.0;
            YXR[2]=gyro_Pig->getGyroSpin(0);
            if(YXR[0]==0.0){
                actionStep=1;
            }
        }
         //旋转180度
        if(actionStep==1){    //第二步
            YXR[0]=0.0;
            YXR[1]=0.0;
            YXR[2]=gyro_Pig->getGyroSpin(180);
            if(YXR[2]==0.0){
                actionStep=2;
                drive_status=(status->rFDriveEnc+status->lRDriveEnc)/2;
            }
        }
         //往回走
        if(actionStep==2){    //第三步
            YXR[0]=goForward((40.0+drive_status)-drive_error);
            YXR[1]=0.0;
            YXR[2]=gyro_Pig->getGyroSpin(180);
            if(YXR[0]==0.0){
                actionStep=3;
                drive_status=(status->rFDriveEnc+status->lRDriveEnc)/2;
            }
        }
        //横着走
        if(actionStep==3){    //第四步
            YXR[0]=0.0;
            YXR[1]=goForward(60+drive_status-drive_error);
            YXR[2]=gyro_Pig->getGyroSpin(180);
            if(YXR[1]==0.0){
                actionStep=4;
            }
        }
        //走到指定距离，且对准
        if(actionStep==4){     //第五步
            // limeMan.setLimeLed(limeLightLedMode::mShootBall);
            // YXR[0]=limeMan.aim_distance(3.0);
            // YXR[1]=0.0;
            // YXR[2]=limeMan.aim_spin_angle();
            // if(YXR[2]==0.0){
            //     actionStep=5;
            // }
            actionStep=5;
        }
        //开始射球
        if(actionStep==5){
            actionStep=6;
            drive_status=(status->rFDriveEnc+status->lRDriveEnc)/2;
        }
        //斜着走,角度为22.5度，回2.5米，开始第二轮收球
        if(actionStep==6){
            YXR[0]=-0.41;
            YXR[1]=-1;
            YXR[2]=gyro_Pig->getGyroSpin(180);
            if((drive_status-drive_error)>=50){
                actionStep=7;
                drive_status=(status->rFDriveEnc+status->lRDriveEnc)/2;
            }
        }
        if(actionStep==7){
            YXR[0]=0;
            YXR[1]=0;
            YXR[2]=gyro_Pig->getGyroSpin(22.5);
            if(YXR[2]==0.0){
                actionStep=8;
                drive_status=(status->rFDriveEnc+status->lRDriveEnc)/2;
            }
        }
        //往前走2米
        if(actionStep==8){
            YXR[0]=goForward(40+drive_status-drive_error);
            YXR[1]=0.0;
            YXR[2]=gyro_Pig->getGyroSpin(22.5);
            if(YXR[0]==0.0){
                actionStep=9;
            }
        }
        //再转90度
        if(actionStep==9){
            YXR[0]=0;
            YXR[1]=0.0;
            YXR[2]=gyro_Pig->getGyroSpin(112.5);
            if(YXR[2]==0.0){
                actionStep=10;
                drive_status=(status->rFDriveEnc+status->lRDriveEnc)/2;
            }
        }
        //再往前走0.45米
        if(actionStep==10){
            YXR[0]=goForward(9+drive_status-drive_error);
            YXR[1]=0.0;
            YXR[2]=gyro_Pig->getGyroSpin(112.5);
            if(YXR[0]==0.0){
                actionStep=11;
            }
        }
        //再转回跟拦杠垂直
        if(actionStep==11){
            YXR[0]=0;
            YXR[1]=0.0;
            YXR[2]=gyro_Pig->getGyroSpin(22.5);
            if(YXR[2]==0.0){
                actionStep=12;
                drive_status=(status->rFDriveEnc+status->lRDriveEnc)/2;
            }
        }
        //走半径为0.7米的弧形
        if(actionStep==12){
            YXR[0]=0.8;
            YXR[1]=0.0;
            YXR[2]=-0.44;
            if(drive_error-drive_status>=40){
                actionStep=13;
            }
        }
    }
        break;
    case secondAuto:
        break;
    case thirdAuto:
        break;
    // default:
    //     break;
    }
}

double  goForward(double error){
    double speed=kp_drive*error+ki_drive*errorSum_drive;
    errorSum_drive=errorSum_drive+error;
    speed=Cap(speed,1.0);
    if(abs(error)<=2)   {
        speed=0.0;
        errorSum_drive=0;
    }
    return speed;
}

vector<double> move(double targetDist, double maxSpeed, double heading, robotStatus* status){
    {

    //adjust for previous direction
    heading -= status->gyro_angle;
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

    //PID loop bel
        
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


        updatePosition(currentX, currentY);
         //drive.getDrivePosition(p);
         rightFront_encoder=p[0];
         leftFront_encoder=p[1];
         leftRear_encoder=p[2];
         rightRear_encoder=p[3];
          printPosition();

        printPosition();
}
}