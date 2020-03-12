#include <auto/autoMode.h>
#include "util/commonUtil.h"
#include <cmath>
#include<frc/smartdashboard/SmartDashboard.h>

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
        //直行走向摆球位置，直行4米，直至抓到两个球为止
        double drive_error=(status->rFDriveEnc+status->lRDriveEnc)/2;
        if(actionStep==0){  //第一步
            YXR[0]=goForward(40.0-drive_error);
            YXR[1]=0.0;
            YXR[2]=gyro_Pig->getGyroSpin(0);
            if(YXR[0]==0.0){
                actionStep=1;
            }
        }
        if(actionStep==1){    //第二步
            YXR[0]=0.0;
            YXR[1]=0.0;
            YXR[2]=gyro_Pig->getGyroSpin(180);
            if(YXR[2]==0.0){
                actionStep=2;
                drive_status=(status->rFDriveEnc+status->lRDriveEnc)/2;
            }
        }
        if(actionStep==2){    //第三步
            YXR[0]=goForward((40.0+drive_status)-drive_error);
            YXR[1]=0.0;
            YXR[2]=gyro_Pig->getGyroSpin(180);
            if(YXR[0]==0.0){
                actionStep=3;
                drive_status=(status->rFDriveEnc+status->lRDriveEnc)/2;
            }
        }
        if(actionStep==3){    //第四步
            YXR[0]=0.0;
            YXR[1]=goForward(60+drive_status-drive_error);
            YXR[2]=gyro_Pig->getGyroSpin(180);
            if(YXR[1]==0.0){
                actionStep=4;
            }
        }
        if(actionStep==4){     //第五步
            limeMan.setLimeLed(limeLightLedMode::mShootBall);

            YXR[0]=0.0;
            YXR[1]=0.0;
            YXR[2]=limeMan.aim_spin_angle();
            if(YXR[2]==0.0){
                actionStep=5;
            }
        }
        if(actionStep==5){
             limeMan.setLimeLed(limeLightLedMode::mShootBall);
            YXR[0]=limeMan.aim_distance(2.0);
            YXR[1]=0.0;
            YXR[2]=limeMan.aim_spin_angle();
            // if(YXR[0]==0.0){
            //     actionStep=6;
            // }
        }


        //往回走4米
        //对准射球孔位
        //开始射球
        //掉头走3米
        //左转90度，启动前置摄像头寻球，收球
        //收购五个球后，启动射球摄像头，再左转寻找射球孔位
        //射球
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