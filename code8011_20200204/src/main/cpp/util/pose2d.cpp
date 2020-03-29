/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "util/pose2d.h"

void pose2d::init(int automode, double* pose_YXR) {
    // 输入不同自动模式的初始坐标YXR
    switch (automode){
        case 1: 
        default:
            break;
    }
}

double L=27.56;//为70厘米
double W=27.56;
double r=38.98;
double pi = 3.1416;

void update (robotStatus *status,gyroPig* gyro_Pig,double *YXR)
 {
    double sConversion = (2*pi/60);
    double aConversion; 
    double sRF = status->rFDriveSpeed;
    double sLF = status->lFDriveSpeed;
    double sRR = status->rRDriveSpeed;
    double sLR = status->lRDriveSpeed;

    double aRF = status->rFSteerEnc; 
    double aLF = status->lFSteerEnc;
    double aRR = status->rRSteerEnc;
    double aLR = status->lFSteerEnc;

    double omega = status->gyro_rate;

    double Lr = L/r; double Wr = W/r;
    double RCW = omega *(r/2);

 }
void update (limeLightManage limeman)
  {

  }

void update (double Y, double X, double R)
  {

  }
