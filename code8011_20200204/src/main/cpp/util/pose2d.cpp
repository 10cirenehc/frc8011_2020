/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "util/pose2d.h"
#include "stdlib.h"
#include "math.h"
#include "stdio.h"
#include "..\..\include\util\alglib\src\optimization.h"

void pose2d::init(int automode, double* pose_YXR, robotStatus *status) {

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
const double ks = 10;
const double ka = 180/pi;
double Lr = L/r; double Wr = W/r;

void update (robotStatus *status,gyroPig* gyro_Pig,double *YXR)
 {
    double sConversion = (2*pi/60);
    double aConversion; 
    pose2d::sRF = status->rFDriveSpeed;
    pose2d::sLF = status->lFDriveSpeed;
    pose2d::sRR = status->rRDriveSpeed;
    pose2d::sLR = status->lRDriveSpeed;

    pose2d::aRF = status->rFSteerEnc; 
    pose2d::aLF = status->lFSteerEnc;
    pose2d::aRR = status->rRSteerEnc;
    pose2d::aLR = status->lFSteerEnc;

    double omega = status->gyro_rate;

    
    double RCW = omega *(r/2);

    alglib::real_1d_array x = "[0,0,0]";
    alglib::real_1d_array s = "[1,1,1]";
    double epsg = 0;
    double epsf = 0;
    double epsx = 0.0000000001;
    alglib::ae_int_t maxits = 0;
    alglib::minlbfgsstate state;
    alglib::minlbfgscreate(1, x, state);
    alglib::minlbfgssetcond(state, epsg, epsf, epsx, maxits);
    alglib::minlbfgsreport rep;
    alglib::minlbfgsoptimize(state, function1_grad);
    minlbfgsresults(state, x, rep);
    

 }
void update (limeLightManage limeman)
  {

  }

void update (double Y, double X, double R)
  {

  }

void function1_grad(const alglib::real_1d_array &x, double &func, alglib::real_1d_array &grad, void *ptr){
    // a = x[1]-x[2]*Lr    b = x[1]+x[2]*Lr    c = x[0]-x[2]*Wr    d = x[0]+x[2]*Wr 
    func = alglib::sqr(ks*(pose2d::sRF-sqrt(alglib::sqr(x[1]+x[2]*Lr)+alglib::sqr(x[0]-x[2]*Wr))))
            +alglib::sqr(ks*(pose2d::sLF-sqrt(alglib::sqr(x[1]+x[2]*Lr)+alglib::sqr(x[0]+x[2]*Wr))))
            +alglib::sqr(ks*(pose2d::sLR-sqrt(alglib::sqr(x[1]-x[2]*Lr)+alglib::sqr(x[0]+x[2]*Wr))))
            +alglib::sqr(ks*(pose2d::sRR-sqrt(alglib::sqr(x[1]-x[2]*Lr)+alglib::sqr(x[0]-x[2]*Wr))))
            +alglib::sqr(ka*(pose2d::aRF-atan2(x[1]+x[2]*Lr,x[0]-x[2]*Wr)))
            +alglib::sqr(ka*(pose2d::aLF-atan2(x[1]+x[2]*Lr,x[0]+x[2]*Wr)))
            +alglib::sqr(ka*(pose2d::aLR-atan2(x[1]-x[2]*Lr,x[0]+x[2]*Wr)))
            +alglib::sqr(ka*(pose2d::aRR-atan2(x[1]-x[2]*Lr,x[0]-x[2]*Wr)));
    
    grad[0] = (2*(ks*(pose2d::sRF-sqrt(alglib::sqr(x[1]+x[2]*Lr)+alglib::sqr(x[0]-x[2]*Wr))))*(-1/2)*
                pow(alglib::sqr(x[1]+x[2]*Lr)+alglib::sqr(x[0]-x[2]*Wr),(-1/2))*(2)*(x[0]-x[2]*Wr))+
                (2*(ks*(pose2d::sLF-sqrt(alglib::sqr(x[1]+x[2]*Lr)+alglib::sqr(x[0]+x[2]*Wr))))*(-1/2)*
                pow(alglib::sqr(x[1]+x[2]*Lr)+alglib::sqr(x[0]+x[2]*Wr),(-1/2))*(2)*(x[0]+x[2]*Wr))+
                (2*(ks*(pose2d::sLR-sqrt(alglib::sqr(x[1]-x[2]*Lr)+alglib::sqr(x[0]-x[2]*Wr))))*(-1/2)*
                pow(alglib::sqr(x[1]-x[2]*Lr)+alglib::sqr(x[0]+x[2]*Wr),(-1/2))*(2)*(x[0]+x[2]*Wr))+
                (2*(ks*(pose2d::sRR-sqrt(alglib::sqr(x[1]-x[2]*Lr)+alglib::sqr(x[0]-x[2]*Wr))))*(-1/2)*
                pow(alglib::sqr(x[1]-x[2]*Lr)+alglib::sqr(x[0]-x[2]*Wr),(-1/2))*(2)*(x[0]-x[2]*Wr))+
                2*(ka*(pose2d::aRF-atan2(x[1]+x[2]*Lr,x[0]-x[2]*Wr)))*(-1/(1+alglib::sqr((x[1]+x[2]*Lr/x[0]-x[2]*Wr))))*
                (-(x[1]+x[2]*Lr)/alglib::sqr(x[0]-x[2]*Wr))+
                2*(ka*(pose2d::aLF-atan2(x[1]+x[2]*Lr,x[0]+x[2]*Wr)))*(-1/(1+alglib::sqr((x[1]+x[2]*Lr/x[0]+x[2]*Wr))))*
                (-(x[1]+x[2]*Lr)/alglib::sqr(x[0]+x[2]*Wr))+
                2*(ka*(pose2d::aLR-atan2(x[1]-x[2]*Lr,x[0]+x[2]*Wr)))*(-1/(1+alglib::sqr((x[1]-x[2]*Lr/x[0]+x[2]*Wr))))*
                (-(x[1]-x[2]*Lr)/alglib::sqr(x[0]+x[2]*Wr))+
                2*(ka*(pose2d::aRR-atan2(x[1]-x[2]*Lr,x[0]-x[2]*Wr)))*(-1/(1+alglib::sqr((x[1]-x[2]*Lr/x[0]-x[2]*Wr))))*
                (-(x[1]-x[2]*Lr)/alglib::sqr(x[0]-x[2]*Wr));

    grad[1] = (2*(ks*(pose2d::sRF-sqrt(alglib::sqr(x[1]+x[2]*Lr)+alglib::sqr(x[0]-x[2]*Wr))))*(-1/2)*
                pow(alglib::sqr(x[1]+x[2]*Lr)+alglib::sqr(x[0]-x[2]*Wr),(-1/2))*(2)*(x[1]+x[2]*Lr))+
                (2*(ks*(pose2d::sLF-sqrt(alglib::sqr(x[1]+x[2]*Lr)+alglib::sqr(x[0]+x[2]*Wr))))*(-1/2)*
                pow(alglib::sqr(x[1]+x[2]*Lr)+alglib::sqr(x[0]+x[2]*Wr),(-1/2))*(2)*(x[1]+x[2]*Lr))+
                (2*(ks*(pose2d::sLR-sqrt(alglib::sqr(x[1]-x[2]*Lr)+alglib::sqr(x[0]-x[2]*Wr))))*(-1/2)*
                pow(alglib::sqr(x[1]-x[2]*Lr)+alglib::sqr(x[0]+x[2]*Wr),(-1/2))*(2)*(x[1]-x[2]*Lr))+
                (2*(ks*(pose2d::sRR-sqrt(alglib::sqr(x[1]-x[2]*Lr)+alglib::sqr(x[0]-x[2]*Wr))))*(-1/2)*
                pow(alglib::sqr(x[1]-x[2]*Lr)+alglib::sqr(x[0]-x[2]*Wr),(-1/2))*(2)*(x[1]-x[2]*Lr))+
                2*(ka*(pose2d::aRF-atan2(x[1]+x[2]*Lr,x[0]-x[2]*Wr)))*(-1/(1+alglib::sqr((x[1]+x[2]*Lr/x[0]-x[2]*Wr))))*
                (x[1]+x[2]*Lr/x[0]-x[2]*Wr)+
                2*(ka*(pose2d::aLF-atan2(x[1]+x[2]*Lr,x[0]+x[2]*Wr)))*(-1/(1+alglib::sqr((x[1]+x[2]*Lr/x[0]+x[2]*Wr))))*
                (x[1]+x[2]*Lr/x[0]+x[2]*Wr)+
                2*(ka*(pose2d::aLR-atan2(x[1]-x[2]*Lr,x[0]+x[2]*Wr)))*(-1/(1+alglib::sqr((x[1]-x[2]*Lr/x[0]+x[2]*Wr))))*
                (x[1]-x[2]*Lr/x[0]+x[2]*Wr)+
                2*(ka*(pose2d::aRR-atan2(x[1]-x[2]*Lr,x[0]-x[2]*Wr)))*(-1/(1+alglib::sqr((x[1]-x[2]*Lr/x[0]-x[2]*Wr))))*
                (x[1]-x[2]*Lr/x[0]-x[2]*Wr);

    grad[2] = (2*(ks*(pose2d::sRF-sqrt(alglib::sqr(x[1]+x[2]*Lr)+alglib::sqr(x[0]-x[2]*Wr))))*(-1/2)*
                pow(alglib::sqr(x[1]+x[2]*Lr)+alglib::sqr(x[0]-x[2]*Wr),(-1/2))*((2)*(x[1]+x[2]*Lr))-2*(x[0]-x[2]*Wr))+
                (2*(ks*(pose2d::sLF-sqrt(alglib::sqr(x[1]+x[2]*Lr)+alglib::sqr(x[0]+x[2]*Wr))))*(-1/2)*
                pow(alglib::sqr(x[1]+x[2]*Lr)+alglib::sqr(x[0]+x[2]*Wr),(-1/2))*((2)*(x[1]+x[2]*Lr))+2*(x[0]+x[2]*Wr))+
                (2*(ks*(pose2d::sLR-sqrt(alglib::sqr(x[1]-x[2]*Lr)+alglib::sqr(x[0]-x[2]*Wr))))*(-1/2)*
                pow(alglib::sqr(x[1]-x[2]*Lr)+alglib::sqr(x[0]+x[2]*Wr),(-1/2))*((2)*(x[1]-x[2]*Lr))+2*(x[0]+x[2]*Wr))+
                (2*(ks*(pose2d::sRR-sqrt(alglib::sqr(x[1]-x[2]*Lr)+alglib::sqr(x[0]-x[2]*Wr))))*(-1/2)*
                pow(alglib::sqr(x[1]-x[2]*Lr)+alglib::sqr(x[0]-x[2]*Wr),(-1/2))*((2)*(x[1]-x[2]*Lr))-2*(x[0]-x[2]*Wr))+
                2*(ka*(pose2d::aRF-atan2(x[1]+x[2]*Lr,x[0]-x[2]*Wr)))*(-1/(1+alglib::sqr((x[1]+x[2]*Lr/x[0]-x[2]*Wr))))*
                (x[0]-x[2]*Wr-(-1)*(x[1]+x[2]*Lr)/alglib::sqr(x[0]-x[2]*Wr))+
                2*(ka*(pose2d::aLF-atan2(x[1]+x[2]*Lr,x[0]+x[2]*Wr)))*(-1/(1+alglib::sqr((x[1]+x[2]*Lr/x[0]+x[2]*Wr))))*
                (x[0]+x[2]*Wr-(x[1]+x[2]*Lr)/alglib::sqr(x[0]-x[2]*Wr))+
                2*(ka*(pose2d::aLR-atan2(x[1]-x[2]*Lr,x[0]+x[2]*Wr)))*(-1/(1+alglib::sqr((x[1]-x[2]*Lr/x[0]+x[2]*Wr))))*
                (x[0]-x[2]*Wr-(x[1]-x[2]*Lr)/alglib::sqr(x[0]-x[2]*Wr))+
                2*(ka*(pose2d::aRR-atan2(x[1]-x[2]*Lr,x[0]-x[2]*Wr)))*(-1/(1+alglib::sqr((x[1]-x[2]*Lr/x[0]-x[2]*Wr))))*
                (x[0]-x[2]*Wr-(-1)*(x[1]-x[2]*Lr)/alglib::sqr(x[0]-x[2]*Wr));
}
