#include <auto/autoMode.h>


void autoAction(double error_gyro,int *error_drive,autoMODE mode,double *YXR){

    switch (mode)
    {
    case firstAuto:
        //直行走向摆球位置，直行4米，直至抓到两个球为止
        if(error_drive[0]<80&&error_drive[1]<80&&error_drive[2]<80&&error_drive[3]<80&&error_gyro<180){
            YXR[0]=1.0;
            YXR[1]=0.0;
            YXR[2]=0.0;
            if(error_gyro!=0.0){
               YXR[2]=error_gyro*0.02;
          }
        }
        else if((error_drive[0]>=80||error_drive[1]>=80||error_drive[2]>=80||error_drive[3]>=80)&&error_gyro<180){
            if(error_gyro<180){//掉头180度
            YXR[0]=0.0;
            YXR[1]=0.0;
            YXR[2]=1.0;
            }
        }
        else {
            YXR[0]=1.0;
            YXR[1]=0.0;
            YXR[2]=0.0;
            if((error_gyro-180)!=0.0){
               YXR[2]=(error_gyro-180)*0.02;
          }
        }

        //往回走4米
        //对准射球孔位
        //开始射球
        //掉头走3米
        //左转90度，启动前置摄像头寻球，收球
        //收购五个球后，启动射球摄像头，再左转寻找射球孔位
        //射球

        break;
    case secondAuto:
        break;
    default:
        break;
    }
}