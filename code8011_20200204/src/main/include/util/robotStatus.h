#pragma once
#ifndef _ROBOTSTATUS_
#define _ROBOTSTATUS_
#endif
/**
 * 存在机器人各类状态
 */
struct robotStatus
{
    /**
     * 底盘四个轮子状态
     */
    double rFDriveEnc,rFSteerEnc,rFDriveSpeed,rFSteerSpeed;  //右前轮
    double lFDriveEnc,lFSteerEnc,lFDriveSpeed,lFSteerSpeed;  //左前轮
    double lRDriveEnc,lRSteerEnc,lRDriveSpeed,lRSteerSpeed;  //左后轮
    double rRDriveEnc,rRSteerEnc,rRDriveSpeed,rRSteerSpeed;  //右后轮

    /**
     * 陀螺仪角度状态
     */
    double gyro_angle;
    double gyro_rate; 

    /**
     * 摄像头状态
     */
    bool isHasTarget;                                         //是否捕捉到目标
    bool LedStatus;                                           //led的状态
    double dist_error;                                        //距离
    double hor_angle_error;                                   //十字架横向偏差
    double ver_angle_error;                                   //十字架垂直偏差
    double targetArea;                                        //面积比例
    /**
     * 收球装置状态
     */

    /**
     * 射球装置状态
     */

    /**
     * 控制转盘状态
     */

    /**
     * 抬升机构状态
     */

};

