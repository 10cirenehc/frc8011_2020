#pragma once

/**
 * 陀螺仪
 */
#include "ctre/Phoenix.h"


class gyroPig{
public:
   /**
    * 构造函数
    */
   gyroPig(int canID);
   /**
    * 初始化设置陀螺仪
    */
   void gyroInit();
   /**
    * 陀螺仪是否准备好
    */
   bool gyroReady();
   /**
    * 获取当前角度
    */
   double getCurrentAngle();
   /**
    * PID控制陀螺仪角度，返回底盘旋转的值
    */
   double getGyroSpin(double targetAngle);
private:
    PigeonIMU *_pidgey;
};