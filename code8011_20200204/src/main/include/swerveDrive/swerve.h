#pragma once


/**
 *差速转向轮
 */
#include "rev/CANSparkMax.h"
#include "frc/WPILib.h"
#include "ctre/Phoenix.h"

namespace frc_8011{

   class Swerve{
   public:
     //构造函数
     Swerve(int driveId,int steerId);
     //马达初始化
     void motorInit();
     //设置前进马达的速度
     void setDriveMotorSpeed(double speed);
     //设置前进马达的位置
     void setDriveMotorPosition(double position);
     //设置转动马达的速度
     void setSteerMotorSpeed(double speed);
     //设置转动马达的位置
     void setSteerMotorPosition(double position);
     //获取前进马达的id号
     int getDriveMotorId();
     //获取旋转编码器的位置
     double getEncoder();

   private:
     rev::CANSparkMax *driveMotor;
     TalonSRX * steerMotor;

   };
}

