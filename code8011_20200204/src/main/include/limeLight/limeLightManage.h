#pragma once
  
#include "limeLight/limelight.h"
#include "util/limelightLedMode.h"

  const double kpDistance = 0.52;               //调整距离的P值 
  const double kiDistance = 0.000007;              //调整距离的I值 
  const double kpHor = 0.038;                   //水平调整角度的P值
  const double kiHor = 0.00001;                   //水平调整角度的P值
  const double kpSpin= 0.024;                  //旋转调整角度的P值 
  const double kiSpin=0.00035;
              
  const double targetDist = 2.5;                //目标距离
  const double min_ain_command_dis=0.07;
  const double min_aim_command_hor = 0.05;   
  const double min_aim_command_spin = 0.06;

  class limeLightManage{
  public:
    /**
     * 找准距离位置
     */
    double aim_distance(double targetDis);
    
    /**
     * 水平找准角度
     */
    double aim_hor_angle();
    /**
     * 旋转找准角度
     */
    double aim_spin_angle();

    void setLimeLed(limeLightLedMode mode);
  private:
     frc_8011::limelight lime;
  };
