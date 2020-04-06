/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "gyro/pigeonIMU.h"
#include "swerveDrive/swerveDriveMode.h"
#include "util/robotStatus.h"
#include "limeLight/limeLightManage.h"
#include "..\..\include\util\alglib\src\optimization.h"

class pose2d {
 public:
  void init(int automode, double* pose_YXR, robotStatus *status);

  void update (robotStatus *status,gyroPig* gyro_Pig,double *YXR);
  void update (limeLightManage limeman);
  void update (double Y, double X, double R);

  void function1_grad(const alglib::real_1d_array &x, double &func, alglib::real_1d_array &grad, void *ptr);

  static double sRF;
  static double sLF;
  static double sRR;
  static double sLR;

  static double aRF; 
  static double aLF;
  static double aRR;
  static double aLR;

};
