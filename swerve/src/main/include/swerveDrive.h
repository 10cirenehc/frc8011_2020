/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

class swerveDrive {
 public:
 const double length = 0.57;
 const double width = 0.37;
  void drive(double x1, double y1, double x2);
  swerveDrive();
};
