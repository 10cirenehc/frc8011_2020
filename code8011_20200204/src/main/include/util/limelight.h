/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/smartdashboard/Smartdashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#pragma once

class limelight {
 public:
  limelight();
  double getDistance();
  double getVertAngle();
  double getHorAngle();
  double getArea();
  double getSkew();
  double printValues();
  boolean hasTarget();
  
  std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");


  const double MOUNT_ANGLE = 20.0;
  const double MOUNT_HEIGHT = 0.7;
  const double TARGET_HEIGHT = 1.6;
  const double pi = 3.1416;
  

};
