/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "rev/CANEncoder.h"
#include "rev/CANSparkMax.h"
#include "gyro/pigeonIMU.h"
#include "swerveDrive/swerveDriveMode.h"


class auto_commands {
 public:
  
  double kpDist = 0.26, kiDist = 0.000001, kdDist = 0.0, kIz = 0, kFF = 0;
  double drive_error_margin = 0.08;
  double sumRotations = 0;

  //Heading是机器人行走的方向，Direction是车头面向的方向。

  void initAutoCommands(gyroPig* gyro_pig, frc_8011::swerveDriveMode driveMode, int mode);
  void move(double maxSpeed, double targetDist, double heading);
  void spinTo(double heading);
  void updatePosition(double addX, double addY);
  void updateDirection(double newDriection);
  double getPreviousDirection();
  double getCurrentEncoderDist(rev::CANEncoder encoder);
  void updateRotations();
  void printPosition();
  
  double* position;
  gyroPig* gyro; 
  frc_8011::swerveDriveMode drive;
  double rotationsTravelled;

  rev::CANSparkMax* _leftFront = new rev::CANSparkMax(3,rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* _leftRear = new rev::CANSparkMax(4,rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* _rightFront = new rev::CANSparkMax(1,rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* _rightRear = new rev::CANSparkMax(2,rev::CANSparkMax::MotorType::kBrushless);

  rev::CANEncoder leftFront_encoder = _leftFront->GetEncoder();
  rev::CANEncoder rightFront_encoder = _rightFront->GetEncoder();
  rev::CANEncoder leftRear_encoder = _leftRear->GetEncoder();
  rev::CANEncoder rightRear_encoder = _leftRear->GetEncoder();  
};
