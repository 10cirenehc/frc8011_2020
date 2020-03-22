#pragma once
#include "rev/CANEncoder.h"
#include "rev/CANSparkMax.h"
#include "gyro/pigeonIMU.h"
#include "swerveDrive/swerveDriveMode.h"
#include "util/robotStatus.h"


class auto_commands {
 public:
  
  double kpDist = 0.26, kiDist = 0.000001, kdDist = 0.0, kIz = 0, kFF = 0;
  double drive_error_margin = 0.08;
  double sumRotations = 0;

  //Heading是机器人行走的方向，Direction是车头面向的方向。

  void initAutoCommands(gyroPig* gyro_pig, frc_8011::swerveDriveMode driveMode, int mode, robotStatus* status);
  void move(double maxSpeed, double targetDist, double heading);
  //void spinTo(double heading);
  void updatePosition(double addX, double addY);
  void updateDirection(double newDriection);
  double getPreviousDirection();
  double getCurrentEncoderDist(double position);
  void updateRotations(double leftFront_encoder,double rightRear_encoder);
  void printPosition();
  
  double* position;
  gyroPig* gyro; 
  frc_8011::swerveDriveMode drive;
  double rotationsTravelled;

};
