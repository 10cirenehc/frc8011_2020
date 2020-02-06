/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <rev/CANSparkMax.h>
#include "ctre/Phoenix.h"
#include <frc/PIDController.h>


class wheelDrive {
 public:

  wheelDrive(int drive, int turn);
  void drive(double speed, double angle);

  
  /*
  //initiate drivetrain motors (total of 8)
  rev::CANSparkMax *rfDrive = new rev::CANSparkMax(11, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax *lfDrive = new rev::CANSparkMax(12, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax *rbDrive = new rev::CANSparkMax(13, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax *lbDrive = new rev::CANSparkMax(14, rev::CANSparkMax::MotorType::kBrushless);

  //initiate turning motors 
  TalonSRX *rfTurn = new TalonSRX(0);
  TalonSRX *lfTurn = new TalonSRX(1);
  TalonSRX *rbTurn = new TalonSRX(2);
  TalonSRX *lbTurn = new TalonSRX(3);
  */

 private: 
  rev::CANSparkMax *driveMotor;
  TalonSRX *turnMotor;
  frc::PIDController *pid;
  double currentPos;
  const double MAX_VOLTS = 12; 
  
  };
