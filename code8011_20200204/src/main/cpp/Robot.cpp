

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include<frc/Joystick.h>
#include<swerveDrive/swerveDriveMode.h>
#include<util/limelight.h>

frc_8011::swerveDriveMode driveMode;
frc::Joystick _joy{0};
limelight lime;

void Robot::RobotInit() {
  driveMode.initMotor();
}

void Robot::RobotPeriodic() { frc2::CommandScheduler::GetInstance().Run(); }


void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}


void Robot::AutonomousInit() {
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  driveMode.initMotor();
}

void Robot::TeleopPeriodic() {
  double X=_joy.GetRawAxis(4);
  double Y=-1*_joy.GetY();
  double R=_joy.GetRawAxis(3);

   frc::SmartDashboard::PutNumber("X",X);
     frc::SmartDashboard::PutNumber("Y", Y);
     frc::SmartDashboard::PutNumber("R",R);
  if(X<0.1&&X>-0.1)  {X=0.0;}
  if(Y<0.1&&Y>-0.1)  {Y=0.0;}
  if(R<0.1&&R>-0.1)  {R=0.0;}
  driveMode.execute(Y,X,R);

  //aiming and tracking mode
  if (_joy.GetRawButtonPressed(6)){
    std::vector<double> vect = lime.aim_range();
    driveMode.execute(vect.at(0),vect.at(1),vect.at(2));
  }
}


void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
