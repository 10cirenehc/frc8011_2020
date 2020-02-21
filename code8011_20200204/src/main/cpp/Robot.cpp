
#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/Joystick.h>
#include "swerveDrive/swerveDriveMode.h"
#include "limeLight/limeLightManage.h"
#include "gyro/pigeonIMU.h"
#include <util/commonUtil.h>
#include <cmath>
#include "auto/auto_commands.h"

using namespace frc_8011;

swerveDriveMode driveMode;
frc::Joystick _joy{0};
frc::Joystick _joy1{1};

limeLightManage limeMan;
gyroPig* gyro_Pig = new gyroPig(9);  //陀螺仪
auto_commands auton; 

int AUTON_MODE = 1; //选择自动程序模式

void Robot::RobotInit() {
  //driveMode.resetMotor();
  gyro_Pig->gyroInit();//初始化陀螺仪
}

void Robot::RobotPeriodic() { frc2::CommandScheduler::GetInstance().Run(); }


void Robot::DisabledInit() {
  //关闭limeLight的LED灯
   limeMan.setLimeLed(limeLightLedMode::mDisable);
}

void Robot::DisabledPeriodic() {}


void Robot::AutonomousInit() {
  //driveMode.resetInit();
  driveMode.initMotor();
  limeMan.setLimeLed(limeLightLedMode::mShootBall);
  auton.initAutoCommands(gyro_Pig, driveMode, AUTON_MODE);
}

void Robot::AutonomousPeriodic() {
  double X=0.0;
  double Y=0.0;
  double R=0.0;
  
   //Y=limeMan.aim_distance(4.0);
   //driveMode.positionControl(10);
  // driveMode.execute(Y,X,R);
    auton.move(0.8,1,0);
}

void Robot::TeleopInit() {
  //初始化底盘各个马达控制器
  driveMode.initMotor();
  //limeMan.setLimeLed(limeLightLedMode::mGetBall);
  
}

void Robot::TeleopPeriodic() {

  double X=_joy.GetRawAxis(4);
  double Y=-1*_joy.GetY();
  double R=_joy.GetRawAxis(3);//顺时针转动
  double R1=_joy.GetRawAxis(2); //逆时针转动
   //对信号进行过滤
  X=DB(X);
  Y=DB(Y);
  R=DB(R);
  R1=DB(R1);
  X=Cap(X,1);
  Y=Cap(Y,1);
  R=Cap(R,1);
  R1=Cap(R1,1);

  if(R1!=0.0){ //判断是顺时针转动还是逆时针
    R=-1*R1;
  }

   //角度对位
   if (_joy.GetRawButton(1)){
     limeMan.setLimeLed(limeLightLedMode::mShootBall);
      R=limeMan.aim_spin_angle();
   }
   
   if(_joy.GetRawButton(2)){
    //  if(gyro_Pig.gyroReady()&&abs(gyro_Pig.getCurrentAngle())<0.5){
    //   limeMan.setLimeLed(limeLightLedMode::mShootBall); 
    //    X=limeMan.aim_hor_angle();
    //  }
    //   else{
    //     X=0.0;
    //     R=-1*gyro_Pig.getGyroSpin(0.0);
    //   }
    limeMan.setLimeLed(limeLightLedMode::mShootBall); 
    X=limeMan.aim_hor_angle();
   }
   
    driveMode.execute(Y,X,R);
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
