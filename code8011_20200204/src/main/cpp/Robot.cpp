
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
#include "auto/autoMode.h"

using namespace frc_8011;

swerveDriveMode driveMode;
frc::Joystick _joy{0};
frc::Joystick _joy1{1};
limeLightManage limeMan;
gyroPig* gyro_Pig = new gyroPig(9);  //陀螺仪
auto_commands auton; 

int AUTON_MODE = 1; //选择自动程序模式
int init_gyro_angle=0.0;
int init_drive_dist[4];

/**----------------------------------Robot-------------------------------------------------**/
void Robot::RobotInit() {
  driveMode.initMotor();
  gyro_Pig->gyroInit();//初始化陀螺仪
}

void Robot::RobotPeriodic() { 
  frc2::CommandScheduler::GetInstance().Run(); 
//driveMode.resetMotor();
}
/**----------------------------------------------------------------------------------------**/

/**---------------------------------Disable------------------------------------------------**/
void Robot::DisabledInit() {
   //driveMode.resetMotor();
  //关闭limeLight的LED灯
   limeMan.setLimeLed(limeLightLedMode::mDisable);
}
void Robot::DisabledPeriodic() {
}
/**----------------------------------------------------------------------------------------**/

/**---------------------------------Auto---------------------------------------------------**/
void Robot::AutonomousInit() {
  //初始化底盘各个马达控制器
 // driveMode.setZeroPoint();
  //init_gyro_angle=gyro_Pig->getCurrentAngle(); //自动程序启动，获取最初角度和位置
  //driveMode.getDrivePosition(init_drive_dist);

//   limeMan.setLimeLed(limeLightLedMode::mShootBall);
   auton.initAutoCommands(gyro_Pig, driveMode, AUTON_MODE);
   auton.move(0.3,10.0,0);
   driveMode.execute(0.0,0.0,0.0);
}
void Robot::AutonomousPeriodic() {
  //frc2::CommandScheduler::GetInstance().Run(); 
  // double YXR[3];
  // //实时更新底盘的状态
  // double error_gyro_angle=init_gyro_angle-gyro_Pig->getCurrentAngle();
  // int current_drive_dist[4];
  // driveMode.getDrivePosition(current_drive_dist);
  // int error_drive_dist[4];
  // error_drive_dist[0]=init_drive_dist[0]-current_drive_dist[0];
  // error_drive_dist[1]=init_drive_dist[1]-current_drive_dist[1];
  // error_drive_dist[2]=init_drive_dist[2]-current_drive_dist[2];
  // error_drive_dist[3]=init_drive_dist[3]-current_drive_dist[3];
  // autoAction(error_gyro_angle,error_drive_dist,autoMODE::firstAuto,YXR);
  
  // driveMode.execute(YXR[0],YXR[1],YXR[2]);
 

}
/**----------------------------------------------------------------------------------------**/
/**-----------------------------------Tele-------------------------------------------------**/
void Robot::TeleopInit() {
  //初始化底盘各个马达控制器
  driveMode.setZeroPoint();
  //limeMan.setLimeLed(limeLightLedMode::mGetBall);
  
}

void Robot::TeleopPeriodic() {
  frc2::CommandScheduler::GetInstance().Run(); 
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

  int pos[4];
  driveMode.getSteerPosition(pos);
  frc::SmartDashboard::PutNumber("pos1",pos[0]);
  frc::SmartDashboard::PutNumber("pos2",pos[1]); 
  frc::SmartDashboard::PutNumber("pos3",pos[2]);
  frc::SmartDashboard::PutNumber("pos4",pos[3]);

  int posD[4];
  driveMode.getDrivePosition(posD);
  frc::SmartDashboard::PutNumber("posD1",posD[0]);
  frc::SmartDashboard::PutNumber("posD2",posD[1]); 
  frc::SmartDashboard::PutNumber("posD3",posD[2]);
  frc::SmartDashboard::PutNumber("posD4",posD[3]);

  if(R1!=0.0){ //判断是顺时针转动还是逆时针
    R=-1*R1;
  }

   //角度对位
   if (_joy.GetRawButton(1)){
     limeMan.setLimeLed(limeLightLedMode::mShootBall);
      R=limeMan.aim_spin_angle();
   }
   
   //角度对准，且在弧顶
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

   //右转转身
     if(_joy.GetRawButtonPressed(3)){
       double initial_position=gyro_Pig->getCurrentAngle();
       double n=0;
       while(_joy.GetRawButton(3)){
         double current_position=gyro_Pig->getCurrentAngle();
         double error_position=current_position-initial_position;
         error_position=error_position+360*n;
         frc::SmartDashboard::PutNumber("gyroAngle",error_position);
         driveMode.turnRight(error_position);//对角度进行判断，执行相应的动作
         if(error_position<-360){
           n=n+1;
         }
       }
       driveMode.execute(0.0,0.0,0.0);
     }
     
    //左转转身
      if(_joy.GetRawButtonPressed(4)){
       double initial_position=gyro_Pig->getCurrentAngle();
       double m=0;
       while(_joy.GetRawButton(4)){
         double current_position=gyro_Pig->getCurrentAngle();
         double error_position=current_position-initial_position;
         error_position=error_position-360*m;
         frc::SmartDashboard::PutNumber("gyroAngle",error_position);
         driveMode.turnLeft(error_position);
         if(error_position>360){
           m=m+1;
         }
       }
       driveMode.execute(0.0,0.0,0.0);
     }


    driveMode.execute(Y,X,R);
}
/**----------------------------------------------------------------------------------------**/
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif