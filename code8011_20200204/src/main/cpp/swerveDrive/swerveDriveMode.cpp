#include <swerveDrive/swerveDriveMode.h>
#include <util/degreeAndSpeedMath.h>
#include <frc/smartdashboard/SmartDashboard.h>

double leftFrontSpeed;
double rightFrontSpeed;
double leftRearSpeed;
double rightRearSpeed;
double WS1DegreeNum;
double WS2DegreeNum;
double WS3DegreeNum;
double WS4DegreeNum;
void shiftDegreeSpeed();

void frc_8011::swerveDriveMode::initMotor(){
     _leftFrontWheel.motorInit();
     _rightFrontWheel.motorInit();
     _leftRearWheel.motorInit();
     _rightRearWheel.motorInit();
}

void frc_8011::swerveDriveMode::execute(double FWD,double STR,double RCW){
    rightFrontSpeed=frc_8011::WS1Speed(FWD,STR,RCW);
    leftFrontSpeed=frc_8011::WS2Speed(FWD,STR,RCW);
    leftRearSpeed=frc_8011::WS3Speed(FWD,STR,RCW);
    rightRearSpeed=frc_8011::WS4Speed(FWD,STR,RCW);

    WS1DegreeNum=frc_8011::WS1Degree(FWD,STR,RCW);
    WS2DegreeNum=frc_8011::WS2Degree(FWD,STR,RCW);
    WS3DegreeNum=frc_8011::WS3Degree(FWD,STR,RCW);
    WS4DegreeNum=frc_8011::WS4Degree(FWD,STR,RCW);

    shiftDegreeSpeed();//检查是否需要转换

     double rightFrontPosition=frc_8011::encoderPosition(WS1DegreeNum);
     double leftFrontPosition=frc_8011::encoderPosition(WS2DegreeNum);
     double leftRearPosition=frc_8011::encoderPosition(WS3DegreeNum);
     double rightRearPosition=frc_8011::encoderPosition(WS4DegreeNum);
     //配置速度
     _rightFrontWheel.setDriveMotorSpeed(rightFrontSpeed*0.5);
     _leftFrontWheel.setDriveMotorSpeed(leftFrontSpeed*0.5);
     _leftRearWheel.setDriveMotorSpeed(leftRearSpeed*0.5);
     _rightRearWheel.setDriveMotorSpeed(rightRearSpeed*0.5);

     //配置转向角度
     _rightFrontWheel.setSteerMotorPosition(rightFrontPosition);
     _leftFrontWheel.setSteerMotorPosition(leftFrontPosition);
     _leftRearWheel.setSteerMotorPosition(leftRearPosition);
     _rightRearWheel.setSteerMotorPosition(rightRearPosition);

     frc::SmartDashboard::PutNumber("X", STR);
     frc::SmartDashboard::PutNumber("Y", FWD);
     frc::SmartDashboard::PutNumber("R", RCW);
     frc::SmartDashboard::PutNumber("leftFrontSpeed",leftFrontSpeed);
     frc::SmartDashboard::PutNumber("rightFrontSpeed",rightFrontSpeed);
     frc::SmartDashboard::PutNumber("leftRearSpeed",leftRearSpeed);
     frc::SmartDashboard::PutNumber("rightRearSpeed",rightRearSpeed);
     frc::SmartDashboard::PutNumber("leftFrontPosition",leftFrontPosition);
     frc::SmartDashboard::PutNumber("rightFrontPosition",rightFrontPosition);
     frc::SmartDashboard::PutNumber("leftRearPosition", leftRearPosition);
     frc::SmartDashboard::PutNumber("rightRearPosition",rightRearPosition);

     }
    void frc_8011::swerveDriveMode::resetInit(){
        _rightFrontWheel.steerMotorResetInit();
        _leftFrontWheel.steerMotorResetInit();
        _leftRearWheel.steerMotorResetInit();
        _rightRearWheel.steerMotorResetInit();
    }
    void frc_8011::swerveDriveMode::resetMotor(){
        _rightFrontWheel.steerMotorReset(2280);
        _leftFrontWheel.steerMotorReset(1750);
        _leftRearWheel.steerMotorReset(720);
        _rightRearWheel.steerMotorReset(1995);
    }

    void frc_8011::swerveDriveMode::positionControl(double position){
        _rightFrontWheel.setDriveMotorPosition(position);
        _leftFrontWheel.setDriveMotorPosition(position);
        _leftRearWheel.setDriveMotorPosition(position);
        _rightRearWheel.setDriveMotorPosition(position);
    }

    /**
     * 转角超过一定值后，直接转变速度方向
     */
     void shiftDegreeSpeed(){
         if(WS1DegreeNum>90||WS1DegreeNum<-90){
             if(WS1DegreeNum>90)
             WS1DegreeNum=WS1DegreeNum-180;
             else 
             WS1DegreeNum=WS1DegreeNum+180;
            rightFrontSpeed=-1*rightFrontSpeed; 
         }
         if(WS2DegreeNum>90||WS2DegreeNum<-90){
             if(WS2DegreeNum>90)
             WS2DegreeNum=WS2DegreeNum-180;
             else 
             WS2DegreeNum=WS2DegreeNum+180;
            leftFrontSpeed=-1*leftFrontSpeed; 
         }
         if(WS3DegreeNum>90||WS3DegreeNum<-90){
             if(WS3DegreeNum>90)
             WS3DegreeNum=WS3DegreeNum-180;
             else 
             WS3DegreeNum=WS3DegreeNum+180;
            leftRearSpeed=-1*leftRearSpeed; 
         }
         if(WS4DegreeNum>90||WS4DegreeNum<-90){
             if(WS4DegreeNum>90)
             WS4DegreeNum=WS4DegreeNum-180;
             else 
             WS4DegreeNum=WS4DegreeNum+180;
            rightRearSpeed=-1*rightRearSpeed; 
         }
     }