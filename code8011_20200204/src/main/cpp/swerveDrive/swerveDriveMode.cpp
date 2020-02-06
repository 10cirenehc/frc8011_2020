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

     frc::SmartDashboard::PutNumber("leftFrontPositionA",_leftFrontWheel.getEncoder());
     frc::SmartDashboard::PutNumber("rightFrontPositionA",_rightFrontWheel.getEncoder());
     frc::SmartDashboard::PutNumber("leftRearPositionA",_leftRearWheel.getEncoder());
     frc::SmartDashboard::PutNumber("rightRearPositionA",_rightRearWheel.getEncoder());
}

void frc_8011::swerveDriveMode::execute(double FWD,double STR,double RCW){

    leftFrontSpeed=frc_8011::WS1Speed(FWD,STR,RCW);
    rightFrontSpeed=frc_8011::WS2Speed(FWD,STR,RCW);
    leftRearSpeed=frc_8011::WS3Speed(FWD,STR,RCW);
    rightRearSpeed=frc_8011::WS4Speed(FWD,STR,RCW);

    WS1DegreeNum=frc_8011::WS1Degree(FWD,STR,RCW);
    WS2DegreeNum=frc_8011::WS2Degree(FWD,STR,RCW);
    WS3DegreeNum=frc_8011::WS3Degree(FWD,STR,RCW);
    WS4DegreeNum=frc_8011::WS4Degree(FWD,STR,RCW);

    shiftDegreeSpeed();//检查是否需要转换

     double leftFrontPosition=frc_8011::encoderPosition(WS1DegreeNum);
     double rightFrontPosition=frc_8011::encoderPosition(WS2DegreeNum);
     double leftRearPosition=frc_8011::encoderPosition(WS3DegreeNum);
     double rightRearPosition=frc_8011::encoderPosition(WS4DegreeNum);

    //  _leftFrontWheel.setDriveMotorSpeed(leftFrontSpeed);
    //  _rightFrontWheel.setDriveMotorSpeed(rightFrontSpeed);
    //  _leftRearWheel.setDriveMotorSpeed(leftRearSpeed);
    //  _rightRearWheel.setDriveMotorSpeed(rightRearSpeed);

    //  _leftFrontWheel.setSteerMotorPosition(leftFrontPosition);
    //  _rightFrontWheel.setSteerMotorPosition(rightFrontPosition);
    //  _leftRearWheel.setSteerMotorPosition(leftRearPosition);
    //  _rightRearWheel.setSteerMotorPosition(rightRearPosition);

    _leftFrontWheel.setDriveMotorSpeed(0.05);
     _rightFrontWheel.setDriveMotorSpeed(0.05);
     _leftRearWheel.setDriveMotorSpeed(0.05);
     _rightRearWheel.setDriveMotorSpeed(0.05);

     _leftFrontWheel.setSteerMotorPosition(0.1);
     _rightFrontWheel.setSteerMotorPosition(0.1);
     _leftRearWheel.setSteerMotorPosition(0.1);
     _rightRearWheel.setSteerMotorPosition(0.1);

     frc::SmartDashboard::PutNumber("leftFrontSpeed",leftFrontSpeed);
     frc::SmartDashboard::PutNumber("rightFrontSpeed",rightFrontSpeed);
     frc::SmartDashboard::PutNumber("leftRearSpeed",leftRearSpeed);
     frc::SmartDashboard::PutNumber("rightRearSpeed",rightRearSpeed);
     frc::SmartDashboard::PutNumber("WS1Degree",WS1DegreeNum);
     frc::SmartDashboard::PutNumber("WS2Degree",WS2DegreeNum);
     frc::SmartDashboard::PutNumber("WS3Degree",WS3DegreeNum);
     frc::SmartDashboard::PutNumber("WS4Degree",WS4DegreeNum);
     frc::SmartDashboard::PutNumber("leftFrontPosition",leftFrontPosition);
     frc::SmartDashboard::PutNumber("rightFrontPosition",rightFrontPosition);
     frc::SmartDashboard::PutNumber("leftRearPosition", leftRearPosition);
     frc::SmartDashboard::PutNumber("rightRearPosition",rightRearPosition);
     frc::SmartDashboard::PutNumber("leftFrontPositionA",_leftFrontWheel.getEncoder());
     frc::SmartDashboard::PutNumber("rightFrontPositionA",_rightFrontWheel.getEncoder());
     frc::SmartDashboard::PutNumber("leftRearPositionA",_leftRearWheel.getEncoder());
     frc::SmartDashboard::PutNumber("rightRearPositionA",_rightRearWheel.getEncoder());


     }
    //转角超过一定值后，直接转变速度方向
     void shiftDegreeSpeed(){
         if(WS1DegreeNum>90||WS1DegreeNum<-90){
             if(WS1DegreeNum>90)
             WS1DegreeNum=WS1DegreeNum-180;
             else 
             WS1DegreeNum=WS1DegreeNum+180;
            leftFrontSpeed=-1*leftFrontSpeed; 
         }
         if(WS2DegreeNum>90||WS2DegreeNum<-90){
             if(WS2DegreeNum>90)
             WS2DegreeNum=WS2DegreeNum-180;
             else 
             WS2DegreeNum=WS2DegreeNum+180;
            rightFrontSpeed=-1*rightFrontSpeed; 
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