#pragma once

#include <swerveDrive/swerve.h>
#include "util/robotStatus.h"
namespace frc_8011{
class swerveDriveMode
{
private:
    //创建底盘四个轮子的对象
    frc_8011::Swerve _rightFrontWheel{1,5};
    frc_8011::Swerve _leftFrontWheel{2,6};
    frc_8011::Swerve _leftRearWheel{3,7};
    frc_8011::Swerve _rightRearWheel{4,8};
    double preDegree_rightF=0.0;//用来记录之前的角度
    double preDegree_leftF=0.0;//用来记录之前的角度
    double preDegree_leftR=0.0;//用来记录之前的角度
    double preDegree_rightR=0.0;//用来记录之前的角度
public:
    //马达初始化
    void initMotor();
    //设置零位
    void setZeroPoint();
    //底盘马达运行
    void execute(double FWD,double STR,double RCW);
    //马达复位初始化
    void resetInit();
    //马达位置复原
    void resetMotor();
    //PID控制底盘行驶距离
    void positionControl(double position);
    //PID控制底盘按照一定速度行驶
    void speedControl(double speed);
    //获取底盘各个马达的数据
    void getMotorStatus(robotStatus* status);
    //停止前进马达
    void stopDrive();
    //测试旋转马达
    void testSteerMotor(double speed);
    

    //转身策略
    void turnRight(double error_angle);

    void turnLeft(double error_angle);
    void turnDrive1();    //右转第一步
    void turnDrive2();
    void turnDrive3();
    void turnDrive4();
    void turnLeftDrive1();   //左转第一步
    void turnLeftDrive2();
    void turnLeftDrive3();
    void turnLeftDrive4();
};
}
