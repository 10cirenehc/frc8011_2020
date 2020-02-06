#pragma once

#include <swerveDrive/swerve.h>

namespace frc_8011{
class swerveDriveMode
{
private:
    //创建底盘四个轮子的对象
    frc_8011::Swerve _leftFrontWheel{3,5};
    frc_8011::Swerve _leftRearWheel{4,7};
    frc_8011::Swerve _rightFrontWheel{1,8};
    frc_8011::Swerve _rightRearWheel{2,6};
public:
    //马达初始化
    void initMotor();
    //底盘马达运行
    void execute(double FWD,double STR,double RCW);
};
}
