
#pragma once
#include <frc/smartdashboard/Smartdashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include<map>
#include <string>
using namespace std;

  const double MOUNT_ANGLE = 0;// 13.13;        //摄像头倾斜角度
  const double MOUNT_HEIGHT = 0.36;             //摄像头高度
  const double TARGET_HEIGHT = 1.3;             //目标对象的高度
  const double PI = 3.1416;   

namespace frc_8011{
class limelight {
 public:
  limelight();
  
  /**
   * 获取捕捉对象与摄像头的距离
   */
  double getDistance();
  /**
   * 得到捕捉对象，与十字架的水平偏移角度
   * tx
   * 范围是LL1 -27度到27度，LL2-29.8度到29.8度
   */
  double getVertAngle();
  /**
   * 得到捕捉对象，与十字架的垂直偏移角度
   * ty
   * 范围是LL1 -20.5度到20.5度，LL2-24.85度到24.85度
   */
  double getHorAngle();//垂直偏移角度
  /**
   * 获取捕捉对象镜像的面积占比，从0~100%
   */
  double getArea();//面积占比
  /**
   * 倾斜或旋转（-90度到0度）
   */
  double getSkew();
  /**
   * 打印参数值
   */
  void printValues();
  /**
   * 镜头范围内是否捕捉到对象
   * true OR false
   */
  bool hasTarget();
  
  /**
   * 设置LED
   */
  void setLed(int modeNum);

 private:
   std::shared_ptr<NetworkTable> table;
  
 };
}