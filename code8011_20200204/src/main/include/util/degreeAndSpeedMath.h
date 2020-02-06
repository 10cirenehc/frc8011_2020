#pragma once

//底盘的尺寸，暂时用英寸未单位

namespace frc_8011{


double calculateA(double STR,double RCW);
double calculateB(double STR,double RCW);
double calculateC(double FWD,double RCW);
double calculateD(double FWD,double RCW);

double calculateWS1(double FWD,double STR,double RCW);
double calculateWS2(double FWD,double STR,double RCW);
double calculateWS3(double FWD,double STR,double RCW);
double calculateWS4(double FWD,double STR,double RCW);

double MaxWS(double WS1,double WS2,double WS3,double WS4);

double WS1Speed(double FWD,double STR,double RCW);
double WS2Speed(double FWD,double STR,double RCW);
double WS3Speed(double FWD,double STR,double RCW);
double WS4Speed(double FWD,double STR,double RCW);

double WS1Degree(double FWD,double STR,double RCW);
double WS2Degree(double FWD,double STR,double RCW);
double WS3Degree(double FWD,double STR,double RCW);
double WS4Degree(double FWD,double STR,double RCW);

double encoderPosition(double degree);

}