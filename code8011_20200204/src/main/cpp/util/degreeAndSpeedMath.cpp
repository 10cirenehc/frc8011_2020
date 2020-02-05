#include<util/degreeAndSpeedMath.h>
#include<math.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace std;
using namespace frc_8011;
const double PI=3.1416;
const double ENCODERCOUNT=4096.0;
double length=27.56;//为70厘米
double width=27.56;

//double R=sqrt(length*length+width*width);
double R=38.98;

double frc_8011::calculateA(double STR,double RCW){
    return STR-RCW*(length/width);
}

double frc_8011::calculateB(double STR,double RCW){
    return STR+RCW*(length/width);
}

double frc_8011::calculateC(double FWD,double RCW){
    return FWD-RCW*(width/R);
}

double frc_8011::calculateD(double FWD,double RCW){
    return FWD+RCW*(width/R);
}

double frc_8011::calculateWS1(double FWD,double STR,double RCW){
    double B=calculateB(STR,RCW);
    double C=calculateC(FWD,RCW);
    return sqrt(B*B+C*C);
}

double frc_8011::calculateWS2(double FWD,double STR,double RCW){
    double B=calculateB(STR,RCW);
    double D=calculateD(FWD,RCW);
    return sqrt(B*B+D*D);
}

double frc_8011::calculateWS3(double FWD,double STR,double RCW){
    double A=calculateA(STR,RCW);
    double D=calculateD(FWD,RCW);
    return sqrt(A*A+D*D);
}

double frc_8011::calculateWS4(double FWD,double STR,double RCW){
    double A=calculateB(STR,RCW);
    double C=calculateC(FWD,RCW);
    return sqrt(A*A+C*C);
}

double frc_8011::MaxWS(double WS1,double WS2,double WS3,double WS4){
    double max=WS1;
    if(WS2>=max){
        max=WS2;
    }
    else if (WS3>=max)
    {
        max=WS3;
    }
    else if (WS4>=max)
    {
        max=WS4;
    }   
    return max;
    
}

double frc_8011::WS1Speed(double FWD,double STR,double RCW){
    double WS1=calculateWS1(FWD,STR,RCW);
    double WS2=calculateWS2(FWD,STR,RCW);
    double WS3=calculateWS3(FWD,STR,RCW);
    double WS4=calculateWS4(FWD,STR,RCW);
    double max=MaxWS(WS1,WS2,WS3,WS4);
    if (max>1)
    {
        return WS1/max;
    }
    else
    {
        return WS1;
    }
}

double frc_8011::WS2Speed(double FWD,double STR,double RCW){
    double WS1=calculateWS1(FWD,STR,RCW);
    double WS2=calculateWS2(FWD,STR,RCW);
    double WS3=calculateWS3(FWD,STR,RCW);
    double WS4=calculateWS4(FWD,STR,RCW);
    double max=MaxWS(WS1,WS2,WS3,WS4);
    if (max>1)
    {
        return WS2/max;
    }
    else
    {
        return WS2;
    }
}

double frc_8011::WS3Speed(double FWD,double STR,double RCW){
    double WS1=calculateWS1(FWD,STR,RCW);
    double WS2=calculateWS2(FWD,STR,RCW);
    double WS3=calculateWS3(FWD,STR,RCW);
    double WS4=calculateWS4(FWD,STR,RCW);
    double max=MaxWS(WS1,WS2,WS3,WS4);
    if (max>1)
    {
        return WS3/max;
    }
    else
    {
        return WS3;
    }
}

double frc_8011::WS4Speed(double FWD,double STR,double RCW){
    double WS1=calculateWS1(FWD,STR,RCW);
    double WS2=calculateWS2(FWD,STR,RCW);
    double WS3=calculateWS3(FWD,STR,RCW);
    double WS4=calculateWS4(FWD,STR,RCW);
    double max=MaxWS(WS1,WS2,WS3,WS4);
    if (max>1)
    {
        return WS4/max;
    }
    else
    {
        return WS4;
    }
}

double frc_8011::WS1Degree(double FWD,double STR,double RCW){
    
    double B=calculateB(STR,RCW);
    double C=calculateC(FWD,RCW);

    if(B==0&&C==0){
        return 0.0;
    }
    else{
        return atan2(B,C)*180/PI;
    }
}

double frc_8011::WS2Degree(double FWD,double STR,double RCW){
    double B=calculateB(STR,RCW);
    double D=calculateD(FWD,RCW);
    if(B==0&&D==0){
        return 0.0;
    }
    else{
        return atan2(B,D)*180/PI;
    }
}

double frc_8011::WS3Degree(double FWD,double STR,double RCW){
    double A=calculateA(STR,RCW);
    double D=calculateD(FWD,RCW);
    if(A==0&&D==0){
        return 0.0;
    }
    else{
        return atan2(A,D)*180/PI;
    }
}

double frc_8011::WS4Degree(double FWD,double STR,double RCW){
     double A=calculateA(STR,RCW);
     double C=calculateC(FWD,RCW);
    if(A==0&&C==0){
        return 0.0;
    }
    else{
        return atan2(A,C)*180/PI;
    }
}

double frc_8011::encoderPosition(double degree){
    return degree/360.0*ENCODERCOUNT;
}

