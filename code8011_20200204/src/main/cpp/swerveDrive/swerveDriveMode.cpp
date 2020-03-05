    #include <swerveDrive/swerveDriveMode.h>
    #include <util/degreeAndSpeedMath.h>
    #include <frc/smartdashboard/SmartDashboard.h>
    #include <cmath>

    void frc_8011::swerveDriveMode::initMotor(){
        _leftFrontWheel.motorInit();
        _rightFrontWheel.motorInit();
        _leftRearWheel.motorInit();
        _rightRearWheel.motorInit();
    }

    void frc_8011::swerveDriveMode::setZeroPoint(){
        _rightFrontWheel.setZeroPoint();
        _leftFrontWheel.setZeroPoint();
        _leftRearWheel.setZeroPoint();
        _rightRearWheel.setZeroPoint();
    }

    void frc_8011::swerveDriveMode::execute(double FWD,double STR,double RCW){

        double rightFrontPosition,rightFrontSpeed;
        double leftFrontPosition,leftFrontSpeed;
        double leftRearPosition,leftRearSpeed;
        double rightRearPosition,rightRearSpeed;
        double wS1Degree;
        double wS2Degree;
        double wS3Degree;
        double wS4Degree;


     if(FWD>0){
        rightFrontSpeed=frc_8011::WS1Speed(FWD,STR,RCW);
        leftFrontSpeed=frc_8011::WS2Speed(FWD,STR,RCW);
        leftRearSpeed=frc_8011::WS3Speed(FWD,STR,RCW);
        rightRearSpeed=frc_8011::WS4Speed(FWD,STR,RCW);

        wS1Degree=frc_8011::WS1Degree(FWD,STR,RCW);
        wS2Degree=frc_8011::WS2Degree(FWD,STR,RCW);
        wS3Degree=frc_8011::WS3Degree(FWD,STR,RCW);
        wS4Degree=frc_8011::WS4Degree(FWD,STR,RCW);
    }

    else if(FWD<0){
        FWD=-1*FWD;
        rightRearSpeed=-1*frc_8011::WS1Speed(FWD,STR,RCW);
        leftRearSpeed=-1*frc_8011::WS2Speed(FWD,STR,RCW);
        leftFrontSpeed=-1*frc_8011::WS3Speed(FWD,STR,RCW);
        rightFrontSpeed=-1*frc_8011::WS4Speed(FWD,STR,RCW);

        wS1Degree=frc_8011::WS4Degree(FWD,STR,RCW);
        wS2Degree=frc_8011::WS3Degree(FWD,STR,RCW);
        wS3Degree=frc_8011::WS2Degree(FWD,STR,RCW);
        wS4Degree=frc_8011::WS1Degree(FWD,STR,RCW);

    }

    if(FWD==0.0&&STR==0.0&&RCW==0.0){
            preDegree_rightF=0.0;
            preDegree_leftF=0.0;
            preDegree_leftR=0.0;
            preDegree_rightR=0.0;
    }
        
    if(std::abs(wS1Degree-preDegree_rightF)>180){
        if(preDegree_rightF>0){
            wS1Degree=wS1Degree+360;
        }
        else{
            wS1Degree=wS1Degree-360;
        }
    }

    if(std::abs(wS2Degree-preDegree_leftF)>180){
        if(preDegree_leftF>0){
            wS2Degree=wS2Degree+360;
            }
        else{
            wS2Degree=wS2Degree-360;
            }
        }

    if(std::abs(wS3Degree-preDegree_leftR)>180){
        if(preDegree_leftR>0){
            wS3Degree=wS3Degree+360;
            }
        else{
            wS3Degree=wS3Degree-360;
            }
    }

    if(std::abs(wS4Degree-preDegree_rightR)>180){
        if(preDegree_rightR>0){
            wS4Degree=wS4Degree+360;
        }
        else{
            wS4Degree=wS4Degree-360;
            }
    }

    rightFrontPosition=frc_8011::encoderPosition(wS1Degree);
    leftFrontPosition=frc_8011::encoderPosition(wS2Degree);
    leftRearPosition=frc_8011::encoderPosition(wS3Degree);
    rightRearPosition=frc_8011::encoderPosition(wS4Degree);

    preDegree_rightF=wS1Degree;
    preDegree_leftF=wS2Degree;
    preDegree_leftR=wS3Degree;
    preDegree_rightR=wS4Degree;

    //配置速度
    _rightFrontWheel.setDriveMotorSpeed(rightFrontSpeed);
    _leftFrontWheel.setDriveMotorSpeed(leftFrontSpeed);
    _leftRearWheel.setDriveMotorSpeed(leftRearSpeed);
    _rightRearWheel.setDriveMotorSpeed(rightRearSpeed);

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
    frc::SmartDashboard::PutNumber("WS1Degree",wS1Degree);
    frc::SmartDashboard::PutNumber("WS2Degree",wS2Degree);
    frc::SmartDashboard::PutNumber("WS3Degree", wS3Degree);
    frc::SmartDashboard::PutNumber("WS4Degree",wS4Degree);
    frc::SmartDashboard::PutNumber("leftFrontPosition",leftFrontPosition);
    frc::SmartDashboard::PutNumber("rightFrontPosition",rightFrontPosition);
    frc::SmartDashboard::PutNumber("leftRearPosition", leftRearPosition);
    frc::SmartDashboard::PutNumber("rightRearPosition",rightRearPosition);

 }

 
    void frc_8011::swerveDriveMode::resetMotor(){
            _rightFrontWheel.setSteerMotorPosition(2711);
            _leftFrontWheel.setSteerMotorPosition(1824);
            _leftRearWheel.setSteerMotorPosition(84);
            _rightRearWheel.setSteerMotorPosition(248);
        }

    void frc_8011::swerveDriveMode::positionControl(double position){
            _rightFrontWheel.setDriveMotorPosition(position);
            _leftFrontWheel.setDriveMotorPosition(position);
            _leftRearWheel.setDriveMotorPosition(position);
            _rightRearWheel.setDriveMotorPosition(position);
    }
    /**
     * 获取driveMotor的编码器的值
     */
    void frc_8011::swerveDriveMode::getDrivePosition(int p[4]){
        p[0]=_rightFrontWheel.getDriveEncoder();
        p[1]=_leftFrontWheel.getDriveEncoder();
        p[2]=_leftRearWheel.getDriveEncoder();
        p[3]=_rightRearWheel.getDriveEncoder();
    }

    /**
     * 获取steerMotor的编码器的值
     */
    void frc_8011::swerveDriveMode::getSteerPosition(int p[4]){
        p[0]=_rightFrontWheel.getSteerEncoder();
        p[1]=_leftFrontWheel.getSteerEncoder();
        p[2]=_leftRearWheel.getSteerEncoder();
        p[3]=_rightRearWheel.getSteerEncoder();
    }

    /**
     * 右转
     */
    void frc_8011::swerveDriveMode::turnRight(double error_angle){
        if(error_angle>=-90){
           turnDrive1();
         }
         else if(-90>error_angle&&error_angle>=-180){
           turnDrive2();
         }
         else if(-180>error_angle&&error_angle>=-270){
            turnDrive3();
         }
         else if(-270>error_angle&&error_angle>=-360){
            turnDrive4();
         }
    }
    /**
     * 左转
     */
    void frc_8011::swerveDriveMode::turnLeft(double error_angle){
         if(error_angle<=90){
           turnLeftDrive1();
         }
         else if(90<error_angle&&error_angle<=180){
           turnLeftDrive2();
         }
         else if(180<error_angle&&error_angle<270){
            turnLeftDrive3();
         }
         else if(270<error_angle&&error_angle<360){
            turnLeftDrive4();
         }
    }
    /**
     * 右转第一步
     */
    void frc_8011::swerveDriveMode::turnDrive1(){
            _rightFrontWheel.setSteerMotorPosition(-512);
            _rightFrontWheel.setDriveMotorSpeed(0.0);
            _leftFrontWheel.setSteerMotorPosition(0.0);
            _leftFrontWheel.setDriveMotorSpeed(0.7070);
            _leftRearWheel.setSteerMotorPosition(-512);
            _leftRearWheel.setDriveMotorSpeed(1.0);
            _rightRearWheel.setSteerMotorPosition(1024);
            _rightRearWheel.setDriveMotorSpeed(-0.7070);

    }
    /**
     * 右转第二步
     */
    void frc_8011::swerveDriveMode::turnDrive2(){
            _rightFrontWheel.setSteerMotorPosition(0.0);
            _rightFrontWheel.setDriveMotorSpeed(-0.707);
            _leftFrontWheel.setSteerMotorPosition(512);
            _leftFrontWheel.setDriveMotorSpeed(0);
            _leftRearWheel.setSteerMotorPosition(-1024);
            _leftRearWheel.setDriveMotorSpeed(0.707);
            _rightRearWheel.setSteerMotorPosition(512);
            _rightRearWheel.setDriveMotorSpeed(-1.0);

    }
    /**
     * 右转第三步
     */
    void frc_8011::swerveDriveMode::turnDrive3(){
            _rightFrontWheel.setSteerMotorPosition(-512);
            _rightFrontWheel.setDriveMotorSpeed(-1.0);
            _leftFrontWheel.setSteerMotorPosition(1024);
            _leftFrontWheel.setDriveMotorSpeed(0.707);
            _leftRearWheel.setSteerMotorPosition(-512);
            _leftRearWheel.setDriveMotorSpeed(0.0);
            _rightRearWheel.setSteerMotorPosition(0.0);
            _rightRearWheel.setDriveMotorSpeed(-0.707);

    }
    /**
     * 右转第四步
     */
    void frc_8011::swerveDriveMode::turnDrive4(){
            _rightFrontWheel.setSteerMotorPosition(-1024);
            _rightFrontWheel.setDriveMotorSpeed(-0.707);
            _leftFrontWheel.setSteerMotorPosition(512);
            _leftFrontWheel.setDriveMotorSpeed(1.0);
            _leftRearWheel.setSteerMotorPosition(0.0);
            _leftRearWheel.setDriveMotorSpeed(0.707);
            _rightRearWheel.setSteerMotorPosition(512);
            _rightRearWheel.setDriveMotorSpeed(0.0);
    }

    /**
     * 左转第一步
     */
    void frc_8011::swerveDriveMode::turnLeftDrive1(){
            _rightFrontWheel.setSteerMotorPosition(0);
            _rightFrontWheel.setDriveMotorSpeed(-0.707);
            _leftFrontWheel.setSteerMotorPosition(512);
            _leftFrontWheel.setDriveMotorSpeed(0);
            _leftRearWheel.setSteerMotorPosition(-1024);
            _leftRearWheel.setDriveMotorSpeed(0.707);
            _rightRearWheel.setSteerMotorPosition(512);
            _rightRearWheel.setDriveMotorSpeed(1.0);
    }
    /**
     * 左转第二步
     */
    void frc_8011::swerveDriveMode::turnLeftDrive2(){
            _rightFrontWheel.setSteerMotorPosition(512);
            _rightFrontWheel.setDriveMotorSpeed(0);
            _leftFrontWheel.setSteerMotorPosition(0);
            _leftFrontWheel.setDriveMotorSpeed(-0.707);
            _leftRearWheel.setSteerMotorPosition(-512);
            _leftRearWheel.setDriveMotorSpeed(-1.0);
            _rightRearWheel.setSteerMotorPosition(1024);
            _rightRearWheel.setDriveMotorSpeed(0.707);
    }
    /**
     * 左转第三步
     */
    void frc_8011::swerveDriveMode::turnLeftDrive3(){
            _rightFrontWheel.setSteerMotorPosition(1024);
            _rightFrontWheel.setDriveMotorSpeed(-0.707);
            _leftFrontWheel.setSteerMotorPosition(512);
            _leftFrontWheel.setDriveMotorSpeed(1.0);
            _leftRearWheel.setSteerMotorPosition(-1024);
            _leftRearWheel.setDriveMotorSpeed(-0.707);
            _rightRearWheel.setSteerMotorPosition(512);
            _rightRearWheel.setDriveMotorSpeed(0.0); 
    }
    /**
     * 左转第四步
     */
    void frc_8011::swerveDriveMode::turnLeftDrive4(){
            _rightFrontWheel.setSteerMotorPosition(-512);
            _rightFrontWheel.setDriveMotorSpeed(1.0);
            _leftFrontWheel.setSteerMotorPosition(1024);
            _leftFrontWheel.setDriveMotorSpeed(-0.707);
            _leftRearWheel.setSteerMotorPosition(-512);
            _leftRearWheel.setDriveMotorSpeed(0);
            _rightRearWheel.setSteerMotorPosition(0);
            _rightRearWheel.setDriveMotorSpeed(0.707); 
    }

    /**
     * 按照指定距离走直线
     */
    void frc_8011::swerveDriveMode::goStraight(double error_angle,double distance){
        
    }