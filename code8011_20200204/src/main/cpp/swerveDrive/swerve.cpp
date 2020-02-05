#include <swerveDrive/swerve.h>
frc_8011::Swerve::Swerve(int driveId,int steerId){
    driveMotor=new rev::CANSparkMax(driveId,rev::CANSparkMax::MotorType::kBrushless);
    steerMotor=new TalonSRX(steerId);
}

void frc_8011::Swerve::motorInit(){
    //driveMotor->
    steerMotor->ConfigFactoryDefault();
    int absolutePosition=steerMotor->GetSelectedSensorPosition(0) & 0xFFF;
    steerMotor->ConfigSelectedFeedbackSensor(
				FeedbackDevice::PulseWidthEncodedPosition, 0,30);
    steerMotor->SetSelectedSensorPosition(0.0, 0,30);
    steerMotor->SetSensorPhase(true);
    steerMotor->Config_kF(0, 0.0, 30);
	steerMotor->Config_kP(0, 0.38, 30);
	steerMotor->Config_kI(0, 0.00001, 30);
	steerMotor->Config_kD(0, 0.0, 30);
}

void frc_8011::Swerve::setDriveMotorSpeed(double speed){
    driveMotor->Set(speed);
}

int frc_8011::Swerve::getDriveMotorId(){
    return driveMotor->GetDeviceId();
}

void frc_8011::Swerve::setDriveMotorPosition(double position){
    
}

void frc_8011::Swerve::setSteerMotorSpeed(double speed){
    steerMotor->Set(ControlMode::PercentOutput,speed);
}

void frc_8011::Swerve::setSteerMotorPosition(double position){
    steerMotor->Set(ControlMode::Position,position);
}

double frc_8011::Swerve::getEncoder(){
    return steerMotor->GetSelectedSensorPosition();
}