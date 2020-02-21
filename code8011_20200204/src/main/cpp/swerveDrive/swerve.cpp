#include <swerveDrive/swerve.h>

frc_8011::Swerve::Swerve(int driveId,int steerId){
    driveMotor=new rev::CANSparkMax(driveId,rev::CANSparkMax::MotorType::kBrushless);
    steerMotor=new TalonSRX(steerId);
    m_pidController=new rev::CANPIDController(driveMotor->GetPIDController());
    //m_encoder=driveMotor->GetEncoder();
}

void frc_8011::Swerve::motorInit(){
    m_pidController->SetP(0.1);
    m_pidController->SetI(1e-4);
    m_pidController->SetD(0.0);
    m_pidController->SetIZone(0.0);
    m_pidController->SetFF(0.0);
   
    steerMotor->ConfigFactoryDefault();
    int absolutePosition=steerMotor->GetSelectedSensorPosition();
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

rev::CANSparkMax* frc_8011::Swerve::getDriveMotor(){
    return driveMotor;
}

void frc_8011::Swerve::setDriveMotorPosition(double position){
    double rotations=position/0.33*6.67;
    m_pidController->SetReference(rotations,rev::ControlType::kPosition);
}

void frc_8011::Swerve::setSteerMotorSpeed(double speed){
    steerMotor->Set(ControlMode::PercentOutput,speed);
}

void frc_8011::Swerve::setSteerMotorPosition(double position){
    steerMotor->Set(ControlMode::Position,position);
}

double frc_8011::Swerve::getEncoder(){
    return steerMotor->GetSelectedSensorPosition(0);
}

int frc_8011::Swerve::steerMotorResetInit(){
    steerMotor->ConfigFactoryDefault();
    int absolutePosition=steerMotor->GetSelectedSensorPosition(0) & 0xFFF;
    steerMotor->ConfigSelectedFeedbackSensor(
				FeedbackDevice::PulseWidthEncodedPosition, 0,30);
    steerMotor->SetSelectedSensorPosition(absolutePosition, 0,30);
    steerMotor->SetSensorPhase(true);
    steerMotor->Config_kF(0, 0.0, 30);
	steerMotor->Config_kP(0, 0.385, 30);
	steerMotor->Config_kI(0, 0.00, 30);
	steerMotor->Config_kD(0, 0.0, 30);
    return absolutePosition;
}
void frc_8011::Swerve::steerMotorReset(double position){
    steerMotor->Set(ControlMode::Position,position);
}