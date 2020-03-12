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
    steerMotor->ConfigSelectedFeedbackSensor(
				FeedbackDevice::CTRE_MagEncoder_Absolute, 0,30);
    int absolutePosition=steerMotor->GetSelectedSensorPosition();
    steerMotor->SetSelectedSensorPosition(absolutePosition, 0,30);
    //steerMotor->ConfigFeedbackNotContinuous(true);
    steerMotor->SetSensorPhase(true);
    steerMotor->Config_kF(0, 0.0, 30);
	steerMotor->Config_kP(0, 0.3895, 30);
	steerMotor->Config_kI(0, 0.00001, 30);
	steerMotor->Config_kD(0, 0.0, 30);
}

void frc_8011::Swerve::setZeroPoint(){
    steerMotor->SetSelectedSensorPosition(0,0,30);
}

void frc_8011::Swerve::setDriveMotorSpeed(double speed){
    driveMotor->Set(speed*0.2);
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

void frc_8011::Swerve::setDriveMotorPIDSpeed(double speed){
    m_pidController->SetReference(speed,rev::ControlType::kVelocity);
}

void frc_8011::Swerve::setSteerMotorSpeed(double speed){
    steerMotor->Set(ControlMode::PercentOutput,speed);
}

void frc_8011::Swerve::setSteerMotorPosition(double position){
    steerMotor->Set(ControlMode::Position,position);
}

double frc_8011::Swerve::getSteerEncoder(){
    return steerMotor->GetSelectedSensorPosition(0);
}

double frc_8011::Swerve::getDriveEncoder(){
    rev::CANEncoder m_encoder=driveMotor->GetEncoder();
    return m_encoder.GetPosition();
}

double frc_8011::Swerve::getDriveSpeed(){
    rev::CANEncoder m_encoder=driveMotor->GetEncoder();
    return m_encoder.GetVelocity();
}

double frc_8011::Swerve::getSteerSpeed(){
    return steerMotor->GetSelectedSensorVelocity();
}

void frc_8011::Swerve::stopDriveMotor(){
    driveMotor->StopMotor();
}