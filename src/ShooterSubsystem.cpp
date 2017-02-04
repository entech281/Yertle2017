#include <WPILib.h>

#include "ShooterSubsystem.h"
#include "RobotSubsystem.h"
#include "RobotConstants.h"

ShooterSubsystem::ShooterSubsystem(EntechRobot *pRobot, std::string name)
	: RobotSubsystem(pRobot, name)
	, m_shooterMotor(NULL)
	, m_shooterSpeed(0.0)
	, m_zeroSensor(NULL)
	, m_state(kOff)
	, m_currentAngle(0.)
	, m_targetAngle(0.)
	, m_timer(NULL)
	, m_position_controller(NULL)
{
}

ShooterSubsystem::~ShooterSubsystem() {}

/********************************** Init Routines **********************************/

void ShooterSubsystem::RobotInit()
{
	m_shooterMotor = new CANTalon(c_shooterMotor_CANid);
	m_shooterSpeed = 0.0;
	m_zeroSensor = new frc::DigitalInput(c_shooterSensorPort);
    m_shooterMotor->ConfigEncoderCodesPerRev(30);
    m_shooterMotor->SetEncPosition(0.0);
    m_shooterMotor->SetPosition(0.0);
    m_timer = new frc::Timer();
    m_timer->Stop();
    m_position_controller = new frc::PIDController(c_pid_position_p, c_pid_position_i, c_pid_position_d, this, this);
    m_position_controller->Disable();
    m_position_controller->SetSetpoint(0.0);
    m_position_controller->SetOutputRange(-c_pid_position_max, c_pid_position_max);
    m_position_controller->SetAbsoluteTolerance(c_positionTolerance);
    m_position_controller->SetContinuous(true);
    m_position_controller->SetInputRange(0.0, 360.0);

    frc::LiveWindow::GetInstance()->AddActuator("ShooterSubsystem", "Shooter Motor", m_shooterMotor);
}

void ShooterSubsystem::DisabledInit() {}

void ShooterSubsystem::TeleopInit()
{
	m_state = kOff;
}

void ShooterSubsystem::AutonomousInit()
{
    // Assume starting position is correct
	m_state = kOff;
    m_shooterMotor->SetEncPosition(0.0);
    m_shooterMotor->SetPosition(0.0);
}

void ShooterSubsystem::TestInit() {}

/********************************** Periodic Routines **********************************/

void ShooterSubsystem::DisabledPeriodic()
{
	m_shooterMotor->Set(0.0);
	m_state = kOff;
}


void ShooterSubsystem::TeleopPeriodic()
{
	m_currentAngle = m_shooterMotor->GetPosition();
	while (m_currentAngle > 360.)
		m_currentAngle -= 360.;
	while (m_currentAngle < 0.)
		m_currentAngle += 360.;
    switch (m_state) {
    case kOff:
    	Off();
    	break;
    case kOn:
    	break;
    case kLost:
        if (m_zeroSensor->Get()) {
            frc::SmartDashboard::PutString("Shooter Position", "Lost");
            SetSpeed(c_slowForward);
        } else {
            frc::SmartDashboard::PutString("Shooter Position", "Zero");
            Off();
            m_shooterMotor->SetEncPosition(0.0);
            m_state = kOff;
        }
        break;
    case kMovingTo:
        if (m_position_controller->OnTarget()) {
        	m_position_controller->Disable();
            Off();
            m_state = kOff;
        }
        break;
    case kShooting:
    case kKicking:
        if (m_timer->Get() > 1.0) {
            m_state = kOff;
            Off();
            MoveTo(0.0);
        } else {
            SetSpeed(0.65*c_fastBackward);
        }
        break;
    case kFiring:
        if (m_timer->Get() > 1.0) {
            m_state = kOff;
            Off();
            MoveTo(0.0);
        } else {
            SetSpeed(c_fastForward);
        }
    	break;
    }
    m_shooterMotor->Set(m_shooterSpeed);
    
    frc::SmartDashboard::PutNumber("Shooter Motor Position", m_currentAngle);
    frc::SmartDashboard::PutBoolean("Shooter Sensor?", m_zeroSensor->Get());
}

void ShooterSubsystem::AutonomousPeriodic()
{
    TeleopPeriodic();
}

void ShooterSubsystem::TestPeriodic()
{
}

void ShooterSubsystem::FastForward()
{
	m_position_controller->Disable();
	SetSpeed(c_fastForward);
	m_state = kOn;
}

void ShooterSubsystem::SlowForward()
{
	m_position_controller->Disable();
	SetSpeed(c_slowForward);
	m_state = kOn;
}

void ShooterSubsystem::FastBackward()
{
	m_position_controller->Disable();
	SetSpeed(c_fastBackward);
	m_state = kOn;
}

void ShooterSubsystem::SlowBackward()
{
	m_position_controller->Disable();
	SetSpeed(c_slowBackward);
	m_state = kOn;
}

void ShooterSubsystem::Kick()
{
	m_position_controller->Disable();
    SetSpeed(c_fastBackward);
    m_state = kKicking;
    m_timer->Stop();
    m_timer->Reset();
    m_timer->Start();
}

void ShooterSubsystem::Fire()
{
	m_position_controller->Disable();
    SetSpeed(c_fastForward);
    m_state = kFiring;
    m_timer->Stop();
    m_timer->Reset();
    m_timer->Start();
}

void ShooterSubsystem::MoveTo(double angle)
{
    m_state = kMovingTo;
    m_targetAngle = angle;
    m_position_controller->SetSetpoint(angle);
    m_position_controller->Enable();
}

void ShooterSubsystem::SetSpeed(double speed)
{
	m_shooterSpeed = speed;
}

void ShooterSubsystem::Off()
{
	m_position_controller->Disable();
    if ((m_state != kKicking) && (m_state != kFiring)) {
        SetSpeed(0.0);
    	m_state = kOff;
    }
}

bool ShooterSubsystem::Done()
{
    if (m_state == kOff)
        return true;
    return false;
}

void ShooterSubsystem::Abort(void)
{
    SetSpeed(0.0);
    m_position_controller->Disable();
    m_state = kOff;
}
double ShooterSubsystem::PIDGet()
{
	double angle;
	angle = m_shooterMotor->GetPosition();
	while (angle > 360.)
		angle -= 360.0;
	while (angle < 0.0)
		angle += 360.;
 	return angle;
}

void ShooterSubsystem::PIDWrite(double output)
{
	if (output > c_pid_position_max)
		output = c_pid_position_max;
	if (output < -c_pid_position_max)
		output = -c_pid_position_max;
	SetSpeed(-output);
}
