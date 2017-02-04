#include <WPILib.h>

#include "RobotConstants.h"
#include "RobotSubsystem.h"
#include <CameraServoSubsystem.h>

CameraServoSubsystem::CameraServoSubsystem(EntechRobot *pRobot, std::string name)
    : RobotSubsystem(pRobot, name)
	, m_tiltServo(NULL)
{
}

CameraServoSubsystem::~CameraServoSubsystem() {}

/********************************** Init Routines **********************************/

void CameraServoSubsystem::RobotInit()
{
    m_tiltServo = new frc::Servo(c_cameraServoUD);
    frc::LiveWindow::GetInstance()->AddActuator("CameraSubsystem", "Servo", m_tiltServo);
}

void CameraServoSubsystem::DisabledInit() {}

void CameraServoSubsystem::TeleopInit() {}

void CameraServoSubsystem::AutonomousInit() {}

void CameraServoSubsystem::TestInit() {}

/********************************** Periodic Routines **********************************/

void CameraServoSubsystem::DisabledPeriodic() { TeleopPeriodic(); }

void CameraServoSubsystem::TeleopPeriodic()
{
}

void CameraServoSubsystem::AutonomousPeriodic() { TeleopPeriodic();  }

void CameraServoSubsystem::TestPeriodic() {  }

void CameraServoSubsystem::SetTilt(double value)
{
    m_tiltServo->Set(value);
}

