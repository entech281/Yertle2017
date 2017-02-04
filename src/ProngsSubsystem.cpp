#include <WPILib.h>

#include "ProngsSubsystem.h"
#include "RobotSubsystem.h"
#include "RobotConstants.h"

ProngsSubsystem::ProngsSubsystem(EntechRobot *pRobot, std::string name)
	: RobotSubsystem(pRobot, name)
	, m_prongsSolenoid1(NULL)
	, m_prongsSolenoid2(NULL)
    , m_prongsUpSensor(NULL)
    , m_prongsDownSensor(NULL)
	, m_position(kUp)
    , m_timer(NULL)
	, lw(NULL)
{
}

ProngsSubsystem::~ProngsSubsystem(){}

/********************************** Init Routines **********************************/

void ProngsSubsystem::RobotInit()
{
	m_prongsSolenoid1 = new frc::Solenoid (c_compressorPCMid, c_prongsSolenoidChannel1);
	m_prongsSolenoid2 = new frc::Solenoid (c_compressorPCMid, c_prongsSolenoidChannel2);
    m_prongsUpSensor = new frc::DigitalInput(c_prongsUpSensorChannel);
    m_prongsUpSensor = new frc::DigitalInput(c_prongsDownSensorChannel);
    m_timer = new Timer();

	lw = frc::LiveWindow::GetInstance();
    lw->AddActuator("ProngsSubsystem", "Prongs Solenoid 1", m_prongsSolenoid1);
    lw->AddActuator("ProngsSubsystem", "Prongs Solenoid 2", m_prongsSolenoid1);
}

void ProngsSubsystem::AutonomousInit(){}

void ProngsSubsystem::TeleopInit(){}

void ProngsSubsystem::DisabledInit(){}

void ProngsSubsystem::TestInit(){}

/********************************** Periodic Routines **********************************/

void ProngsSubsystem::AutonomousPeriodic()
{
	TeleopPeriodic();
}

void ProngsSubsystem::TeleopPeriodic()
{
    if (m_position == kDown)
    {
    	m_prongsSolenoid1->Set(true);
    	m_prongsSolenoid2->Set(true);
        frc::SmartDashboard::PutString("Prongs Position", "Down");
    }
    else
    {
    	m_prongsSolenoid1->Set(false);
    	m_prongsSolenoid2->Set(false);
        frc::SmartDashboard::PutString("Prongs Position", "Up");
    }
}

void ProngsSubsystem::DisabledPeriodic() {}

void ProngsSubsystem::TestPeriodic()
{
}

void ProngsSubsystem::SetPosition(ArmPosition position)
{
    m_position = position;
    m_timer->Stop();
    m_timer->Reset();
    m_timer->Start();
}

bool ProngsSubsystem::Done(void)
{
    //if ((m_position == kDown) && m_prongsDownSensor->Get())
    //    return true;
    //if ((m_position == kUp) && m_prongsUpSensor->Get())
    //    return true;
    if (m_timer->Get() > 2.5)
        return true;
    return false;
}
