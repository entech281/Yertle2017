/*
 * BallPickUpSubsystem.cpp
 *
 *  Created on: Feb 6, 2016
 *      Author: admin
 */
#include "RobotConstants.h"
#include "BallPickUpSubsystem.h"
#include <string>

BallPickUpSubsystem::BallPickUpSubsystem(EntechRobot *pRobot, std::string name)
	: RobotSubsystem(pRobot, name)
        , m_motor(NULL)
        , m_speed(0.0)
        , m_pickupSensor(NULL)
        , m_timer(NULL)
        , m_timeout(2.0)
        , m_state(kNormal)
		, m_ballSeen(false)
{
}

void BallPickUpSubsystem::Off()
{
	if (m_state == kNormal)
		m_speed = 0.;
}
void BallPickUpSubsystem::Forward()
{
	if (m_state == kNormal)
		m_speed = -1.;
}
void BallPickUpSubsystem::Backward()
{
	if (m_state == kNormal)
		m_speed = 1.;
}
void BallPickUpSubsystem::BallOutForKicker()
{
	m_timer->Stop();
	m_timer->Reset();
	m_timer->Start();
	m_speed = 1.;
        if (m_pickupSensor->Get()) {
            m_state = kOutForKicker;
            m_timeout = 1.5;
        } else {
            m_state = kOutToSensor;
            m_timeout = 2.0;
        }
}

void BallPickUpSubsystem::BallInToHold()
{
	if (m_state == kNormal) {
		m_speed = -0.8;
		m_ballSeen = false;
		m_state = kInToHold;
	}
}

void BallPickUpSubsystem::BallOutForShooter()
{
	m_timer->Stop();
	m_timer->Reset();
	m_timer->Start();
	m_speed = -1.;
        m_timeout = 1.5;
	m_state = kOutForShooter;
}

void BallPickUpSubsystem::RobotInit(void)
{
	frc::LiveWindow *lw;
    m_motor = new CANTalon(c_BallPickUpmotor_CANid);
    m_speed = 0.;
    m_pickupSensor = new frc::DigitalInput(c_pickupSensorPort);
    m_timer = new frc::Timer();

    lw = frc::LiveWindow::GetInstance();
    lw->AddActuator("BallPickUpSubsystem", "Ball Pick Up Motor", m_motor);
    frc::DriverStation::ReportError("BallPickUp RobotInit done\n");
}

void BallPickUpSubsystem::AutonomousInit(void)
{
	m_timer->Stop();
	m_timer->Reset();
}
void BallPickUpSubsystem::AutonomousPeriodic(void)
{
	TeleopPeriodic();
}
void BallPickUpSubsystem::DisabledInit(void)
{
	m_speed = 0.;
}
void BallPickUpSubsystem::TestInit(void)
{
}
void BallPickUpSubsystem::DisabledPeriodic(void)
{
	frc::SmartDashboard::PutNumber("Pick Up state", m_state);
	frc::SmartDashboard::PutBoolean("Pick Up Sensor?", m_pickupSensor->Get());
	m_motor->Set(0.);
}
void BallPickUpSubsystem::TestPeriodic(void)
{
}
void BallPickUpSubsystem::TeleopInit(void)
{
}
void BallPickUpSubsystem::TeleopPeriodic(void)
{
	switch (m_state) {
	case kOutToSensor:
            if (m_timer->Get() > m_timeout) {
                m_state = kNormal;
            }
            if (m_pickupSensor->Get()) {
                m_state = kOutForKicker;
            }
            break;
	case kOutForKicker:            
            if (!m_pickupSensor->Get()) {
                m_state = kNormal;
            }
            if (m_timer->Get() > m_timeout) {
                m_state = kNormal;
            }
            break;
	case kOutForShooter:
            if (m_timer->Get() > m_timeout) {
                m_state = kNormal;
            }
            break;
	case kInToHold:
        if (!m_ballSeen && m_pickupSensor->Get()) {
            m_ballSeen = true;
        }
        if (m_ballSeen && !m_pickupSensor->Get()) {
            m_speed = 0.0;
            m_state = kHolding;
        }
		break;
	case kNormal:
	case kHolding:
            break;
	}
	m_motor->Set(m_speed);
	SmartDashboard::PutNumber("Pick Up state", m_state);
	SmartDashboard::PutBoolean("Pick Up Sensor?", m_pickupSensor->Get());
}

void BallPickUpSubsystem::Abort()
{
	m_state = kNormal;
}

bool BallPickUpSubsystem::Done()
{
	if ((m_state == kNormal) || (m_state == kHolding))
		return true;
	return false;
}
BallPickUpSubsystem::~BallPickUpSubsystem() {
}

