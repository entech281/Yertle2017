#ifndef _PRONGS_SUBSYSTEM_H
#define _PRONGS_SUBSYSTEM_H

#include <WPILib.h>
#include <Solenoid.h>

#include "RobotSubsystem.h"
#include "OperatorButton.h"

class ProngsSubsystem : public RobotSubsystem {
public:
    ProngsSubsystem(EntechRobot *pRobot, std::string name = "prongs");
    virtual ~ProngsSubsystem();
    enum ArmPosition {kUp, kDown};

    /********************************** Init Routines **********************************/

    virtual void RobotInit();
    virtual void DisabledInit();
    virtual void TeleopInit();
    virtual void AutonomousInit();
    virtual void TestInit();

    /********************************** Periodic Routines **********************************/
    virtual void DisabledPeriodic();
    virtual void AutonomousPeriodic();
    virtual void TeleopPeriodic();
    virtual void TestPeriodic();

    void SetPosition(ArmPosition position);
    bool Done();

private:
    frc::Solenoid *m_prongsSolenoid1;
    frc::Solenoid *m_prongsSolenoid2;
    frc::DigitalInput *m_prongsUpSensor;
    frc::DigitalInput *m_prongsDownSensor;
    ArmPosition m_position;
    frc::Timer *m_timer;

    frc::LiveWindow* lw;
};

#endif
