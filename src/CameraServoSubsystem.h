#ifndef _CAMERASERVO_SUBSYSTEM_H
#define _CAMERASERVO_SUBSYSTEM_H

#include <WPILib.h>

#include "RobotSubsystem.h"

class CameraServoSubsystem : public RobotSubsystem {
private:
	frc::Servo *m_tiltServo;

public:
    CameraServoSubsystem(EntechRobot *pRobot, std::string name = "camera");
    virtual ~CameraServoSubsystem();

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

    void SetTilt(double value);
};
#endif
