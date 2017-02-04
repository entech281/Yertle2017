#ifndef _SHOOTER_SUBSYSTEM_H
#define _SHOOTER_SUBSYSTEM_H

#include <WPILib.h>
#include <CANTalon.h>

#include "RobotSubsystem.h"
#include "OperatorButton.h"

class ShooterSubsystem : public RobotSubsystem, frc::PIDSource, frc::PIDOutput {
private:
    CANTalon* m_shooterMotor;
    double m_shooterSpeed;

    frc::DigitalInput* m_zeroSensor;

    enum ShooterState { kLost, kOff, kOn, kMovingTo, kShooting, kKicking, kFiring };
    ShooterState m_state;
    double m_currentAngle;
    double m_targetAngle;
    frc::Timer *m_timer;
    frc::PIDController *m_position_controller;

    void SetSpeed(double speed);

    // Test fix
    const double c_fastForward =  -1.0;
    const double c_slowForward =  -0.25;
    const double c_fastBackward =  1.0;
    const double c_slowBackward =  0.25;

    const float c_pid_position_p = 0.005;  // 0.005
    const float c_pid_position_i = 0.001;    // 0.005;
    const float c_pid_position_d = 0.005;    // 0.02;
    const float c_pid_position_max = 0.2;
    const double c_positionTolerance = 5.;
public:
    ShooterSubsystem(EntechRobot *pRobot, std::string name = "shooter");
    virtual ~ShooterSubsystem();

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

    void FastForward();
    void SlowForward();
    void FastBackward();
    void SlowBackward();
    void Kick();
    void Fire();
    void MoveTo(double angle);
    bool Done();
    void Off();
    void Abort(void);
    double PIDGet();
    void PIDWrite(double output);
};
#endif
