#ifndef _DRIVE_SUBSYSTEM_H
#define _DRIVE_SUBSYSTEM_H

#include <WPILib.h>
#include <CANTalon.h>

#include "RobotConstants.h"
#if GYRO_NAVX
#include <AHRS.h>
#endif
#include <PIDSource.h>
#include <PIDOutput.h>
#include <PIDController.h>

#include "RobotSubsystem.h"
#include "OperatorButton.h"


class DriveSubsystem : public RobotSubsystem, frc::PIDSource, frc::PIDOutput {
private:
    frc::Joystick* m_joystickLeft;
    frc::Joystick* m_joystickRight;
    CANTalon* m_flmotor;
    CANTalon* m_frmotor;
    CANTalon* m_rlmotor;
    CANTalon* m_rrmotor;
#if GYRO_NAVX
    AHRS    *m_ahrs;
    frc::PIDController *m_straight_controller;
    frc::PIDController *m_large_turn_controller;
    frc::PIDController *m_small_turn_controller;
#endif
    
    bool m_autoDrive;
    bool m_tankMode;
    OperatorButton *m_driveModeToggle;
    OperatorButton *m_twistTrigger;
    double m_maxSpeed;
    double m_lastMaxSpeed;
    double m_autoDriveTime;
    double m_autoDriveSpeed;
    double m_targetLeftDistance;
    double m_targetRightDistance;
    double m_targetHeading;
    double m_leftRightCorrection;
    frc::Timer* m_timer;
    enum DriveState { kNone=0, kDrivingDistance, kDrivingStraight, kDrivingTime, kTurningTo };
    DriveState m_state;
    RobotDrive* m_robotDrive;

    double GetLeftEncoders(void);
    double GetRightEncoders(void);
    double GetLeftPosition(void);
    double GetRightPosition(void);
    double GetMaxSpeed(void);

    const double c_transitionTime = 4.0;
    const int c_encoderSignal = 200;
    const int c_encoderNoSignal = 10;

    const float c_pid_straight_p = 0.018;   // 3% motor power change (each side) per degree heading
    const float c_pid_straight_i = 0.003;
    const float c_pid_straight_d = 0.015;
    const float c_pid_straight_max = 0.03;    // 5% each side is the max
    const float c_pid_straight_tolerance = 1.0;   // degrees

    const float c_pid_large_turn_p = 0.04;
    const float c_pid_large_turn_i = 0.005;
    const float c_pid_large_turn_d = 0.07;
    const float c_pid_large_turn_max = 0.6;

    const float c_pid_turn_tolerance = 1.0;   // degrees
    const float c_pid_small_large_transition = 7.5;  // when to transition between two controllers
    
    const float c_pid_small_turn_p = 0.1;
    const float c_pid_small_turn_i = 0.07;
    const float c_pid_small_turn_d = 0.03;
    const float c_pid_small_turn_max = 0.6;

    bool m_frontLeftEncoderBad;
    bool m_rearLeftEncoderBad;
    bool m_frontRightEncoderBad;
    bool m_rearRightEncoderBad;

public:
    DriveSubsystem(EntechRobot *pRobot, std::string name = "drive");
    virtual ~DriveSubsystem();

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

    /********************************** Entech Routines **********************************/
    void SetMaxSpeed(double max_speed);
    void DriveByDistance(double leftEncoderDist, double rightEncoderDist, double speed=0.5);
    void DriveBySeconds(double speed, double time);
    void DriveStraight(double distance, double speed=0.5);
    void DriveStraightHeading(double angle);
    void TurnToHeading(double angle);
    bool Done(void);
    void Abort(void);
    bool SubSystemOk();

    /* PID Interface */
    virtual double PIDGet();
    virtual void PIDWrite(double output);

    void SmartDashboardOutput();
};
#endif
