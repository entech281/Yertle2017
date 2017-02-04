#ifndef ENTECH_ROBOT_H
#define ENTECH_ROBOT_H

// Comment added

#include <WPILib.h>
#include <list>

#include "RobotConstants.h"
#include "RobotSubsystem.h"
#include "OperatorButton.h"
#include "DriveSubsystem.h"
#include "ShooterSubsystem.h"
#include "ProngsSubsystem.h"
#include "CameraServoSubsystem.h"
#include "BallPickUpSubsystem.h"

class EntechRobot : public IterativeRobot
{
public:
	EntechRobot();
	virtual ~EntechRobot();
    virtual void RobotInit();
    virtual void DisabledInit();
    virtual void DisabledPeriodic();
    virtual void TeleopInit();
    virtual void TeleopPeriodic();
    virtual void AutonomousInit();
    virtual void AutonomousPeriodic();
    virtual void TestInit();
    virtual void TestPeriodic();
	
	void RegisterSubsystem(RobotSubsystem*);
	
private:
	DriveSubsystem* m_drive;
	ShooterSubsystem* m_shooter;
	ProngsSubsystem* m_prongs;
	CameraServoSubsystem* m_cameraServo;
    BallPickUpSubsystem *m_ballpickup;

    Joystick* m_driverJoystick;
#if GAMEPAD_ACTIVE
    Joystick* m_gamepad;
    OperatorButton* m_ballpickupForwardButton;
    OperatorButton* m_ballpickupBackwardButton;
    OperatorButton* m_prongsOutButton;
    OperatorButton *m_shooterFastButton;
    OperatorButton *m_shooterSlowButton;
    OperatorButton *m_kickerFastButton;
    OperatorButton *m_kickerSlowButton;
	OperatorButton* m_gyroResetButton;
#endif
#if PANEL_ACTIVE
    Joystick* m_arduinoPanel;
    OperatorButton* m_gateLifterButton; //prongs out
    OperatorButton* m_pickupButton;
    OperatorButton* m_releaseButton;
    OperatorButton *m_kickerOverrideButton;
    OperatorButton *m_shooterOverrideButton;
    OperatorButton *m_shooterButton;
    OperatorButton *m_kickerButton;
    OperatorButton *m_highGoalButton;
    OperatorButton *m_lowGoalButton;
	OperatorButton *m_ESTOPButton;
#endif
    LiveWindow* m_lw;
    Compressor *m_compressor;

    std::list<RobotSubsystem*> m_robotSubsystems;

    enum RollerShooterState { kRollerShooterStart, kRollerStart, kShooterFire, kRollerStop, kRollerShooterDone};
    RollerShooterState m_rollerShooterState;
    enum ScoringState { kWaiting, kKicking, kShooting };
    ScoringState m_scoring;
    Timer *m_rollerShooterTimer;
    Timer *m_autoTimer;

    // AUTONOMOUS
    OperatorButton *m_autoSelectionButton1;
    OperatorButton *m_autoSelectionButton2;
    int m_autoPicking;
    int m_autoRoutine;
    int m_autoPosition;
    int m_autoShoot;
    enum AutoState { kStart = 0, kDriveToGate, kWaitForDriveToGate, kProngsDown, kDriveUnderGate, kWaitForDriveUnderGate,
		kProngsUpandDrive, kWaitForProngsandDriveFinish, kDriveAfterGate, kWaitForDriveAfterGate, kDriveToSeeSaw,
		kWaitForDriveToSeeSaw, kLowerSeeSaw, kDriveOverSeeSaw, kWaitForDriveOverSeeSaw, kWaitForProngsUp,
		kDrive, kWaitForDrive, kWaitForProngsAndShooter, kWaitForProngs, kWaitForDriveOverDefense,
		kWaitForDelayedStartPeriodic, kWaitForHeadingCorrection1, kTurnRight, kWaitForTurn1, kWaitForDriveForward1,
		kWaitForTimer, kWaitForDriveForward2, kWaitForBallOut, kWaitForKick, kWaitForDriveForward3,
		kDone };
    AutoState m_autoState;
    bool m_delayedStartDone;

    void AutoRollerShooter();
    void AutoRollerKicker();

    void AutoLongDriveOnlyPeriodic();
    void AutoDriveToRampPeriodic();
	void GatePeriodic();
	void SeeSawPeriodic();
	void LowBarPeriodic();
	void DelayedStartPeriodic();
	bool GetIsDelayedStartPeriodicDone();

	void UpdateDashboard();
#if USB_CAMERA
	static void VisionThread();
#endif
};
#endif
