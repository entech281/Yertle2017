#include <WPILib.h>

#include "RobotConstants.h"
#include "EntechRobot.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

EntechRobot::EntechRobot(void)
	: m_drive(NULL)
	, m_shooter(NULL)
	, m_prongs(NULL)
    , m_cameraServo(NULL)
    , m_ballpickup(NULL)
    , m_driverJoystick(NULL)
#if GAMEPAD_ACTIVE
	, m_gamepad(NULL)
	, m_ballpickupForwardButton(NULL)
	, m_ballpickupBackwardButton(NULL)
	, m_prongsOutButton(NULL)
	, m_shooterFastButton(NULL)
	, m_shooterSlowButton(NULL)
	, m_kickerFastButton(NULL)
	, m_kickerSlowButton(NULL)
	, m_gyroResetButton(NULL)
#endif
#if PANEL_ACTIVE
	, m_arduinoPanel(NULL)
	, m_gateLifterButton(NULL)
	, m_pickupButton(NULL)
	, m_releaseButton(NULL)
	, m_kickerOverrideButton(NULL)
	, m_shooterOverrideButton(NULL)
	, m_shooterButton(NULL)
	, m_kickerButton(NULL)
	, m_highGoalButton(NULL)
	, m_lowGoalButton(NULL)
	, m_ESTOPButton(NULL)
#endif
	, m_lw(NULL)
    , m_compressor(NULL)
	, m_rollerShooterState(kRollerShooterStart)
	, m_scoring(kWaiting)
	, m_rollerShooterTimer(NULL)
	, m_autoTimer(NULL)
	, m_autoSelectionButton1(NULL)
	, m_autoSelectionButton2(NULL)
	, m_autoPicking(0)
	, m_autoRoutine(0)
	, m_autoPosition(0)
        , m_autoShoot(0)
	, m_autoState(kStart)
	, m_delayedStartDone(false)
{
	m_robotSubsystems.clear();
}

EntechRobot::~EntechRobot() {};

void EntechRobot::RobotInit()
{
    m_lw = LiveWindow::GetInstance();
    m_drive = new DriveSubsystem(this, "drive");
    m_shooter = new ShooterSubsystem(this, "shooter");
    m_prongs = new ProngsSubsystem(this, "prongs");
    m_ballpickup = new BallPickUpSubsystem(this, "ballpickup");
    m_driverJoystick = Joystick::GetStickForPort(c_joystickLeftPort);

#if GAMEPAD_ACTIVE
    m_gamepad = Joystick::GetStickForPort(c_gamepadJoystickPort);
	m_ballpickupForwardButton = new OperatorButton(m_gamepad, 5);
	m_ballpickupBackwardButton = new OperatorButton(m_gamepad, 6);
	m_prongsOutButton = new OperatorButton(m_gamepad, c_prongsOutPortgp);
	m_shooterFastButton = new OperatorButton(m_gamepad, c_shooterFastPortgp);
	m_shooterSlowButton = new OperatorButton(m_gamepad, c_shooterSlowPortgp);
	m_kickerFastButton = new OperatorButton(m_gamepad, c_kickerFastPortgp);
	m_kickerSlowButton = new OperatorButton(m_gamepad, c_kickerSlowPortgp);
	m_gyroResetButton = new OperatorButton(m_gamepad, c_gyroResetPortgp);
#endif

#if PANEL_ACTIVE
	m_arduinoPanel = Joystick::GetStickForPort(c_arduinoJoystickPort);
	m_gateLifterButton = new OperatorButton(m_arduinoPanel, c_prongsOutButtonp);
	m_pickupButton = new OperatorButton(m_arduinoPanel, c_pickUpButtonp);
	m_releaseButton = new OperatorButton(m_arduinoPanel, c_releaseButtonp);
	m_kickerOverrideButton = new OperatorButton(m_arduinoPanel, c_kickerOverrideButtonp);
	m_shooterOverrideButton = new OperatorButton(m_arduinoPanel, c_shooterOverrideButtonp);
	m_shooterButton = new OperatorButton(m_arduinoPanel, c_shooterButtonp);
	m_kickerButton = new OperatorButton(m_arduinoPanel, c_kickerButtonp);
	m_highGoalButton = new OperatorButton(m_arduinoPanel, c_highGoalButtonp);
	m_lowGoalButton = new OperatorButton(m_arduinoPanel, c_lowGoalButtonp);
	m_ESTOPButton = new OperatorButton(m_arduinoPanel, c_ESTOPButtonp);
#endif


    m_compressor = new Compressor(c_compressorPCMid);
    if (m_compressor) {
    	m_compressor->Start();
    }

#if USB_CAMERA
    m_cameraServo = new CameraServoSubsystem(this,"cameraServo");
    std::thread t_visionThread(VisionThread);
    t_visionThread.detach();
#endif

    m_rollerShooterState = kRollerShooterStart;
    m_rollerShooterTimer = new Timer();
    m_rollerShooterTimer->Stop();
    m_autoTimer = new Timer();
    m_autoTimer->Stop();

    m_autoPicking = 0;
    m_autoRoutine = 0;
    m_autoPosition = 0;
    m_autoShoot = 0;
    m_autoSelectionButton1 = new OperatorButton(m_driverJoystick, 1);
    m_autoSelectionButton2 = new OperatorButton(m_driverJoystick, 2);

    m_delayedStartDone = false;

    /*
     * Iterate through each sub-system and run the
     * appropriate function for the current mode.
     * Descriptions for each mode can be found in
     * RobotSubsystem.h
     */
    for (std::list<RobotSubsystem*>::iterator it = m_robotSubsystems.begin();
         it != m_robotSubsystems.end(); ++it) {
        (*it)->RobotInit();
    }

    UpdateDashboard();
    DriverStation::ReportError("Robot Init DONE");
}

#if USB_CAMERA
void EntechRobot::VisionThread()
{
	cs::UsbCamera t_camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();
	t_camera.SetResolution(640,480);
	cs::CvSink t_cvSink = frc::CameraServer::GetInstance()->GetVideo();
	cs::CvSource t_outputStreamStd = frc::CameraServer::GetInstance()->PutVideo("Gray", 640, 480);
	cv::Mat t_source;
	cv::Mat t_output;
	while (true) {
		t_cvSink.GrabFrame(t_source);
		cvtColor(t_source, t_output, cv::COLOR_BGR2GRAY);
		t_outputStreamStd.PutFrame(t_output);
	}
}
#endif

void EntechRobot::AutonomousInit()
{
    m_autoState = kStart;

    for (std::list<RobotSubsystem*>::iterator it = m_robotSubsystems.begin();
         it != m_robotSubsystems.end(); ++it) {
        (*it)->AutonomousInit();
    }

    UpdateDashboard();
}

void EntechRobot::TeleopInit()
{
	for (std::list<RobotSubsystem*>::iterator it = m_robotSubsystems.begin();
         it != m_robotSubsystems.end(); ++it) {
        (*it)->TeleopInit();
    }

	UpdateDashboard();
}

void EntechRobot::DisabledInit()
{
	for (std::list<RobotSubsystem*>::iterator it = m_robotSubsystems.begin();
         it != m_robotSubsystems.end(); ++it) {
        (*it)->DisabledInit();
    }

	UpdateDashboard();
	DriverStation::ReportError("DisabledInit() DONE");
}

void EntechRobot::TestInit()
{
    for (std::list<RobotSubsystem*>::iterator it = m_robotSubsystems.begin();
         it != m_robotSubsystems.end(); ++it) {
        (*it)->TestInit();
    }

    UpdateDashboard();
}

void EntechRobot::AutonomousPeriodic()
{
    if(!m_drive->SubSystemOk()){
        m_shooter->Abort();
        m_ballpickup->Abort();
        m_drive->Abort();
        m_autoState = kDone;
        DriverStation::ReportError("ABORTING AUTONOMOUS -- NO ENCODERS");
    }

    switch(m_autoRoutine){
    case 0:
        break;
    case 1:
        break;
    case 2:
        GatePeriodic();
        break;
    case 3:
        SeeSawPeriodic();
        break;
    case 4:
        AutoLongDriveOnlyPeriodic();
        break;
    case 5:
        LowBarPeriodic();
        break;
    default:
        break;
    }

    for (std::list<RobotSubsystem*>::iterator it = m_robotSubsystems.begin();
         it != m_robotSubsystems.end(); ++it) {
        (*it)->AutonomousPeriodic();
    }

    UpdateDashboard();
}

void EntechRobot::TeleopPeriodic()
{
#if GYRO_NAVX
#endif
    if (m_driverJoystick && m_cameraServo) {
        m_cameraServo->SetTilt(m_driverJoystick->GetThrottle());
    }
#if GAMEPAD_ACTIVE
	if (m_ballpickupForwardButton->Get() == OperatorButton::kPressed){
		m_ballpickup->Forward();
	} else if (m_ballpickupBackwardButton->Get() == OperatorButton::kPressed){
		m_ballpickup->Backward();
	} else {
		m_ballpickup->Off();
	}

	if (m_prongsOutButton->Get() == OperatorButton::kPressed) {
        m_prongs->SetPosition(ProngsSubsystem::kDown);
            m_drive->SetMaxSpeed(0.7);
	} else {
        m_prongs->SetPosition(ProngsSubsystem::kUp);
        m_drive->SetMaxSpeed(1.0);
	}

	if (m_shooterFastButton->Get() == OperatorButton::kPressed) {
		m_shooter->FastForward();
	} else if (m_shooterSlowButton->Get() == OperatorButton::kPressed){
		m_shooter->SlowForward();
	} else if (m_kickerFastButton->Get() == OperatorButton::kPressed){
	  	m_shooter->FastBackward();
	} else if (m_kickerSlowButton->Get() == OperatorButton::kPressed)
	{
		m_shooter->SlowBackward();
	} else {
	   	m_shooter->Off();
	}

//	if (m_gyroResetButton->Get() == OperatorButton::kJustPressed){
//            m_drive->ZeroYaw();
//    }
#endif

#if PANEL_ACTIVE
	if (m_ESTOPButton->GetBool()) {
		m_shooter->Abort();
		m_ballpickup->Abort();
		m_drive->Abort();
	}
	if (m_ballpickup->Done() && m_pickupButton->GetBool()){
		// m_shooter->MoveTo(0.0);
		// m_ballpickup->BallInToHold();
		m_ballpickup->Abort();
		m_ballpickup->Forward();
	} else if (m_ballpickup->Done() && m_releaseButton->GetBool()){
		m_ballpickup->Abort();
		m_ballpickup->Backward();
	} else if (m_ballpickup->Done()) {
		m_ballpickup->Abort();
		m_ballpickup->Off();
	}

	if (m_scoring == kWaiting) {
		if (m_gateLifterButton->GetBool()) {
			m_prongs->SetPosition(ProngsSubsystem::kDown);
			m_drive->SetMaxSpeed(0.7);
		} else {
			m_prongs->SetPosition(ProngsSubsystem::kUp);
			m_drive->SetMaxSpeed(1.0);
		}
	}

	if (m_scoring == kKicking){
			AutoRollerKicker();
	} else if (m_scoring == kShooting){
			AutoRollerShooter();
	} else if (m_highGoalButton->Get() == OperatorButton::kJustPressed){
		m_scoring = kShooting;
		m_rollerShooterState = kRollerShooterStart;
		AutoRollerShooter();
	} else if (m_lowGoalButton->Get() == OperatorButton::kJustPressed){
		m_scoring = kKicking;
		m_rollerShooterState = kRollerShooterStart;
		AutoRollerKicker();
	} else if (m_kickerButton->GetBool()) {
		if (m_kickerOverrideButton->GetBool()) {
			m_shooter->SlowBackward();
		} else {
			m_shooter->FastBackward();
		}
	} else if (m_shooterButton->GetBool()){
		if (m_shooterOverrideButton->GetBool()) {
			m_shooter->FastForward();
		} else {
			m_shooter->SlowForward();
		}
#if 0
	} else if (m_autoSelectionButton1 && (m_autoSelectionButton1->Get() == OperatorButton::kJustPressed)) {
	    	m_shooter->MoveTo(0.0);
	} else if (m_autoSelectionButton2 && (m_autoSelectionButton2->Get() == OperatorButton::kJustPressed)) {
	    	m_shooter->MoveTo(90.0);
#endif
	} else {
		m_shooter->Off();
	}
#endif

    for (std::list<RobotSubsystem*>::iterator it = m_robotSubsystems.begin();
         it != m_robotSubsystems.end(); ++it) {
        (*it)->TeleopPeriodic();
    }

	UpdateDashboard();
}

void EntechRobot::DisabledPeriodic()
{
    if (m_autoSelectionButton1 && m_autoSelectionButton2) {
        if (m_autoSelectionButton2->Get() == OperatorButton::kJustPressed) {
            m_autoPicking++;
            if (m_autoPicking > 2)
                m_autoPicking = 0;
        }
        if (m_autoSelectionButton1->Get() == OperatorButton::kJustPressed) {
            switch (m_autoPicking) {
            case 0:
                m_autoRoutine++;
                if (m_autoRoutine > 5) {
                    m_autoRoutine = 0;
                }
                break;
            case 1:
                m_autoPosition++;
                if (m_autoPosition > 4) {
                    m_autoPosition = 0;
                }
                break;
            case 2:
                m_autoShoot++;
                if (m_autoShoot > 2) {
                    m_autoShoot = 0;
                }
                break;
            }
        }
    } else {
        DriverStation::ReportError("Auto Pick -- NO BUTTON\n");
    }

    if (m_autoRoutine == 5) {
    	m_autoPosition = 0;
    } else if (m_autoPosition == 0) {
    	m_autoPosition = 1;
    }

    for (std::list<RobotSubsystem*>::iterator it = m_robotSubsystems.begin();
         it != m_robotSubsystems.end(); ++it) {
        (*it)->DisabledPeriodic();
    }

    UpdateDashboard();
}

void EntechRobot::TestPeriodic()
{
	 /* Update Live Window */
    if (m_lw)
	 m_lw->Run();

    for (std::list<RobotSubsystem*>::iterator it = m_robotSubsystems.begin();
         it != m_robotSubsystems.end(); ++it) {
        (*it)->TestPeriodic();
    }

    UpdateDashboard();
}

void EntechRobot::RegisterSubsystem(RobotSubsystem* subsys)
{
    m_robotSubsystems.push_back(subsys);
}

void EntechRobot::AutoRollerShooter()
{
	switch(m_rollerShooterState){
	case kRollerShooterStart:
		m_rollerShooterTimer->Stop();
		m_rollerShooterTimer->Reset();
		m_rollerShooterTimer->Start();
		m_prongs->SetPosition(ProngsSubsystem::kDown);
		m_shooter->MoveTo(60.0);
		m_ballpickup->BallOutForShooter();
		m_rollerShooterState = kShooterFire;
		break;
	case kShooterFire:
		if ((m_rollerShooterTimer->Get() > 1.5) ||
			(m_prongs->Done() && m_ballpickup->Done() && m_shooter->Done())) {
			m_shooter->Abort();
			m_shooter->Fire();
			m_rollerShooterState = kRollerStop;
		}
		break;
	case kRollerStop:
		if ((m_rollerShooterTimer->Get() > 2.0) || (m_shooter->Done())) {
			m_ballpickup->Off();
			m_rollerShooterState = kRollerShooterDone;
			m_scoring = kWaiting;
		}
		break;
	case kRollerShooterDone:
		break;
	default:
		break;
	}
}

void EntechRobot::AutoRollerKicker()
{
	switch(m_rollerShooterState){
	case kRollerShooterStart:
		m_rollerShooterTimer->Stop();
		m_rollerShooterTimer->Reset();
		m_rollerShooterTimer->Start();
		m_prongs->SetPosition(ProngsSubsystem::kUp);
		m_shooter->MoveTo(135.0);
		m_rollerShooterState = kRollerStart;
		break;
	case kRollerStart:
		if (m_rollerShooterTimer->Get() > 2.0) {
			m_shooter->Abort();
			m_shooter->Kick();
			m_rollerShooterState = kShooterFire;
		} else {
			m_ballpickup->Backward();
		}
		break;
	case kShooterFire:
		if (m_shooter->Done()){
			m_ballpickup->Off();
			m_shooter->MoveTo(0.0);
			m_scoring = kWaiting;
			m_rollerShooterState = kRollerShooterDone;
		}
		break;
	case kRollerShooterDone:
		break;
	default:
		break;
	}
}
/************************** AUTO ROUTINES ***************************/
void EntechRobot::GatePeriodic()
{
	switch(m_autoState){
	case kStart:
		m_autoState = kDriveToGate;
		break;
	case kDriveToGate:
		m_drive->DriveStraight(-34.0, 0.60); // need values!
		m_autoState = kWaitForDriveToGate;
		break;
	case kWaitForDriveToGate:
		if (m_drive->Done()){
			m_autoState = kProngsDown;
		}
		break;
	case kProngsDown:
		m_prongs->SetPosition(ProngsSubsystem::kDown);
		m_autoState = kDriveUnderGate;
		break;
	case kDriveUnderGate:
		if (m_prongs->Done()) {
			m_drive->DriveStraight(-4.5, 0.65); // need values!
			m_autoState = kWaitForDriveUnderGate;
		}
		break;
	case kWaitForDriveUnderGate:
		if (m_drive->Done()){
			m_prongs->SetPosition(ProngsSubsystem::kUp);
			m_drive->DriveStraight(-25.0, 0.55); // need values!
			m_autoState = kWaitForProngsandDriveFinish;
		}
		break;
	case kWaitForProngsandDriveFinish:
		if (m_prongs->Done()){
			m_drive->DriveStraight(-25.0, 0.60); // need values!
			m_autoState = kWaitForDriveAfterGate;
		}
		break;
	case kWaitForDriveAfterGate:
		if (m_drive->Done()){
			m_autoState = kDone;
		}
		break;
	case kDone:
		break;
	default:
		break;
	}
}
void EntechRobot::SeeSawPeriodic()
{
	switch(m_autoState){
	case kStart:
		m_autoState = kDriveToSeeSaw;
		break;
	case kDriveToSeeSaw:
		m_drive->DriveStraight(-33.0, 0.60); // need values!
		m_autoState = kWaitForDriveToSeeSaw;
		break;
	case kWaitForDriveToSeeSaw:
		if (m_drive->Done()){
			m_drive->TurnToHeading(0.);
			m_autoState = kWaitForHeadingCorrection1;
		}
		break;
	case kWaitForHeadingCorrection1:
		if (m_drive->Done()){
			m_autoState = kLowerSeeSaw;
		}
		break;
	case kLowerSeeSaw:
		m_prongs->SetPosition(ProngsSubsystem::kDown);
		m_autoState = kDriveOverSeeSaw;
		break;
	case kDriveOverSeeSaw:
		if (m_prongs->Done()) {
			// m_drive->DriveByDistance(-500.0, -500.0, 0.75); // need values!
			m_drive->DriveStraight(-36.0, 0.70); // need values!
			//m_prongs->SetPosition(ProngsSubsystem::kUp);
			m_autoState = kWaitForDriveOverSeeSaw;
		}
		break;
	case kWaitForDriveOverSeeSaw:
		if (m_drive->Done()){
			m_prongs->SetPosition(ProngsSubsystem::kUp);
			m_drive->DriveStraight(-50.0, 0.60); // need values!
			m_autoState = kWaitForProngsUp;
		}
		break;
	case kWaitForProngsUp:
		if (m_prongs->Done() && m_drive->Done()){
			m_drive->DriveStraight(10.0, 0.60); // need values!
			m_autoState = kWaitForKick;
		}
		break;
	case kWaitForDriveForward1:
		if (m_drive->Done()){
			m_drive->TurnToHeading(45.0); // need values!
			m_autoState = kWaitForTurn1;
		}
		break;
	case kWaitForTurn1:
		if (m_drive->Done()){
			m_drive->DriveByDistance(-875.0, -875.0, 0.75); // need values!
			m_autoState = kWaitForDriveForward2;
		}
		break;
	case kWaitForDriveForward2:
		if (m_drive->Done()){
			m_ballpickup->BallOutForKicker();
			m_autoState = kWaitForBallOut;
		}
		break;
	case kWaitForBallOut:
		if (m_ballpickup->Done()) {
			m_shooter->Kick();
			m_autoState = kWaitForKick;
		}
		break;
	case kWaitForKick:
		if (m_shooter->Done()) {
			m_ballpickup->Off();
			m_shooter->Off();
			m_autoState = kDone;
		}
		break;
	case kDone:
		break;
	default:
		break;
	}
}

void EntechRobot::AutoLongDriveOnlyPeriodic()
{
	switch(m_autoState){
	case kStart:
		m_autoState = kDrive;
		break;
	case kDrive:
		m_drive->DriveStraight(-150.0, 0.7);
		m_autoState = kWaitForDrive;
		break;
	case kWaitForDrive:
		if (m_drive->Done()){
			m_autoState = kDone;
		}
		break;
	case kDone:
		break;
	default:
		break;
	}
}
void EntechRobot::AutoDriveToRampPeriodic()
{
	switch(m_autoState){
	case kStart:
		m_autoState = kDrive;
		break;
	case kDrive:
		m_drive->DriveByDistance(-1400.0, -1400.0); // motor revs (x20 for encoder pulses)
		m_autoState = kWaitForDrive;
		break;
	case kWaitForDrive:
		if (m_drive->Done()){
			m_autoState = kDone;
		}
		break;
	case kDone:
		break;
	default:
		break;
	}
}
void EntechRobot::LowBarPeriodic()
{
	switch(m_autoState){
	case kStart:
		m_autoTimer->Stop();
		m_autoTimer->Reset();
		m_autoTimer->Start();
		m_drive->DriveStraight(144.0, 0.65);
		m_autoState = kWaitForProngsAndShooter;
		break;
	case kWaitForProngsAndShooter:
		if (m_autoTimer->Get() > 0.5) {
			m_prongs->SetPosition(ProngsSubsystem::kDown);
			m_shooter->MoveTo(90.);
			m_autoState = kWaitForDriveForward1;
		}
		break;
	case kWaitForDriveForward1:
		if (m_drive->Done()){
			// If not shooting, prongs up
			if (m_autoShoot != 2) {
				m_prongs->SetPosition(ProngsSubsystem::kUp);
			}
			m_drive->TurnToHeading(240.0);
			m_autoState = kWaitForTurn1;
		}
		break;
	case kWaitForTurn1:
		if (m_drive->Done() && m_prongs->Done()) {
            if (m_autoShoot == 0) {
                m_autoState = kDone;
            } else if (m_autoShoot == 1) {
    			m_shooter->MoveTo(135.);
			    m_drive->DriveStraight(-40.0, 0.6); // TODO: Low goal -- more than 30.
                m_drive->DriveStraightHeading(240.0);
			    m_autoState = kWaitForDriveForward2;
            } else if (m_autoShoot == 2) {
    			m_shooter->MoveTo(0.);
			    m_drive->DriveStraight(-13.0, 0.6); // TODO: high goal @20?
                m_drive->DriveStraightHeading(240.0);
			    m_autoState = kWaitForDriveForward2;
            }
		}
		break;
	case kWaitForDriveForward2:
		if (m_drive->Done()){
            if (m_autoShoot == 1) {
            	m_ballpickup->BallOutForKicker();
            } else if (m_autoShoot == 2) {
            	m_ballpickup->BallOutForShooter();
            }
        	m_autoState = kWaitForBallOut;
		}
		break;
	case kWaitForBallOut:
		if (m_ballpickup->Done()) {
			if (m_autoShoot == 1) {
				m_shooter->Kick();
				m_autoState = kWaitForKick;
			} else if (m_autoShoot == 2) {
				m_shooter->Fire();
				m_autoState = kWaitForKick;
			}
		}
		break;
	case kWaitForKick:
		if (m_shooter->Done()) {
			m_ballpickup->Off();
			m_shooter->Off();
			m_prongs->SetPosition(ProngsSubsystem::kUp);
			m_autoState = kDone;
		}
		break;
	case kDone:
		break;
	default:
		break;
	}
}
// TODO: test delayed start code. implementation?
void EntechRobot::DelayedStartPeriodic()
{
	switch(m_autoState){
	case kStart:
		m_autoState = kDrive;
		m_delayedStartDone = false;
		break;
	case kDrive:
		m_drive->DriveByDistance(-1400.0, -1400.0); // need values!
		m_autoState = kWaitForDrive;
		break;
	case kWaitForDrive:
		if (m_drive->Done()){
			m_autoState = kTurnRight;
		}
		break;
	case kTurnRight:
		m_drive->DriveByDistance(-700.0, -700.0);
		m_autoState = kWaitForTurn1;
		break;
	case kWaitForTurn1:
		if(m_drive->Done()){
			m_autoState = kDone;
		}
		break;
	case kDone:
		m_delayedStartDone = true;
		break;
	default:
		break;
	}
}

bool EntechRobot::GetIsDelayedStartPeriodicDone()
{
	return m_delayedStartDone;
}

void EntechRobot::UpdateDashboard()
{
  SmartDashboard::PutNumber("m_autoState", m_autoState);
  switch(m_autoPicking){
  case 0:
    SmartDashboard::PutString("Auto Picking", "Routine");
    break;
  case 1:
    SmartDashboard::PutString("Auto Picking", "Position");
    break;
  case 2:
    SmartDashboard::PutString("Auto Picking", "Shoot");
    break;
  }
  switch(m_autoRoutine){
  case 0:
    SmartDashboard::PutString("Auto Routine", "NONE (0)");
    break;
  case 1:
    SmartDashboard::PutString("Auto Routine", "DrawbridgePeriodic (1)");
    break;
  case 2:
    SmartDashboard::PutString("Auto Routine", "GatePeriodic (2)");
    break;
  case 3:
    SmartDashboard::PutString("Auto Routine", "SeeSawPeriodic (3)");
    break;
  case 4:
    SmartDashboard::PutString("Auto Routine", "Long Drive Only (4)");
    break;
  case 5:
    SmartDashboard::PutString("Auto Routine", "LowBarPeriodic (5)");
    break;
  default:
    SmartDashboard::PutString("Auto Routine", "No Obstacle! Periodic (6+)");
    break;
  }
  switch(m_autoPosition){
  case 0:
    SmartDashboard::PutString("Auto Position", "LowBar (0)");
    break;
  case 1:
    SmartDashboard::PutString("Auto Position", "Position (1)");
    break;
  case 2:
    SmartDashboard::PutString("Auto Position", "Position (2)");
    break;
  case 3:
    SmartDashboard::PutString("Auto Position", "Position (3)");
    break;
  case 4:
    SmartDashboard::PutString("Auto Position", "Position (4)");
    break;
  default:
    SmartDashboard::PutString("Auto Position", "Unknown");
    break;
  }
  switch(m_autoShoot){
  case 0:
    SmartDashboard::PutString("Auto Shoot", "NoShot (0)");
    break;
  case 1:
    SmartDashboard::PutString("Auto Shoot", "Low Goal (1)");
    break;
  case 2:
    SmartDashboard::PutString("Auto Shoot", "High Goal (2)");
    break;
  default:
    SmartDashboard::PutString("Auto Shoot", "Unknown");
    break;
  }
  for (std::list<RobotSubsystem*>::iterator it = m_robotSubsystems.begin();
       it != m_robotSubsystems.end(); ++it) {
    (*it)->UpdateDashboard();
  }
}

START_ROBOT_CLASS(EntechRobot);
