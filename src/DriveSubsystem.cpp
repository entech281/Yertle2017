#include <WPILib.h>

#include "DriveSubsystem.h"
#include "RobotConstants.h"

#include <math.h>
#include <algorithm>

DriveSubsystem::DriveSubsystem(EntechRobot *pRobot, std::string name)
	: RobotSubsystem(pRobot, name)
    , m_joystickLeft(NULL)
	, m_joystickRight(NULL)
    , m_flmotor(NULL)
    , m_frmotor(NULL)
    , m_rlmotor(NULL)
    , m_rrmotor(NULL)
#if GYRO_NAVX
	, m_ahrs(NULL)
    , m_straight_controller(NULL)
    , m_large_turn_controller(NULL)
    , m_small_turn_controller(NULL)
#endif
    , m_autoDrive(false)
	, m_tankMode(false)
	, m_driveModeToggle(NULL)
	, m_twistTrigger(NULL)
	, m_maxSpeed(1.0)
    , m_lastMaxSpeed(1.0)
	, m_autoDriveTime(0.0)
	, m_autoDriveSpeed(0.0)
	, m_targetLeftDistance(0.0)
	, m_targetRightDistance(0.0)
    , m_targetHeading(0.0)
    , m_leftRightCorrection(0.0)
	, m_timer(NULL)
	, m_state(kNone)
	, m_robotDrive(NULL)
	, m_frontLeftEncoderBad(false)
	, m_rearLeftEncoderBad(false)
	, m_frontRightEncoderBad(false)
	, m_rearRightEncoderBad(false)
{
}

DriveSubsystem::~DriveSubsystem() {}

/********************************** Init Routines **********************************/

void DriveSubsystem::RobotInit()
{
    LiveWindow* lw;

    m_joystickLeft = new Joystick(c_joystickLeftPort);
    m_joystickRight = new Joystick(c_joystickRightPort);
    m_tankMode = false;
    m_driveModeToggle = new OperatorButton(m_joystickLeft, 8);
    m_twistTrigger = new OperatorButton(m_joystickLeft, 1);

    m_flmotor = new CANTalon(c_flmotor_CANid);
    m_frmotor = new CANTalon(c_frmotor_CANid);
    m_rlmotor = new CANTalon(c_rlmotor_CANid);
    m_rrmotor = new CANTalon(c_rrmotor_CANid);

    m_flmotor->SetControlMode(CANSpeedController::kPercentVbus);
    m_frmotor->SetControlMode(CANSpeedController::kPercentVbus);
    m_rlmotor->SetControlMode(CANSpeedController::kPercentVbus);
    m_rrmotor->SetControlMode(CANSpeedController::kPercentVbus);

    m_flmotor->ConfigEncoderCodesPerRev(20);
    m_frmotor->ConfigEncoderCodesPerRev(20);
    m_rlmotor->ConfigEncoderCodesPerRev(20);
    m_rrmotor->ConfigEncoderCodesPerRev(20);

#if GYRO_NAVX
    try {
    	m_ahrs = new AHRS(SPI::kMXP);
        DriverStation::ReportError("NavX FOUND");
        m_ahrs->Reset();
        if (m_ahrs->IsCalibrating()) {
            Wait(0.25);
        }
        m_ahrs->ZeroYaw();
    } catch (std::exception& ex) {
        m_ahrs = NULL;
    }
    m_straight_controller = new frc::PIDController(c_pid_straight_p, c_pid_straight_i, c_pid_straight_d, this, this);
    m_straight_controller->Disable();
    m_straight_controller->SetOutputRange(-c_pid_straight_max, c_pid_straight_max);
    m_straight_controller->SetSetpoint(0.0);
    m_straight_controller->SetAbsoluteTolerance(c_pid_straight_tolerance);
    m_straight_controller->SetContinuous(false);
    
    m_large_turn_controller = new frc::PIDController(c_pid_large_turn_p, c_pid_large_turn_i, c_pid_large_turn_d, this, this);
    m_large_turn_controller->Disable();
    m_large_turn_controller->SetSetpoint(0.0);
    m_large_turn_controller->SetOutputRange(-c_pid_large_turn_max, c_pid_large_turn_max);
    m_large_turn_controller->SetAbsoluteTolerance(c_pid_turn_tolerance);
    m_large_turn_controller->SetContinuous(true);
    m_large_turn_controller->SetInputRange(0.0, 360.0);
    
    m_small_turn_controller = new PIDController(c_pid_small_turn_p, c_pid_small_turn_i, c_pid_small_turn_d, this, this);
    m_small_turn_controller->Disable();
    m_small_turn_controller->SetSetpoint(0.0);
    m_small_turn_controller->SetOutputRange(-c_pid_small_turn_max, c_pid_small_turn_max);
    m_small_turn_controller->SetAbsoluteTolerance(c_pid_turn_tolerance);
    m_small_turn_controller->SetContinuous(true);
    m_small_turn_controller->SetInputRange(0.0, 360.0);
#endif

    m_timer = new Timer();

    m_robotDrive = new RobotDrive(m_flmotor, m_rlmotor, m_frmotor, m_rrmotor);
    m_robotDrive->SetSafetyEnabled(false);

    lw = LiveWindow::GetInstance();
    lw->AddActuator("DriveSubsystem", "Front Left Motor", m_flmotor);
    lw->AddActuator("DriveSubsystem", "Front Right Motor", m_frmotor);
    lw->AddActuator("DriveSubsystem", "Rear Left Motor", m_rlmotor);
    lw->AddActuator("DriveSubsystem", "Rear Right Motor", m_rrmotor);

    m_robotDrive->SetInvertedMotor(RobotDrive::kFrontLeftMotor, c_kflmotor_inversed);
    m_robotDrive->SetInvertedMotor(RobotDrive::kRearLeftMotor, c_krlmotor_inversed);
    m_robotDrive->SetInvertedMotor(RobotDrive::kFrontRightMotor, c_kfrmotor_inversed);
    m_robotDrive->SetInvertedMotor(RobotDrive::kRearRightMotor, c_krrmotor_inversed);
}

void DriveSubsystem::DisabledInit()
{
#if GYRO_NAVX
    m_straight_controller->Disable();
    m_large_turn_controller->Disable();
    m_small_turn_controller->Disable();
#endif
}

void DriveSubsystem::TeleopInit()
{
#if GYRO_NAVX
    m_straight_controller->Disable();
    m_large_turn_controller->Disable();
    m_small_turn_controller->Disable();
#endif
    m_state = kNone;
}

void DriveSubsystem::AutonomousInit()
{
#if GYRO_NAVX
    m_ahrs->ResetDisplacement();
    m_ahrs->ZeroYaw();

    m_straight_controller->Disable();
    m_large_turn_controller->Disable();
    m_small_turn_controller->Disable();

    m_straight_controller->Reset();
    m_large_turn_controller->Reset();
    m_small_turn_controller->Reset();
#endif
}

void DriveSubsystem::TestInit() {}

/********************************** Periodic Routines **********************************/

void DriveSubsystem::DisabledPeriodic()
{
    m_robotDrive->TankDrive(0.0, 0.0);
    SmartDashboardOutput();
}

void DriveSubsystem::TeleopPeriodic()
{
    double right_speed, left_speed;

    switch (m_state) {
    case kNone:
        if (m_driveModeToggle->Get() == OperatorButton::kJustPressed) {
            if (m_tankMode) {
                m_tankMode = false;
            }else {
                m_tankMode = true;
            }
        }
        if (m_tankMode){
            m_robotDrive->TankDrive(GetMaxSpeed()*m_joystickLeft->GetY(), GetMaxSpeed()*m_joystickRight->GetY());
            SmartDashboard::PutString("Drive Mode","Tank");
        } else {
        	if (m_twistTrigger->Get() == OperatorButton::kPressed){
                    m_robotDrive->ArcadeDrive(GetMaxSpeed()*m_joystickLeft->GetY(), GetMaxSpeed()*m_joystickLeft->GetZ());
        		// m_robotDrive->ArcadeDrive(m_joystickLeft);
        		SmartDashboard::PutString("Drive Mode", "Arcade Twist");
        	} else {
                    m_robotDrive->ArcadeDrive(GetMaxSpeed()*m_joystickLeft->GetY(),GetMaxSpeed()*m_joystickLeft->GetX());
                SmartDashboard::PutString("Drive Mode", "Arcade");
        	}
                SmartDashboard::PutNumber("Drive MaxSpeed", GetMaxSpeed());
        }
        break;
    case kDrivingDistance:
        if (m_timer->Get() > 0.2) {
            if (fabs(GetLeftEncoders()) < 0.85*fabs(m_targetLeftDistance)) {
                left_speed = m_autoDriveSpeed;
            } else if (fabs(GetLeftEncoders()) < fabs(m_targetLeftDistance)) {
                left_speed = 0.75*m_autoDriveSpeed;
            } else {
                left_speed = 0.0;
            }
            if (m_targetLeftDistance < 0.0){
                left_speed = -left_speed;
            }
            if (fabs(GetRightEncoders()) < 0.85*fabs(m_targetRightDistance)) {
                right_speed = m_autoDriveSpeed;
            } else if (fabs(GetRightEncoders()) < fabs(m_targetRightDistance)) {
                right_speed = 0.75*m_autoDriveSpeed;
            } else {
                right_speed = 0.0;
            }
            if (m_targetRightDistance < 0.0){
                right_speed = -right_speed;
            }

            left_speed = std::min(std::max(-1.0,left_speed),1.0);
            right_speed = std::min(std::max(-1.0,right_speed),1.0);
            if ((fabs(left_speed) > 0.05) || fabs(right_speed) > 0.05) {
                m_robotDrive->TankDrive(left_speed, right_speed);
            } else {
                m_robotDrive->TankDrive(0.0, 0.0);
                m_state = kNone;
            }
        }
        break;
    case kDrivingStraight:
        if (m_timer->Get() > 0.2) {
            if (fabs(GetLeftPosition()) < 0.85*fabs(m_targetLeftDistance)) {
                left_speed = m_autoDriveSpeed + m_leftRightCorrection;
            } else if (fabs(GetLeftPosition()) < fabs(m_targetLeftDistance)) {
                left_speed = 0.75*(m_autoDriveSpeed + m_leftRightCorrection);
            } else {
                left_speed = 0.0;
            }
            if (m_targetLeftDistance < 0.0){
                left_speed = -left_speed;
            }
            if (fabs(GetRightPosition()) < 0.85*fabs(m_targetRightDistance)) {
                right_speed = m_autoDriveSpeed - m_leftRightCorrection;
            } else if (fabs(GetRightPosition()) < fabs(m_targetRightDistance)) {
                right_speed = 0.75*(m_autoDriveSpeed - m_leftRightCorrection);
            } else {
                right_speed = 0.0;
            }
            if (m_targetRightDistance < 0.0){
                right_speed = -right_speed;
            }

            left_speed = std::min(std::max(-1.0,left_speed),1.0);
            right_speed = std::min(std::max(-1.0,right_speed),1.0);
            if ((fabs(left_speed) > c_pid_straight_max) || fabs(right_speed) > c_pid_straight_max) {
                m_robotDrive->TankDrive(left_speed, right_speed);
            } else {
                m_robotDrive->TankDrive(0.0, 0.0);
                m_state = kNone;
                m_straight_controller->Disable();
            }
        }
        break;
    case kTurningTo:
        m_robotDrive->TankDrive(-m_leftRightCorrection, m_leftRightCorrection);
        if ((m_timer->Get() > 0.2) &&
            ((m_large_turn_controller->IsEnabled() && m_large_turn_controller->OnTarget()) ||
             (m_small_turn_controller->IsEnabled() && m_small_turn_controller->OnTarget())    ) ) {
            m_robotDrive->TankDrive(0.0, 0.0);
            m_state = kNone;
            m_small_turn_controller->Disable();
            m_large_turn_controller->Disable();
        }
        break;
    case kDrivingTime:
        if (m_timer->Get() < m_autoDriveTime) {
            m_robotDrive->TankDrive(m_autoDriveSpeed, m_autoDriveSpeed);
        }
        else {
            m_robotDrive->TankDrive(0.0, 0.0);
            m_state = kNone;
        }
        break;
    }

    SmartDashboardOutput();
}

void DriveSubsystem::AutonomousPeriodic()
{
	TeleopPeriodic();
}

void DriveSubsystem::TestPeriodic()
{
}

void DriveSubsystem::SmartDashboardOutput()
{
    SmartDashboard::PutNumber("Drive Subsystem State", m_state);

    SmartDashboard::PutNumber("Front Left Motor Position", m_flmotor->GetPosition());
    SmartDashboard::PutNumber("Front Right Motor Position", m_frmotor->GetPosition());
    SmartDashboard::PutNumber("Rear Left Motor Position", m_rlmotor->GetPosition());
    SmartDashboard::PutNumber("Rear Right Motor Position", m_rrmotor->GetPosition());

    SmartDashboard::PutNumber("Front Left Motor GetEncPosition()", m_flmotor->GetEncPosition());
    SmartDashboard::PutNumber("Rear Left Motor GetEncPosition()", m_rlmotor->GetEncPosition());
    SmartDashboard::PutNumber("Front Right Motor GetEncPosition()", m_frmotor->GetEncPosition());
    SmartDashboard::PutNumber("Rear Right Motor GetEncPosition()", m_rrmotor->GetEncPosition());

#if GYRO_NAVX
    if (m_ahrs) {
        SmartDashboard::PutData("NavX", m_ahrs);
        SmartDashboard::PutString("NavX Exists", "YES");
        SmartDashboard::PutNumber("NavX GetAngle()", m_ahrs->GetAngle()); //total accumulated yaw angle, 360+
        SmartDashboard::PutNumber("NavX GetYaw()", m_ahrs->GetYaw()); //-180 to 180 degrees
    } else {
        SmartDashboard::PutString("NavX Exists", "NO");
    }
    SmartDashboard::PutBoolean("Drive Done()", Done());
#endif
}
/********************************** Entech Routines **********************************/

bool DriveSubsystem::Done(void)
{
    if (m_state == kNone)
        return true;
    return false;
}
void DriveSubsystem::Abort(void)
{
    m_state = kNone;
}

void DriveSubsystem::DriveStraight(double distance, double speed)
{
    m_straight_controller->Disable();
    m_large_turn_controller->Disable();
    m_small_turn_controller->Disable();

    m_state = kDrivingStraight;
    m_targetLeftDistance = distance;
    m_targetRightDistance = distance;
    m_targetHeading = m_ahrs->GetYaw();
    if (fabs(m_targetHeading) > 90.0) {
        m_targetHeading = m_ahrs->GetAngle();
    }
    m_autoDriveSpeed = speed;
    m_flmotor->SetPosition(0.0);
    m_frmotor->SetPosition(0.0);
    m_rlmotor->SetPosition(0.0);
    m_rrmotor->SetPosition(0.0);
    m_flmotor->SetEncPosition(0.0);
    m_frmotor->SetEncPosition(0.0);
    m_rlmotor->SetEncPosition(0.0);
    m_rrmotor->SetEncPosition(0.0);

    m_leftRightCorrection = 0.0;
    m_straight_controller->Enable();
    m_timer->Stop();
    m_timer->Reset();
    m_timer->Start();
}

void DriveSubsystem::DriveStraightHeading(double angle)
{
    m_targetHeading = angle;
}

void DriveSubsystem::TurnToHeading(double angle)
{
    double delta;

    // Turn off all controllers
    m_straight_controller->Disable();
    m_large_turn_controller->Disable();
    m_small_turn_controller->Disable();

    delta = fabs(angle - m_ahrs->GetAngle());
    if (delta < c_pid_turn_tolerance) {
        m_state = kNone;
    } else {
        m_state = kTurningTo;
        m_targetHeading = angle;
        m_large_turn_controller->SetSetpoint(angle);
        m_small_turn_controller->SetSetpoint(angle);
        m_targetLeftDistance = 0.;
        m_targetRightDistance = 0.;
        m_leftRightCorrection = 0.0;

        if (delta < c_pid_small_large_transition) {
            m_small_turn_controller->Enable();
        } else {
            m_large_turn_controller->Enable();
        }
    }
    m_timer->Stop();
    m_timer->Reset();
    m_timer->Start();
}

void DriveSubsystem::DriveByDistance(double leftMotorRevs, double rightMotorRevs, double speed)
{
    m_state = kDrivingDistance;
    m_targetLeftDistance = leftMotorRevs;
    m_targetRightDistance = rightMotorRevs;
    m_autoDriveSpeed = speed;
    m_flmotor->SetEncPosition(0.0);
    m_frmotor->SetEncPosition(0.0);
    m_rlmotor->SetEncPosition(0.0);
    m_rrmotor->SetEncPosition(0.0);
    m_timer->Stop();
    m_timer->Reset();
    m_timer->Start();
}

void DriveSubsystem::DriveBySeconds(double speed, double time)
{
    m_state = kDrivingTime;
    m_autoDriveSpeed = speed;
    m_autoDriveTime = time;
    m_timer->Stop();
    m_timer->Reset();
    m_timer->Start();
}

void DriveSubsystem::SetMaxSpeed(double max_speed)
{
    m_lastMaxSpeed = m_maxSpeed;
    m_maxSpeed = std::min(std::max(-1.0,max_speed),1.0);
    m_timer->Stop();
    m_timer->Reset();
    m_timer->Start();
}

double DriveSubsystem::GetLeftEncoders(void)
{
    double front = m_flmotor->GetEncPosition();
    double rear  = m_rlmotor->GetEncPosition();
    if ((fabs(front) > c_encoderSignal) && (fabs(rear) < c_encoderNoSignal)) {
    	m_rearLeftEncoderBad = true;
        return front;
    }
    if ((fabs(rear) > c_encoderSignal) && (fabs(front) < c_encoderNoSignal)) {
    	m_frontLeftEncoderBad = true;
        return rear;
    }
    if ((m_timer->Get() > 0.5) && (fabs(front) < c_encoderNoSignal) && (fabs(rear) < c_encoderNoSignal)) {
    	m_rearLeftEncoderBad = true;
    	m_frontLeftEncoderBad = true;
    	return 0.0;
    }
    return 0.5*(front + rear);
}

double DriveSubsystem::GetLeftPosition(void)
{
    double front = m_flmotor->GetEncPosition();
    double rear  = m_rlmotor->GetEncPosition();
    if ((fabs(front) > c_encoderSignal) && (fabs(rear) < c_encoderNoSignal)) {
    	m_rearLeftEncoderBad = true;
        return m_flmotor->GetPosition();
    }
    if ((fabs(rear) > c_encoderSignal) && (fabs(front) < c_encoderNoSignal)) {
    	m_frontLeftEncoderBad = true;
        return m_rlmotor->GetPosition();
    }
    if ((m_timer->Get() > 0.5) && (fabs(front) < c_encoderNoSignal) && (fabs(rear) < c_encoderNoSignal)) {
    	m_rearLeftEncoderBad = true;
    	m_frontLeftEncoderBad = true;
    	return 0.0;
    }
    return 0.5*(m_flmotor->GetPosition() + m_rlmotor->GetPosition());
}

double DriveSubsystem::GetRightEncoders(void)
{
    double front = m_frmotor->GetEncPosition();
    double rear  = m_rrmotor->GetEncPosition();
    if ((fabs(front) > c_encoderSignal) && (fabs(rear) < c_encoderNoSignal)) {
    	m_rearRightEncoderBad = true;
        return front;
    }
    if ((fabs(rear) > c_encoderSignal) && (fabs(front) < c_encoderNoSignal)) {
    	m_frontRightEncoderBad = true;
        return rear;
    }
    if ((m_timer->Get() > 0.5) && (fabs(front) < c_encoderNoSignal) && (fabs(rear) < c_encoderNoSignal)) {
    	m_rearRightEncoderBad = true;
    	m_frontRightEncoderBad = true;
    	return 0.0;
    }
    return 0.5*(front + rear);
}

double DriveSubsystem::GetRightPosition(void)
{
    double front = m_frmotor->GetEncPosition();
    double rear  = m_rrmotor->GetEncPosition();
    if ((fabs(front) > c_encoderSignal) && (fabs(rear) < c_encoderNoSignal)) {
    	m_rearRightEncoderBad = true;
        return m_frmotor->GetPosition();
    }
    if ((fabs(rear) > c_encoderSignal) && (fabs(front) < c_encoderNoSignal)) {
    	m_frontRightEncoderBad = true;
        return m_rrmotor->GetPosition();
    }
    if ((m_timer->Get() > 0.5) && (fabs(front) < c_encoderNoSignal) && (fabs(rear) < c_encoderNoSignal)) {
    	m_rearRightEncoderBad = true;
    	m_frontRightEncoderBad = true;
    	return 0.0;
    }
    return 0.5*(m_frmotor->GetPosition() + m_rrmotor->GetPosition());
}

double DriveSubsystem::GetMaxSpeed()
{
    if (m_maxSpeed < m_lastMaxSpeed)
        return m_maxSpeed;
    
    double t = m_timer->Get();
    if (t < c_transitionTime) {
        return (m_maxSpeed-m_lastMaxSpeed)*(t/c_transitionTime) + m_lastMaxSpeed;
    }
    return m_maxSpeed;
}

bool DriveSubsystem::SubSystemOk()
{
	if(m_frontRightEncoderBad && m_rearRightEncoderBad){
		return false;
	}
	if (m_frontLeftEncoderBad && m_rearLeftEncoderBad){
		return false;
	}
	return true;
}

double DriveSubsystem::PIDGet()
{
#if GYRO_NAVX
    if (m_state == kDrivingStraight) {
        if (fabs(m_targetHeading) < 90.0) {
            return m_ahrs->GetYaw() - m_targetHeading;
        } else {
            return m_ahrs->GetAngle() - m_targetHeading;
        }
        // return  m_targetHeading - m_ahrs->GetYaw();
    } else if (m_state == kTurningTo) {
        return m_ahrs->GetAngle();
    }
    return 0.;
#else
    return 0.;
#endif
}

void DriveSubsystem::PIDWrite(double output)
{
	if ((m_targetRightDistance > 0.) && (m_targetLeftDistance > 0.)) {
		output = -output;
	}
	if (m_state == kDrivingStraight) {
		if (output > c_pid_straight_max) {
			output = c_pid_straight_max;
		}
		if (output < -c_pid_straight_max) {
			output = -c_pid_straight_max;
		}
	}
    m_leftRightCorrection = output;
}
