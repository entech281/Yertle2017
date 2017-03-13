#ifndef ROBOT_CONSTANTS
#define ROBOT_CONSTANTS

#include "WPILib.h"

// Camera definitions
#define USB_CAMERA  0
#define AXIS_CAMERA 0
#define CAMERA_SERVO 0

// Gyro definitions  -- Code tries to allow GYRO_NAVX to always work (or atleast not crash)
#define GYRO_NAVX   1

// Joystick Ports
const uint32_t c_joystickLeftPort					= 0;
const uint32_t c_joystickRightPort					= 1;

// Gamepad Ports
#define GAMEPAD_ACTIVE 1
const uint32_t c_gamepadJoystickPort				= 2;
const uint32_t c_prongsOutPortgp					= 13;
const uint32_t c_pickUpPortgp						= 5;
const uint32_t c_releasePortgp						= 7;
const uint32_t c_shooterFastPortgp					= 1;
const uint32_t c_shooterSlowPortgp					= 2;
const uint32_t c_kickerFastPortgp					= 3;
const uint32_t c_kickerSlowPortgp					= 4;
const uint32_t c_gyroResetPortgp					= 9;

// Operator Panel Buttons
#define PANEL_ACTIVE 0
const uint32_t c_arduinoJoystickPort				= 3;
const uint32_t c_prongsOutButtonp					= 1;
const uint32_t c_pickUpButtonp						= 2;
const uint32_t c_releaseButtonp						= 3;
const uint32_t c_kickerOverrideButtonp				= 4;
const uint32_t c_shooterOverrideButtonp				= 5;
const uint32_t c_shooterButtonp						= 6;
const uint32_t c_kickerButtonp						= 7;
const uint32_t c_highGoalButtonp					= 8;
const uint32_t c_lowGoalButtonp						= 9;
const uint32_t c_ESTOPButtonp						= 10;

// Drive motor inversion states
const bool c_kflmotor_inversed = true;
const bool c_krlmotor_inversed = true;
const bool c_kfrmotor_inversed = true;
const bool c_krrmotor_inversed = true;

// CAN Bus IDs
const uint32_t c_flmotor_CANid	 					= 0;
const uint32_t c_frmotor_CANid	 					= 2;
const uint32_t c_rlmotor_CANid	 					= 1;
const uint32_t c_rrmotor_CANid						= 3;
const uint32_t c_BallPickUpmotor_CANid				= 4;
const uint32_t c_shooterMotor_CANid					= 5;
const uint32_t c_compressorPCMid					= 10;

// Pneumatic Solenoid  ports
const uint32_t c_prongsSolenoidChannel1				= 0;
const uint32_t c_prongsSolenoidChannel2				= 1;

// Digital Inputs/Outputs
const uint32_t c_pickupSensorPort		 = 1;
const uint32_t c_shooterSensorPort		 = 0;
const uint32_t c_prongsUpSensorChannel   = 2;
const uint32_t c_prongsDownSensorChannel = 3;

// PWM Ports
const uint32_t c_cameraServoUD						= 0;
// const uint32_t c_cameraServoLR					= 0;

#endif

