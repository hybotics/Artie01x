/*
	Program: 	W.A.L.T.E.R. 2.0, Navigation_03x.h - Navigation Control Program sketch header file
	Date:		28-Jun-2014
	Version:	0.2.0 ALPHA

	Platform:	Arduino Mega 2560 ADK,
					Lynxmotion's SSC-32 Servo Controller,
					and a 3DOF (Raise/Lower, Wrist Rotate, Open/Close) Little Grip gripper

	Purpose:	To experiment with various sensor configurations, tracking objects (heat or
					color based), course following, manipulation of the environment, vision, and
					general robotics techniques.

	Comments:	Credit is given, where applicable, for code I did not originate.

				Copyright (C) 2014 Dale Weber <hybotics.pdx@gmail.com>.
*/

#ifndef	__ROMEO_V1_1_MCP_H__
#define	__ROMEO_V1_1_MCP_H__

/************************************************************/
/*	General settings 										*/
/************************************************************/

#define	I2C_SLAVE_ADDRESS				0x50

#define	BUILD_VERSION					"0.2.0"
#define	BUILD_DATE 						"28-Jun-2014 ALPHA"
#define	BUILD_BOARD						"Romeo v1.1 All In One (Arduino Uno compatible)"

#define	LOOP_DELAY_SECONDS				10
#define STARTUP_DELAY_SECONDS			7

/*
	These settings control whether standard information is displayed
		on the seven segment and matrix displays or not, and how
		often, in minutes.
*/
#define	DISPLAY_INFORMATION				false

#define	DISPLAY_DATE_FREQ_MIN			15 				//	Minutes
#define	DISPLAY_TIME_FREQ_MIN			15 				//	Minutes
#define	DISPLAY_TEMPERATURE_FREQ_MIN	15 				//	Minutes

/*
	Optional Sensors and Peripherals enables (true)/disables (false)
*/
#define HAVE_COLOR_SENSOR				false
#define HAVE_HEAT_SENSOR				false
#define HAVE_DS1307_RTC					false

#define	HAVE_10DOF_IMU					false

//	NOTE: These three are all contained on the Adafruit 10DOF IMU board
#define	HAVE_LSM303DLHC_ACCEL			false
#define	HAVE_L3GD20_GYRO				false
#define	HAVE_BMP180_TEMP				false

#define HAVE_7SEGMENT_DISPLAYS			false

/*********************************************************
	Romeo v1.1 All In One Microcontroller (Arduino) Settings
*********************************************************/

/*
	Serial ports
*/

//	Hardware Serial0: Console and Debug port
#define	SERIAL_CONSOLE_RX_PIN			0
#define	SERIAL_CONSOLE_TX_PIN			1

/*
	Peripheral Settings, for Displays, Sound, etc.
*/

#define	WHEEL_ENCODER_LEFT_PIN			2
#define	WHEEL_ENCODER_RIGHT_PIN			3

//	Display constants
#define	MAX_NUMBER_7SEG_DISPLAYS		0
#define	SEVEN_SEG_BASE_ADDR				0x70

#define	MATRIX_DISPLAY_ADDR				SEVEN_SEG_BASE_ADDR + MAX_NUMBER_7SEG_DISPLAYS

/*
	Other Resources
*/
#define	HEARTBEAT_LED 					13
#define	COLOR_SENSOR_LED				12

/*
	Sensor settings
*/
#define	MAX_NUMBER_AREA_READINGS		18
#define AREA_SCAN_DEGREE_INCREMENT		20

//	Sharp GP2Y0A21YK0F IR sensors
#define	MAX_NUMBER_IR					1
#define	IR_PIN_BASE						0			//	Analog 0
#define IR_MIN_DISTANCE_CM				14.0 		//	In CM, which is approximately 5"

#define	IR_FRONT_CENTER					0
#define	IR_FRONT_LEFT					1
#define	IR_FRONT_RIGHT					2
#define	IR_BACK_RIGHT					3

/*********************************************************
	Sound generation constants
*********************************************************/

/*********************************************************
	Servo Settings
*********************************************************/

#define	SERVO_MAX_DEGREES				90
#define	SERVO_CENTER_MS					1500
#define	SERVO_MOVE_SPEED				4000		//	uS per second

//	Lesser = Right, Greater = Left
#define	SERVO_MAIN_PAN_PIN				9
#define SERVO_MAIN_PAN_NAME				"Main Pan"
#define	SERVO_MAIN_PAN_HOME				SERVO_CENTER_MS
#define	SERVO_MAIN_PAN_OFFSET			35
#define	SERVO_MAIN_PAN_RIGHT_MIN		500
#define	SERVO_MAIN_PAN_LEFT_MAX			2500

//	Greater = Down, Lesser = Up
#define	SERVO_MAIN_TILT_PIN				10
#define SERVO_MAIN_TILT_NAME			"Main Tilt"
#define	SERVO_MAIN_TILT_HOME			SERVO_CENTER_MS
#define	SERVO_MAIN_TILT_OFFSET			0
#define	SERVO_MAIN_TILT_UP_MIN			600
#define	SERVO_MAIN_TILT_DOWN_MAX		2500

//	Lesser = Right, Greater = Left
#define	SERVO_CAMERA_PAN_PIN			11
#define SERVO_CAMERA_PAN_NAME			"Camera Pan"
#define	SERVO_CAMERA_PAN_HOME			SERVO_CENTER_MS
#define	SERVO_CAMERA_PAN_OFFSET			0
#define	SERVO_CAMERA_PAN_RIGHT_MIN		600
#define	SERVO_CAMERA_PAN_LEFT_MAX		2500

//	Greater = Down, Lesser = Up
#define	SERVO_CAMERA_TILT_PIN			12
#define SERVO_CAMERA_TILT_NAME			"Camera Tilt"
#define	SERVO_CAMERA_TILT_HOME			SERVO_CENTER_MS
#define	SERVO_CAMERA_TILT_OFFSET		0
#define	SERVO_CAMERA_TILT_UP_MIN		600
#define	SERVO_CAMERA_TILT_DOWN_MAX		2500

//	Lesser = Up, Greater = Down
#define	SERVO_GRIP_LIFT_PIN				4
#define SERVO_GRIP_LIFT_NAME			"Grip Lift"
#define	SERVO_GRIP_LIFT_HOME			900
#define	SERVO_GRIP_LIFT_OFFSET			-100
#define	SERVO_GRIP_LIFT_MIN				900
#define	SERVO_GRIP_LIFT_MAX				1750

//	Lesser = , Greater = 
#define	SERVO_GRIP_ELBOW_PIN			6
#define SERVO_GRIP_ELBOW_NAME			"Grip Elbow"
#define	SERVO_GRIP_ELBOW_HOME			SERVO_CENTER_MS
#define	SERVO_GRIP_ELBOW_OFFSET			0
#define	SERVO_GRIP_ELBOW_MIN			500
#define	SERVO_GRIP_ELBOW_MAX			2500

//	Lesser = Right, Greater = Left
#define	SERVO_GRIP_WRIST_PIN			6
#define SERVO_GRIP_WRIST_NAME			"Grip Wrist"
#define	SERVO_GRIP_WRIST_HOME			SERVO_CENTER_MS
#define	SERVO_GRIP_WRIST_OFFSET			0
#define	SERVO_GRIP_WRIST_MIN			550
#define	SERVO_GRIP_WRIST_MAX			2500

//	Greater = Close, Lesser = Open
#define	SERVO_GRIP_GRAB_PIN				7
#define SERVO_GRIP_GRAB_NAME			"Grip Grab"
#define	SERVO_GRIP_GRAB_HOME			500
#define	SERVO_GRIP_GRAB_OFFSET			0
#define	SERVO_GRIP_GRAB_MIN				500
#define	SERVO_GRIP_GRAB_MAX				2000

/*
	Pin 8 has been intentionally left unconnected
*/

//	Lesser = , Greater = 
#define	SERVO_LEFT_TILT_PIN				9
#define SERVO_LEFT_TILT_NAME			"Left Tilt"
#define	SERVO_LEFT_TILT_HOME			SERVO_CENTER_MS
#define	SERVO_LEFT_TILT_OFFSET			-50
#define	SERVO_LEFT_TILT_UP_MIN			600
#define	SERVO_LEFT_TILT_DOWN_MAX		2500

/*
	Pin 10 has been intentionally left unconnected
*/

//	Lesser = , Greater = 
#define	SERVO_RIGHT_TILT_PIN			11
#define SERVO_RIGHT_TILT_NAME			"Right Tilt"
#define	SERVO_RIGHT_TILT_HOME			SERVO_CENTER_MS
#define	SERVO_RIGHT_TILT_OFFSET			-50
#define	SERVO_RIGHT_TILT_UP_MIN			600
#define	SERVO_RIGHT_TILT_DOWN_MAX		2500

//	Left Servo Motor assignments, Lesser = , Greater =
#define	SERVO_MOTOR_LEFT_PIN			12
#define SERVO_MOTOR_LEFT_NAME			"Left Servo Motor"
#define	SERVO_MOTOR_LEFT_NEUTRAL		SERVO_MOTOR_NEUTRAL
#define	SERVO_MOTOR_LEFT_OFFSET 		0
#define	SERVO_MOTOR_LEFT_SPEED_ADJ		-35
#define	SERVO_MOTOR_LEFT_DIRECTION		false
#define	SERVO_MOTOR_LEFT_MIN			500
#define	SERVO_MOTOR_LEFT_MAX			2500

/*********************************************************
	Structs for data we store on various onboard devices
*********************************************************/

typedef enum MotorLocation {
	Left,
	Right,
	Front,
	Back
};

struct bmp180Data {
	sensors_event_t tempEvent;
	float seaLevelPressure;

	bool temperatureValid;
	float celsius;
	float fahrenheit;

	float altitude;
};

//	L3GD20 Gyroscope data
struct l3gd20Data {
	int X;
	int Y;
	int Z;
};

//	LSM303DLHC Accelerometer/Magnetometer (Compass) data
struct lsm303dlhcData {
	sensors_event_t accelEvent;
	sensors_event_t compassEvent;

	float accelX, accelY, accelZ;
	float compassX, compassY, compassZ;
};

//	The 10DOF Inertial Measurement Unit (IMU)
struct InertialMeasurementUnit {
	bmp180Data tempData;
	lsm303dlhcData accelCompassData;
	l3gd20Data gyroData;

	sensors_vec_t orientation;

	bool pitchRollValid;
	bool headingValid;
};

//	TC S34725 RGB Color Sensor
struct ColorSensor {
	uint16_t colorTempC;
	uint16_t lux;

	uint16_t red;
	uint16_t green;
	uint16_t blue;

	uint16_t c;
};

//	TMP006 Heat Sensor
struct HeatSensor {
	float dieTempC;
	float objectTempC;
};

//	Holds are closest and farthest object distance readinga
struct DistanceObject {
	uint8_t closestPING;
	uint16_t closestPosPING;

	uint8_t closestIR;
	uint16_t closestPosIR;

	uint8_t farthestPING;
	uint16_t farthestPosPING;

	uint8_t farthestIR;
	uint16_t farthestPosIR;
};

//	For areaScan() readings
struct AreaScanReading {
	float ir;

	ColorSensor color;
	HeatSensor heat;

	int positionDeg;
};

//	Continuous Rotation Servos - R/C PWM control mode parameters
struct ServoMotor {
	Servo servo;

	uint8_t pin;
	String descr;

	int offset;
	int8_t speedAdjustment;
	bool forward;
	uint16_t neutral;
	uint16_t minPulse;
	uint16_t maxPulse;
	int mspeed;

	uint16_t pulse;

	uint16_t error;
};

//	DC Motors - Packet Serial control mode parameters
struct GearMotor {
	String descr;
	MotorLocation location;

	uint32_t encoder;
	uint8_t encoderStatus;
	bool encoderValid;

	uint32_t mspeed;
	uint8_t speedStatus;
	bool speedValid;

	bool forward;

	long distance;
	bool distanceValid;

	uint16_t error;
};

//	Standard R/C Servos
struct StandardServo {
	Servo servo;

	uint8_t pin;
	String descr;

	int offset;
	uint16_t homePos;
	uint16_t msPulse;
	int angle;
	uint16_t minPulse;
	uint16_t maxPulse;
	uint8_t maxDegrees;

	uint16_t error;
};

#endif
