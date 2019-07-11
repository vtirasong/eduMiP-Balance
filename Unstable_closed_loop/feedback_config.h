/*******************************************************************************
* feedback_config.h
*
* Contains defines for wheel_position.c
* Defines properties of eduMiP and motor polarities
*******************************************************************************/

#ifndef FEEDBACK_CONFIG
#define FEEDBACK_CONFIG

// Structural properties of eduMiP
#define GEARBOX 				35.555
#define ENCODER_RES				60
#define WHEEL_RADIUS_M			0.034
#define TRACK_WIDTH_M			0.035

// Electrical hookups
#define MOTOR_CHANNEL_L			3
#define MOTOR_CHANNEL_R			2
#define MOTOR_POLARITY_L		1
#define MOTOR_POLARITY_R		-1
#define ENCODER_CHANNEL_L		3
#define ENCODER_CHANNEL_R		2
#define ENCODER_POLARITY_L		1
#define ENCODER_POLARITY_R		-1

#endif	//FEEDBACK_CONFIG
