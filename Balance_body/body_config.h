/*******************************************************************************
* body_config.h
*
* Contains defines for balance_body.c
* Defines properties for controller D1, the complementary filter,
* the eduMiP, the motors, and the encoders.
*******************************************************************************/

#ifndef BODY_CONFIG
#define BODY_CONFIG

// timing constants
#define MICRO                   1000000 // 10^6 microseconds
#define D1_HZ                   100

// structural properties of eduMiP
#define GEARBOX 				35.577
#define ENCODER_RES				60

// MiP balance constants
#define TIP_ANGLE               0.8 // radians from y-axis (~45 degrees)
#define THETA_REFERENCE         0

// complementary filter constants
#define OMEGA_C                 2 // 1/time constant
#define DT                      0.01 // step in seconds
#define THETA_OFFSET            0.15

// inner loop controller
#define D1_GAIN					-4.24
#define D1_N				    2 // # of zeros in numerator
#define D1_M                    2 // # of poles in denominator
#define D1_NUM					{1, -1.678, 0.6931}
#define D1_DEN					{1, -1.566, 0.566}
#define D1_SATURATION        	1

// electrical hookups
#define MOTOR_CHANNEL_L			3
#define MOTOR_CHANNEL_R			2
#define MOTOR_POLARITY_L		1
#define MOTOR_POLARITY_R		-1
#define ENCODER_CHANNEL_L		3
#define ENCODER_CHANNEL_R		2
#define ENCODER_POLARITY_L		1
#define ENCODER_POLARITY_R		-1

// controller structure
typedef struct controller_d_t{
    float gain;
    int n;
    int m;
    float numerator[3];
    float denominator[3];
    float inputs[3];
    float outputs[3];
    float saturation;
} controller_d_t;

#endif	//BODY_CONFIG
