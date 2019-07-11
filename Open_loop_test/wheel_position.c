/*******************************************************************************
* wheel_position.c
*
* Simple open-loop test involving encoder positions.
* Right wheel of MiP is used to tune speed of the
* left wheel. As the position of the right wheel
* (in rads) increases (+/- directions), the left
* wheel's speed also increases opposite the
* direction the right wheel is tuned.
*******************************************************************************/
#include <rc_usefulincludes.h>
#include <roboticscape.h>
#include "./feedback_config.h"

// function declarations
void on_pause_pressed();
void on_pause_released();

/*******************************************************************************
* int main()
*
* This template main function contains these critical components
* - call to rc_initialize() at the beginning
* - main while loop that checks for EXITING condition
* - open-loop test conducted in while loop
* - rc_cleanup() at the end
*******************************************************************************/
int main(){
	// always initialize cape library first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to initialize rc_initialize(), are you root?\n");
		return -1;
	}

	// do your own initialization here
	printf("\nWheel Position Check\n");
	rc_set_pause_pressed_func(&on_pause_pressed);
	rc_set_pause_released_func(&on_pause_released);
	rc_enable_motors();

	// create local variables
    float l_wheel;
    float r_wheel;

	// done initializing so set state to RUNNING
	rc_set_state(RUNNING);

	// Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){
		// handle other states
		if(rc_get_state()==RUNNING){
            // find radians right wheel turns
            r_wheel=(rc_get_encoder_pos(ENCODER_CHANNEL_R) \
            *ENCODER_POLARITY_R*TWO_PI/(GEARBOX*ENCODER_RES));
            // find radians left wheel turns
            l_wheel=(rc_get_encoder_pos(ENCODER_CHANNEL_L) \
            *ENCODER_POLARITY_L*TWO_PI/(GEARBOX*ENCODER_RES));
            // power left wheel in direction opposite right wheel
            rc_set_motor(MOTOR_CHANNEL_L,-r_wheel*MOTOR_POLARITY_L);

            // print angular position of wheels (rad)
            printf("Radians Turned(Left): %6f, ",l_wheel);
            printf("Radians Turned(Right): %6f \n",r_wheel);

			rc_set_led(GREEN, ON);
			rc_set_led(RED, OFF);
		}
		else if(rc_get_state()==PAUSED){
			// do other things
			rc_set_led(GREEN, OFF);
			rc_set_led(RED, ON);
		}
		// always sleep at some point
		usleep(100000);
	}

	// exit cleanly
	rc_disable_motors();
	printf("Motors Disabled");
	rc_cleanup();
	return 0;
}


/*******************************************************************************
* void on_pause_released()
*
* Make the Pause button toggle between paused and running states.
*******************************************************************************/
void on_pause_released(){
	// toggle between paused and running modes
	if(rc_get_state()==RUNNING){
        rc_set_state(PAUSED);
	}
	else if(rc_get_state()==PAUSED){
        rc_set_state(RUNNING);
	}
	return;
}

/*******************************************************************************
* void on_pause_pressed()
*
* If the user holds the pause button for 2 seconds, set state to exiting which
* triggers the rest of the program to exit cleanly.
*******************************************************************************/
void on_pause_pressed(){
	int i=0;
	const int samples = 100;	// check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds

	// now keep checking to see if the button is still held down
	for(i=0;i<samples;i++){
		rc_usleep(us_wait/samples);
		if(rc_get_pause_button() == RELEASED) return;
	}
	printf("long press detected, shutting down\n");
	rc_set_state(EXITING);
	return;
}
