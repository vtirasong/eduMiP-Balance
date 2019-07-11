/*******************************************************************************
* read_data.c
*
* Reads the IMU and converts accelerometer and gyroscope
* data into angle values (in radians) of the BeagleBone
* relative to the x-axis. Prints unfiltered values of
* theta.
*******************************************************************************/
#include <rc_usefulincludes.h>
#include <roboticscape.h>

// variable declarations
rc_imu_data_t imu_read;
float theta_a_raw;
float theta_g_raw;

// function declarations
void on_pause_pressed();
void on_pause_released();
int imu_angles();

/*******************************************************************************
* int main()
*
* This template main function contains these critical components
* - call to rc_initialize() at the beginning
* - configures imu defaults and sets an interrupt function
* - main while loop that checks for EXITING condition
* - rc_cleanup() at the end
*******************************************************************************/
int main(){
	// always initialize cape library first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to initialize rc_initialize(), are you root?\n");
		return -1;
	}

	// do your own initialization here
	printf("\nRead IMU Data\n");
	rc_set_pause_pressed_func(&on_pause_pressed);
	rc_set_pause_released_func(&on_pause_released);

	// set default IMU configuration
	rc_imu_config_t config = rc_default_imu_config();

	// initialize IMU
	if(rc_initialize_imu_dmp(&imu_read,config)){
            printf("Error initializing IMU\n");
            return -1;
	}

	// print imu angle values
	rc_set_imu_interrupt_func(&imu_angles);

	// done initializing so set state to RUNNING
	rc_set_state(RUNNING);

	// Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){
		// handle other states
		if(rc_get_state()==RUNNING){
            // do things
			rc_set_led(GREEN, ON);
			rc_set_led(RED, OFF);
		}
		else if(rc_get_state()==PAUSED){
			// do other things
			rc_set_led(GREEN, OFF);
			rc_set_led(RED, ON);
		}
		// set 100 Hz timing
		rc_usleep(100000);
	}

	// exit cleanly
	rc_power_off_imu();
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
	if(rc_get_state()==RUNNING)		rc_set_state(PAUSED);
	else if(rc_get_state()==PAUSED)	rc_set_state(RUNNING);
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

/*******************************************************************************
* int imu_angles()
*
* Converts accelerometer and gyroscope data into angle values (in radians) of
* the BeagleBone relative to the x-axis. These values are then printed to the
* console as unfiltered theta values.
*******************************************************************************/
int imu_angles(){
    // compute accelerometer angle of BeagleBone relative to x-axis
    theta_a_raw=atan2(-imu_read.accel[2],imu_read.accel[1]);
    // use Euler's integration on gyroscope x-axis data
    theta_g_raw+=(imu_read.gyro[0]*DEG_TO_RAD)/100;
    // print values to console at 100 Hz
    printf("\r");
    printf("theta_a_raw= %f,theta_g_raw= %f",theta_a_raw,theta_g_raw);
    fflush(stdout);
    rc_usleep(10000);
    return 0;
}
