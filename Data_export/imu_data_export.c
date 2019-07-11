/*******************************************************************************
* imu_data_export.c
*
* Prints filtered accelerometer and gyroscope data and
* exports the theta values to a text file for external
* use and plotting.
*******************************************************************************/
#include <rc_usefulincludes.h>
#include <roboticscape.h>

// variable declarations
rc_imu_data_t imu_read;
float theta_a_raw;
float theta_g_raw;
float theta_g_prev;
float theta_a;
float theta_g;
float theta_f;

// set predefined variables
float omega_c=2;
float step_size=0.01;
float micro=1000000;
float sample_freq=100;
float print_freq=10;

// function declarations
void on_pause_pressed();
void on_pause_released();
void* theta_display();
void* data_export();
int imu_filters();

/*******************************************************************************
* int main()
*
* This template main function contains these critical components
* - call to rc_initialize() at the beginning
* - sets imu configuration and interrupt function
* - creates threads for printing and exporting theta data
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
	printf("\nExport IMU Data\n");
	rc_set_pause_pressed_func(&on_pause_pressed);
	rc_set_pause_released_func(&on_pause_released);

	// set default IMU configuration
	rc_imu_config_t config = rc_default_imu_config();

	// initialize IMU
	if(rc_initialize_imu_dmp(&imu_read,config)){
            printf("Error initializing IMU\n");
            return -1;
	}

	// set initial values for filtering
	theta_a_raw=0.0;
	theta_g_raw=0.0;
	theta_a=theta_a_raw;
	theta_g=theta_g_raw;
	theta_g_prev=0.0;

	// filter imu angle values
	rc_set_imu_interrupt_func(&imu_filters);

	// print filtered theta values
	pthread_t theta_thread;
	pthread_create(&theta_thread,NULL,theta_display,(void*)NULL);

	// export filtered data
	pthread_t data_thread;
	pthread_create(&data_thread,NULL,data_export,(void*)NULL);

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
		// always sleep at some point
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
* int imu_filters()
*
* Converts accelerometer and gyroscope data into angle values (in radians) of
* the BeagleBone relative to the x-axis. These values are then passed through
* low-pass (accelerometer data) and high-pass (gyroscope data) filters.
*******************************************************************************/
int imu_filters(){
    // compute accelerometer angle of BeagleBone relative to x-axis
    theta_a_raw=atan2(-imu_read.accel[2],imu_read.accel[1]);
    // use Euler's integration on gyroscope x-axis data
    theta_g_raw+=(imu_read.gyro[0]*DEG_TO_RAD)/sample_freq;

    // apply a low pass filter to theta_a_raw
    theta_a=(1-omega_c*step_size)*theta_a+(omega_c*step_size)*theta_a_raw;
    // apply a high-pass filter to theta_g_raw
    theta_g=(1-omega_c*step_size)*theta_g+theta_g_raw-theta_g_prev;
    theta_f=theta_a+theta_g;

    // update theta_g_prev value
    theta_g_prev=theta_g_raw;
    // set 100 Hz timing
    rc_usleep(micro/sample_freq);
    return 0;
}

/*******************************************************************************
* void* theta_display()
*
* Displays filtered theta values calculated and retrieved from imu_filters.
*******************************************************************************/
void* theta_display(){
    // print filtered theta values to screen
    while(rc_get_state()!=EXITING){
        printf("\r");
        printf("theta_a= %f,theta_g= %f,theta_f= %f",theta_a,theta_g,theta_f);
        fflush(stdout);
        // set 100 Hz timing
        rc_usleep(micro/sample_freq);
    }
    return NULL;
}

/*******************************************************************************
* void* data_export()
*
* Creates a text file to store filtered data for external use. Samples data
* at 10 Hz.
*******************************************************************************/
void* data_export(){
    // create text file to store filtered values for
    // MATLAB plotting
    FILE *theta_data;
    theta_data=fopen("theta_data.txt","w");
    fprintf(theta_data,"time(s),theta_a,theta_g,theta_f\n");
    float time_count=0.0;

    while(rc_get_state()!=EXITING){
        fprintf(theta_data,"%f,%f,%f,%f\n",time_count/print_freq, \
                theta_a,theta_g,theta_f);
        time_count++;
        // set 10 Hz timing
        rc_usleep(micro/print_freq);
    }
    fclose(theta_data);
    return NULL;
}
