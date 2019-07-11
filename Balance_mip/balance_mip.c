/*******************************************************************************
* balance_mip.c
*
* Balances the body angle of the MiP and the position of the wheels.
*******************************************************************************/
#include <rc_usefulincludes.h>
#include <roboticscape.h>
#include "mip_config.h"

// function declarations
controller_d_t initialize_controller(float gain,int n, int m,float* num, \
                                     float* den,float sat);
float complementary_filter();
float control_step(controller_d_t* d,float loop_error);
void clear_controls(controller_d_t* d);
void clear_encoders();
void on_pause_pressed();
void on_pause_released();
void initialize_ops();
void suspend_ops();
void inner_loop();
void* outer_loop();

// variable declarations
rc_imu_data_t imu_reader;
controller_d_t D1;
controller_d_t D2;
float theta_f;
float theta_r;
float current_theta;

/*******************************************************************************
* int main()
*
* This template main function contains these critical components
* - call to rc_initialize() at the beginning
* - configuration and initialization of IMU
* - initialization of controllers D1 and D2
* - IMU interrupt function set to inner loop at 100 Hz
* - outer loop pthread set to 20 Hz
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
	printf("\nBalance Body\n");
	rc_set_pause_pressed_func(&on_pause_pressed);
	rc_set_pause_released_func(&on_pause_released);

	// set default IMU configuration
	rc_imu_config_t config = rc_default_imu_config();

	// initialize IMU for DMP mode
	if(rc_initialize_imu_dmp(&imu_reader,config)){
            printf("Error initializing IMU\n");
            return -1;
	}

	// create controllers
	float D1_num[]=D1_NUM;
	float D1_den[]=D1_DEN;
	D1=initialize_controller(D1_GAIN,D1_N,D1_M,D1_num, \
                             D1_den,D1_SATURATION);

    float D2_num[]=D2_NUM;
    float D2_den[]=D2_DEN;
    D2=initialize_controller(D2_GAIN,D2_N,D2_M,D2_num, \
                             D2_den,D2_SATURATION);

	// set inner loop as IMU interrupt function
	rc_set_imu_interrupt_func(&inner_loop);

	// create thread for outer loop
	pthread_t outer_loop_thread;
	pthread_create(&outer_loop_thread,NULL,outer_loop,(void*) NULL);

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
		usleep(100000);
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
* void inner_loop()
*
* Retrieves angle of the body of the MiP from the complementary filter.
* The difference between the reference theta and the angle of the body
* is then used as an input for controller D1, which will then produce
* an appropriate duty to balance the MiP.
*******************************************************************************/
void inner_loop(){
    // initialize local variables
    float control_duty,theta_error;
    // find current angle of MiP
    current_theta=complementary_filter();
    // check for tipping
    if(fabs(current_theta)>TIP_ANGLE){
        suspend_ops();
    }
    else{
        initialize_ops();
    }
    // calculate input error and motor duty
    theta_error=theta_r-current_theta;
    control_duty=control_step(&D1,theta_error);
    // send duty to motors to balance body angle
    rc_set_motor(MOTOR_CHANNEL_L,MOTOR_POLARITY_L*control_duty);
    rc_set_motor(MOTOR_CHANNEL_R,MOTOR_POLARITY_R*control_duty);
    // set 100 Hz timing
    rc_usleep(NANO/D1_HZ);
    return;
}

/*******************************************************************************
* void* outer_loop()
*
* Calculates wheel angle from encoders. The difference between the reference
* phi and the average angle of the wheels is then used as an input for
* controller D2, which will then produce a reference theta for the inner loop.
*******************************************************************************/
void* outer_loop(){
    // initialize local variables
    float l_wheel,r_wheel,current_phi,phi_error;
    // calculate wheel positions in radians
    l_wheel=(rc_get_encoder_pos(ENCODER_CHANNEL_L) \
             *ENCODER_POLARITY_L*TWO_PI/(GEARBOX*ENCODER_RES));
    r_wheel=(rc_get_encoder_pos(ENCODER_CHANNEL_R) \
             *ENCODER_POLARITY_R*TWO_PI/(GEARBOX*ENCODER_RES));
    // calculate average wheel position and subtract out current
    // MiP body angle
    current_phi=(0.5*(l_wheel+r_wheel))-current_theta;
    // calculate input error and theta reference
    phi_error=PHI_REFERENCE-current_phi;
    theta_r=control_step(&D2,phi_error);
    // set 20 Hz timing
    rc_usleep(NANO/D2_HZ);

    return NULL;
}

/*******************************************************************************
* float complementary_filter()
*
* Converts accelerometer and gyroscope data into angle values (in radians) of
* the BeagleBone relative to the x-axis. These values are then passed through
* low-pass (accelerometer data) and high-pass (gyroscope data) filters before
* being summed to a theta angle estimate of the MIP body relative to the
* x-axis.
*******************************************************************************/
float complementary_filter(){
    // initialize theta values
    static float theta_a[2],theta_a_raw[2]={0,0};
    static float theta_g[2],theta_g_raw[2]={0,0};

    // compute accelerometer angle of BeagleBone relative to x-axis
    theta_a_raw[0]=atan2(-imu_reader.accel[2],imu_reader.accel[1]);
    // use Euler's integration on gyroscope x-axis data
    theta_g_raw[0]=theta_g_raw[1]+(imu_reader.gyro[0]*DEG_TO_RAD*DT);

    // apply a low-pass filter to theta_a_raw
    theta_a[0]=(1-OMEGA_C*DT)*theta_a[1]+(OMEGA_C*DT)*theta_a_raw[1];
    // apply a high-pass filter to theta_g_raw
    theta_g[0]=(1-OMEGA_C*DT)*theta_g[1]+theta_g_raw[0]-theta_g_raw[1];
    // calculate theta angle of MIP
    theta_f=theta_a[0]+theta_g[0]+THETA_OFFSET;

    // update theta values for next iteration
    theta_a_raw[1]=theta_a_raw[0];
    theta_g_raw[1]=theta_g_raw[0];
    theta_a[1]=theta_a[0];
    theta_g[1]=theta_g[0];

    return theta_f;
}

/*******************************************************************************
* initialize_controller()
*
* Allocates controller values to be used for difference equation
* computations. Default inputs and outputs are set to zero.
*******************************************************************************/
controller_d_t initialize_controller(float gain,int n, int m,float* num, \
                                     float* den,float sat){
    // create controller object
    controller_d_t d;
    // initialize for loop counts
    int i=0;
    int j=0;
    // allocate controller numerator values and zero inputs
    for(i=0;i<(n+1);i++){
        d.numerator[i]=num[i];
        d.inputs[i]=0;
    }
    // allocate controller denominator values and zero outputs
    for(j=0;j<(m+1);j++){
        d.denominator[j]=den[j];
        d.outputs[j]=0;
    }
    // allocate gain and saturation values
    d.gain=gain;
    d.saturation=sat;
    // save # of poles and zeros of controller
    d.n=n;
    d.m=m;
    return d;
}

/*******************************************************************************
* float control_step()
*
* Performs difference equation calculation using controller values from
* controller d and an input error.
*******************************************************************************/
float control_step(controller_d_t* d,float loop_error){
    // retrieve # of poles and zeros for calculations
    int n=d->n;
    int m=d->m;
    // initialize for loop counts
    int i=0; int j=0; int k=0; int l=0;
    // set input error as initial input
    d->inputs[0]=loop_error;
    // initialize output
    float update_error=0;
    // perform difference equation calculation
    for(i=0;i<(n+1);i++){
        d->outputs[0]+=d->gain*d->numerator[i]*d->inputs[i];
    }
    for(j=1;j<(m+1);j++){
        d->outputs[0]-=d->denominator[j]*d->outputs[j];
    }
    d->outputs[0]/=d->denominator[0];

    update_error=d->outputs[0];
    // check for saturation
    if(update_error>d->saturation){
        update_error=d->saturation;
    }
    else if(update_error<-d->saturation){
        update_error=-d->saturation;
    }
    // zero out output of difference equation
    d->outputs[0]=0;
    // update inputs and outputs for next iteration
    for(k=n;k>0;k--){
        d->inputs[k]=d->inputs[k-1];
    }
    for(l=m;l>0;l--){
        d->outputs[l]=d->outputs[l-1];
    }

    return update_error;
}

/*******************************************************************************
* void clear_controls()
*
* Clears input and output values of controllers to prevent lock-up.
*******************************************************************************/
void clear_controls(controller_d_t* d){
    int n=d->n;
    int m=d->m;
    int i=0; int j=0;
    // set input and output values to zero
    for(i=n;i>=0;i--){
        d->inputs[i]=0;
    }
    for(j=m;j>=0;j--){
        d->outputs[j]=0;
    }
    return;
}

/*******************************************************************************
* void clear_encoders()
*
* Set encoder positions to zero for wheel tracking.
*******************************************************************************/
void clear_encoders(){
    rc_set_encoder_pos(ENCODER_CHANNEL_L,0);
    rc_set_encoder_pos(ENCODER_CHANNEL_R,0);
    return;
}

/*******************************************************************************
* void initialize_ops()
*
* Enable motors and zero out controllers and encoders.
*******************************************************************************/
void initialize_ops(){
    clear_controls(&D1);
    clear_controls(&D2);
    clear_encoders();
    rc_enable_motors();
    return;
}

/*******************************************************************************
* void suspend_ops()
*
* Disable motors to stop balancing.
*******************************************************************************/
void suspend_ops(){
    rc_disable_motors();
    printf("\r");
    printf("Oops,unexpected trustfall!");
    return;
}
