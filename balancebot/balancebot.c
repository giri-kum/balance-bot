/*******************************************************************************
* balancebot.c
*
* Main template code for the balanceBot
* 
*******************************************************************************/
#include "balancebot.h"

/*******************************************************************************
* int main() 
*
*******************************************************************************/
int main(){
	int iter = 0;
	queue_length = 100;

	intialize_queue(0);
	mb_setpoints.fwd_velocity = 0;
	mb_setpoints.turn_velocity = 0;	
	// always initialize cape library first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to initialize rc_initialize(), are you root?\n");
		return -1;
	}

	//set cpu freq to max performance
	rc_set_cpu_freq(FREQ_1000MHZ);
	//Giri: IMU Calibration data, delete later.
	f = fopen("imu.dat", "w"); 
	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother
	if(isatty(fileno(stdout))){
		printf("starting print thread... \n");
		pthread_t  printf_thread;
		pthread_create(&printf_thread, NULL, printf_loop, (void*) NULL);
	}

	// start control thread
	printf("starting setpoint thread... \n");
	pthread_t  setpoint_control_thread;
	pthread_create(&setpoint_control_thread, NULL, setpoint_control_loop, (void*) NULL);


	// TODO: start motion capture message recieve thread

	// set up IMU configuration
	printf("initializing imu... \n");
	rc_imu_config_t imu_config = rc_default_imu_config();
	imu_config.dmp_sample_rate = SAMPLE_RATE_HZ; // Sprite: defined in mb_defs.h
	imu_config.orientation = ORIENTATION_Z_UP;

	if(rc_initialize_imu_dmp(&imu_data, imu_config)){
		fprintf(stderr,"ERROR: can't talk to IMU! Exiting.\n");
		return -1;
	}

	rc_nanosleep(10E9); // wait for imu to stabilize
	printf("Calibrating for equilibrium position \n");
	for(iter = 0; iter < queue_length; iter++)
		{		
				push_queue(wrap_angle(imu_data.dmp_TaitBryan[TB_PITCH_X]));
				rc_nanosleep(20E6);
		}
	mb_state.equilibrium_point = average_queue();
	printf("Calibration completed: %lf \n",mb_state.equilibrium_point);
	
	//initialize state mutex
    pthread_mutex_init(&state_mutex, NULL);

	//attach controller function to IMU interrupt
	printf("initializing controller...\n");
	mb_initialize_controller();

	printf("initializing motors...\n");
	mb_initialize_motors();

	printf("resetting encoders...\n");
	rc_set_encoder_pos(1, 0);
	rc_set_encoder_pos(2, 0);

	printf("initializing odometry...\n");
	mb_initialize_odometry(&mb_odometry, 0.0,0.0,0.0);

	printf("attaching imu interupt...\n");
	rc_set_imu_interrupt_func(&balancebot_controller);



	printf("we are running!!!...\n");
	// done initializing so set state to RUNNING
	rc_set_state(RUNNING); 

	// Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){

		// all the balancing is handled in the imu interupt function
		// other functions are handled in other threads
		// there is no need to do anything here but sleep

		// always sleep at some point
		usleep(100000);
	}
	
	// exit cleanly
	mb_disable_motors();
	rc_cleanup();
	fclose(f); 
	return 0;
}


/*******************************************************************************
* void balancebot_controller()
*
* discrete-time balance controller operated off IMU interrupt
* Called at SAMPLE_RATE_HZ
*
* TODO: You must implement this function to keep the balancebot balanced
*
* You may change this how you like it, many things are here as suggestions 
*
*******************************************************************************/
void balancebot_controller(){
	
	

	//lock state mutex
	pthread_mutex_lock(&state_mutex);
	// Read IMU
	mb_state.alpha = imu_data.dmp_TaitBryan[TB_PITCH_X];
	// wrap angle
	mb_state.alpha = wrap_angle(mb_state.alpha);
	push_queue(mb_state.alpha);
	//mb_state.alpha = average_queue();
	mb_state.theta = imu_data.dmp_TaitBryan[TB_YAW_Z];
	fprintf(f, "%lf,", mb_state.alpha);
	fprintf(f, "%lf,", mb_state.in_pid_d);
	fprintf(f, "%lf\n", mb_state.out_pid_d);
	// Read encoders
	mb_state.left_encoder = ENC_1_POL * rc_get_encoder_pos(1);
    mb_state.right_encoder = ENC_2_POL * rc_get_encoder_pos(2);

    // Update odometry 
    mb_update_odometry(&mb_odometry, &mb_state);

    // Calculate controller outputs
    mb_controller_update(&mb_state,&mb_setpoints);

    //unlock state mutex
    pthread_mutex_unlock(&state_mutex);

    // reset encoders to 0
    rc_set_encoder_pos(1, 0);
    rc_set_encoder_pos(2, 0);

    mb_set_motor(RIGHT_MOTOR, mb_state.right_cmd);
    mb_set_motor(LEFT_MOTOR, mb_state.left_cmd);
    
    // Sprite: commented out for debugging purposese, uncomment later
   /*  if(!mb_setpoints.manual_ctl){
     	mb_set_motor(RIGHT_MOTOR, mb_state.right_cmd);
   	 	mb_set_motor(LEFT_MOTOR, mb_state.left_cmd);
   	 }

     if(mb_setpoints.manual_ctl){
     	mb_set_motor(RIGHT_MOTOR, mb_setpoints.fwd_velocity + mb_setpoints.turn_velocity);
   	 	mb_set_motor(LEFT_MOTOR, mb_setpoints.fwd_velocity - mb_setpoints.turn_velocity);
   	 }
    
*/
    // TODO: Set motor velocities
	

}


/*******************************************************************************
*  setpoint_control_loop()
*
*  sets current setpoints based on dsm radio data, odometry, and Optitrak
*
*
*******************************************************************************/
void* setpoint_control_loop(void* ptr){

	// start dsm listener for radio control
	rc_initialize_dsm();

	while(1){
		if (rc_is_new_dsm_data()) 
		{		
			// TODO: Handle the DSM data from the Spektrum radio reciever
			// You may also implement switching between manual and autonomous mode
			// using channel 5 of the DSM data.
			if(rc_get_dsm_ch_normalized(5) > 0.0)
				{
					mb_setpoints.manual_ctl = 1;
					mb_setpoints.fwd_velocity = FWD_VEL_SENSITIVITY * rc_get_dsm_ch_normalized(3);
					mb_setpoints.turn_velocity = TURN_VEL_SENSITIVITY * rc_get_dsm_ch_normalized(4);
				}
			else{ //Autonomous mode: set points are controlled by the code, it is zero for the time being.
				mb_setpoints.manual_ctl = 0;
				mb_setpoints.fwd_velocity = 0;
				mb_setpoints.turn_velocity = 0;
				}
	 	}
	 	else
		{
			mb_setpoints.fwd_velocity = 0;
			mb_setpoints.turn_velocity = 0;	
		}
	 	usleep(1000000 / RC_CTL_HZ);
	}
}




/*******************************************************************************
* printf_loop() 
*
* prints diagnostics to console
* this only gets started if executing from terminal
*
* TODO: Add other data to help you tune/debug your code
*******************************************************************************/
void* printf_loop(void* ptr){
	rc_state_t last_state, new_state; // keep track of last state
	FILE * f1;
	f1 = fopen("pid.dat", "w");

	while(rc_get_state()!=EXITING){
		new_state = rc_get_state();
		// check if this is the first time since being paused
		if(new_state==RUNNING && last_state!=RUNNING){
			printf("\nRUNNING: Hold upright to balance.\n");
			printf("                 SENSORS               |           ODOMETRY          |");
			printf("\n");
			printf("    α    |");
			printf("    θ    |");
			printf("  L Enc  |");
			printf("  R Enc  |");
			printf("  L_u    |");
			printf("  R_u    |");
			printf("    X    |");
			printf("    Y    |");
			printf("    θ    |");
			/*printf("  CH3    |");
			printf("  CH4    |");
			printf("  CH5    |");*/
			printf("  error  |");
			printf("  L_P    |");
			printf("  out_P  |");
			printf("  L_I    |");
			printf("  out_I  |");
			printf("  L_D    |");
			printf("  out_D  |");
			printf("  xdot   |");
			printf("Des_alpha|");
			printf("\n");
/*
			fputs("α,", f1);
			fputs("θ,", f1);
			fputs("L Enc,", f1);
			fputs("R Enc,", f1);
			fputs("L_u,", f1);
			fputs("R_u,", f1);
			fputs("X,", f1);
		    fputs("Y,", f1);
			fputs("θ,", f1);
			fputs("error,", f1);
			fputs("L_P,", f1);
			fputs("R_P,", f1);
			fputs("L_I,", f1);
			fputs("R_I,", f1);
		    fputs("L_D,", f1);
			fputs("R_D,", f1);
			fputs("xdot,", f1);
			fputs("Target_alpha", f1);
		    fputs("\n", f1);
*/
		}
		else if(new_state==PAUSED && last_state!=PAUSED){
			printf("\nPAUSED\n");
		}
		last_state = new_state;
		
		if(new_state == RUNNING){
			
			printf("\r");
			//Add Print statements here, do not follow with /n
			printf("%7.3f  |", mb_state.alpha);
			printf("%7.3f  |", mb_state.theta);
			printf("%7d  |", mb_state.left_encoder);
			printf("%7d  |", mb_state.right_encoder);
			printf("%7.3f  |", mb_state.left_cmd);
			printf("%7.3f  |", mb_state.right_cmd);
			printf("%7.3f  |", mb_odometry.x);
			printf("%7.3f  |", mb_odometry.y);
			printf("%7.3f  |", mb_odometry.theta);
			/*printf("%7.3f  |", rc_get_dsm_ch_normalized(3));
			printf("%7.3f  |", rc_get_dsm_ch_normalized(4));
			printf("%7.3f  |", rc_get_dsm_ch_normalized(5));*/
			printf("%7.3f  |", mb_state.error);
			printf("%7.3f  |", mb_state.in_pid_p);
			printf("%7.3f  |", mb_state.out_pid_p);
			printf("%7.3f  |", mb_state.in_pid_i);
			printf("%7.3f  |", mb_state.out_pid_i);
			printf("%7.3f  |", mb_state.in_pid_d);
			printf("%7.3f  |", mb_state.out_pid_d);
			printf("%7.3f  |", mb_state.xdot);
			printf("%7.3f  |", mb_state.desired_alpha);
			fflush(stdout);
			fprintf(f1, "%lf,", mb_state.alpha);
			fprintf(f1, "%lf,", mb_state.theta);
			fprintf(f1, "%d,", mb_state.left_encoder);
			fprintf(f1, "%d,", mb_state.right_encoder);
			fprintf(f1, "%lf,", mb_state.left_cmd);
			fprintf(f1, "%lf,", mb_state.right_cmd);
			fprintf(f1, "%lf,", mb_odometry.x);
			fprintf(f1, "%lf,", mb_odometry.y);
			fprintf(f1, "%lf,", mb_odometry.theta);
			fprintf(f1, "%lf,", mb_state.error);
			fprintf(f1, "%lf,", mb_state.in_pid_p);
			fprintf(f1, "%lf,", mb_state.out_pid_p);
			fprintf(f1, "%lf,", mb_state.in_pid_i);
			fprintf(f1, "%lf,", mb_state.out_pid_i);
			fprintf(f1, "%lf,", mb_state.in_pid_d);
			fprintf(f1, "%lf,", mb_state.out_pid_d);
			fprintf(f1, "%lf,", mb_state.xdot);
			fprintf(f1, "%lf\n", mb_state.desired_alpha);
		}
		usleep(1000000 / PRINTF_HZ);
	}
	fclose(f1);
	return NULL;
} 

void push_queue(float value)
{
	for(int i = 0; i<queue_length-1;i++)
		{
			filter_queue[i]= filter_queue[i+1];
		}
	filter_queue[queue_length-1] = value;
}
void intialize_queue(float value)
{
	filter_queue[0] = value;
	for(int i = 0; i<queue_length-1;i++)
		{
			filter_queue[i+1]= filter_queue[i];
		}
}

float average_queue()
{
	float sum = 0;
	for (int i = 0; i < queue_length; ++i)
	{
		sum += filter_queue[i];
	}
	return sum/queue_length;
}
float wrap_angle(float value)
{
	return value>0? (value):(value + TWO_PI);
}