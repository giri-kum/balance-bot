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
	float sum = 0;
	mb_setpoints.fwd_velocity = 0;
	mb_setpoints.turn_velocity = 0;	
	prev_imu_theta = 0;
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


	// start lcm handle thread
	printf("starting lcm thread... \n");
	lcm = lcm_create(NULL);
	pthread_t lcm_subscribe_thread;
    pthread_create(&lcm_subscribe_thread, NULL, lcm_subscribe_loop, (void*) NULL);
	
	// set up IMU configuration
	printf("initializing imu... \n");
	rc_imu_config_t imu_config = rc_default_imu_config();
	imu_config.dmp_sample_rate = SAMPLE_RATE_HZ; // Sprite: defined in mb_defs.h
	imu_config.orientation = ORIENTATION_Z_UP;

	if(rc_initialize_imu_dmp(&imu_data, imu_config)){
		fprintf(stderr,"ERROR: can't talk to IMU! Exiting.\n");
		return -1;
	}
	mb_load_setpoint_config();
    mb_load_controller_config(); //Reading from pid.cfg file
    printf("setpoints %f, %f, %f\n", mb_setpoints.position[0],mb_setpoints.position[1],mb_setpoints.heading);
	rc_nanosleep(10E9); // wait for imu to stabilize
	if(calibrate_imu - 1.0 < 0.0001)
	{
		printf("Calibrating for equilibrium position \n");
		for(iter = 0; iter < 100; iter++)
			{		
					sum = sum + (wrap_angle(imu_data.dmp_TaitBryan[TB_PITCH_X]));
					rc_nanosleep(20E6);
			}
		mb_state.equilibrium_point = sum/100;
		prev_imu_theta = mb_state.equilibrium_point;
	}
	else
	{
		mb_state.equilibrium_point = calibrate_imu;
		prev_imu_theta = mb_state.equilibrium_point;	
	}
	mb_state.count = 0;
	
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
	if(use_optitrack == 1)
		mb_initialize_odometry(&mb_odometry, mb_state.opti_x ,mb_state.opti_y,mb_state.opti_theta);
	else
		mb_initialize_odometry(&mb_odometry, 0.0 ,0.0,0.0);
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
	mb_state.imu_theta = wrap_angle(imu_data.dmp_TaitBryan[TB_YAW_Z]);

	mb_state.imu_deltheta = (mb_state.imu_theta-prev_imu_theta);
	prev_imu_theta = mb_state.imu_theta;
	
	if(mb_state.imu_deltheta>6 || mb_state.imu_deltheta<-6)
		mb_state.imu_deltheta = 0;
	if(mb_state.thetadot>6*SAMPLE_RATE_HZ || mb_state.thetadot<-6*SAMPLE_RATE_HZ)
		mb_state.thetadot = 0;
	fprintf(f, "%lf,", mb_state.alpha);
	fprintf(f, "%lf,", mb_state.imu_theta);
	fprintf(f, "%lf,", mb_state.theta);
	fprintf(f, "%lf,", mb_state.xdot);
	fprintf(f, "%lf,", mb_state.thetadot);
	fprintf(f, "%lf,", mb_state.imu_deltheta);
	fprintf(f, "%lf,", mb_state.odometry_deltheta);
	fprintf(f, "%lf\n", mb_odometry.final_deltheta);
			
	//fprintf(f, "%lf,", mb_state.in_pid_d);
	//fprintf(f, "%lf,", mb_state.out_pid_d);
	//fprintf(f, "%lf\n", mb_state.turn_pid_d);
	// Read encoders
	mb_state.left_encoder = ENC_1_POL * rc_get_encoder_pos(1);
    mb_state.right_encoder = ENC_2_POL * rc_get_encoder_pos(2);

    // Update odometry 
    mb_update_odometry(&mb_odometry, &mb_state, sensor_scheme);

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
				}
	 	}
	 	else
		{
			printf("Joy stick off!!");
			mb_setpoints.manual_ctl = 0;
			mb_setpoints.position[0]=0;
			mb_setpoints.position[1]=0;
			mb_setpoints.heading=0;
			mb_setpoints.fwd_velocity = 0;
			mb_setpoints.turn_velocity = 0;	
		}
	 	usleep(1000000 / RC_CTL_HZ);
	}
}

/*******************************************************************************
*  optitrack_message_handler()
*
*  handler function for optitrack driver messages
*  optitrack_driver must be running and optitrack must be set up
*
*******************************************************************************/
void optitrack_message_handler(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const balancebot_msg_t* msg,
                                void* userdata){

    // lock the state mutex
    pthread_mutex_lock(&state_mutex);
    mb_state.bb_msg = balancebot_msg_t_copy(msg);

    // copy the data from the message to the state
    mb_state.opti_x = mb_state.bb_msg->pose.x;
    mb_state.opti_y = mb_state.bb_msg->pose.y;
    mb_state.opti_theta = mb_state.bb_msg->pose.theta;


    pthread_mutex_unlock(&state_mutex);
}

/*******************************************************************************
* lcm_subscribe_loop() 
*
* thread subscribes to lcm channels and sets handler functions
* then handles lcm messages in a non-blocking fashion
*
* TODO: Add other subscriptions as needed
*******************************************************************************/
void *lcm_subscribe_loop(void *data){

    balancebot_msg_t_subscribe(lcm,
                         OPTITRACK_CHANNEL,
                         optitrack_message_handler,
                         NULL);

    while(1){
        // define a timeout (for erroring out) and the delay time
        lcm_handle_timeout(lcm, 1);
        usleep(1000000 / LCM_HZ);
    }
    lcm_destroy(lcm);
    return 0;
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
			printf("                 SENSORS               |           OPTITRAK          |");
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
			printf(" opti_X  |");
			printf(" opti_Y  |");
			printf(" opti_θ    |");
			printf("positionP|");
			printf("heading_P|");
			printf("error_P  |");
			printf("error_he |");
			printf("  xdot   |");
			printf("states|");
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
			printf("%7.3f  |", mb_state.imu_theta);
			printf("%7d  |", mb_state.left_encoder);
			printf("%7d  |", mb_state.right_encoder);
			printf("%7.3f  |", mb_state.left_cmd);
			printf("%7.3f  |", mb_state.right_cmd);
			printf("%7.3f  |", mb_odometry.x);
			printf("%7.3f  |", mb_odometry.y);
			printf("%7.3f  |", mb_odometry.theta);
			printf("%7.3f  |", mb_state.opti_x);
			printf("%7.3f  |", mb_state.opti_y);
			printf("%7.3f  |", mb_state.opti_theta);

			printf("%7.3f  |", mb_state.position_pid_p);
			printf("%7.3f  |", mb_state.heading_pid_p);
			printf("%7.3f  |", mb_state.error_position);
			printf("%7.3f  |", mb_state.error_heading);
			printf("%7.3f  |", mb_state.xdot);
			printf("%d%d%d |", states[0], states[1],states[2]);
		//Add Print stattements here, do not follow with /n
			fflush(stdout);
			fprintf(f1, "%lf,", mb_state.alpha);
			fprintf(f1, "%lf,", mb_state.imu_theta);
			fprintf(f1, "%d,", mb_state.left_encoder);
			fprintf(f1, "%d,", mb_state.right_encoder);
			fprintf(f1, "%lf,", mb_state.left_cmd);
			fprintf(f1, "%lf,", mb_state.right_cmd);
			fprintf(f1, "%lf,", mb_odometry.x);
			fprintf(f1, "%lf,", mb_odometry.y);
			fprintf(f1, "%lf,", mb_odometry.theta);
			fprintf(f1, "%lf,", mb_state.opti_x);
			fprintf(f1, "%lf,", mb_state.opti_y);
			fprintf(f1, "%lf,", mb_state.opti_theta);
			fprintf(f1, "%lf,", mb_state.position_pid_p);
			fprintf(f1, "%lf,", mb_state.heading_pid_p);
			fprintf(f1, "%lf,", mb_state.error_position);
			fprintf(f1, "%lf,", mb_state.error_heading);
			fprintf(f1, "%lf,", mb_state.xdot);
			fprintf(f1, "%d%d%d,", states[0], states[1],states[2]);
			fprintf(f1, "%lf,", mb_state.turn_pid_p);
			fprintf(f1, "%lf,", mb_state.turn_pid_i);
			fprintf(f1, "%lf,", mb_state.turn_pid_d);
			fprintf(f1, "%lf,", mb_state.position_pid_p);
			fprintf(f1, "%lf,", mb_state.heading_pid_p);
			fprintf(f1, "%lf,", mb_state.imu_deltheta);
			fprintf(f1, "%lf,", mb_state.odometry_deltheta);
			fprintf(f1, "%lf,", mb_odometry.final_deltheta);
			fprintf(f1, "%lf,", mb_state.theta);
			fprintf(f1, "%lf,", mb_setpoints.heading);
			fprintf(f1, "%lf\n", mb_state.thetadot);
			

		}
		usleep(1000000 / PRINTF_HZ);
	}
	fclose(f1);
	return NULL;
} 

float wrap_angle(float value)
{
	return value>0? (value):(value + TWO_PI);
}

int mb_load_setpoint_config(){
    FILE* file = fopen("setpoints.cfg", "r");
    if (file == NULL){
        printf("Error opening setpoints.cfg\n");
    }
				
    //TODO: You can add to or modify the cfg file as you like
    // This is here as an example
    fscanf(file, "%f %f %f ", 
        &mb_setpoints.position[0],
        &mb_setpoints.position[1],
        &mb_setpoints.heading
        );

    fclose(file);
   
    return 0;
}


int get_motor_state()
{
	float tolerance_position = 0.1, tolerance_angle = 0.1;
	if(mb_state.error_position < tolerance_position && mb_state.error_heading < tolerance_angle)
		return 0; // idle state
	else
		return 1; // motion state
}

int get_rtr_state()
{
	switch(states[1])
	{
		case 0: return 1; // from idle state to rotation 1 state
		case 1: return 2; // from idle rotation 1 to translation state
		case 2: return 3; // from translation to rotation 2 state
		case 3: return 0; // from rotation 2 to idle state
		default: return 0;
	}
}

int get_waypoint_state()
{
float tolerance_position = 0.01;
	if(mb_odometry.x - mb_setpoints.position[0] < tolerance_position && mb_odometry.y - mb_setpoints.position[1] < tolerance_position)
		return 0; // reached target
	else
		return 1; //not reached target yet
}

void statemachine()
{
	states[0] = get_motor_state();
	if(states[0] == 0)
		{
			states[1] = get_rtr_state();
			if(states[1] == 0)
				states[2] = get_waypoint_state();
		}
}

