/*******************************************************************************
* measure_motors.c
*
* 
*******************************************************************************/
#include "../balancebot/balancebot.h"
                   // Sprite: PI is already included in rc_usefulincludes.h
#define CPR 979.2 // Sprite: counts per revolution = 20.4*48
#define TS 10000 // Sprite: define sampling period to be 10,000 microseconds could also use DT in mb_defs.h, however an additional macro definition provides more flexibility in motor measurement
#define DUTY_CYCLE 0.5    // Sprite: define duty cycle, may want to test at different duty_cycle

FILE * f1; 

void* data_loop(void* ptr){
	f1 = fopen("motor.dat", "w");

	// Sprite: local variable for storing encoder positions, and motor speed in rad/sec
	int encoder_pos_right_new, encoder_pos_left_new, encoder_pos_right_old, encoder_pos_left_old;
	encoder_pos_right_new = encoder_pos_left_new = encoder_pos_right_old = encoder_pos_left_old = 0;

	double motor_speed_right, motor_speed_left;
	motor_speed_right = motor_speed_left = 0.0;

	double current_right, current_left;
	current_right = current_left = 0.0;

	double Vs_right, Vs_left;
	Vs_right = Vs_left = 0.0;

	long int count; // Sprite: long is redundant almost for sure
	count = 0;

	double data_time;
	data_time = 0.0;

	rc_state_t last_state, new_state; // Sprite: keep track of the last state

	while(rc_get_state()!=EXITING){ // Sprite: replaced "1" with rc_get_state() != EXITING
	//TODO: take your data here
		new_state = rc_get_state();

		if(new_state==RUNNING && last_state!=RUNNING){
			fputs("Time,", f1);
			fputs("M1SupplyVoltage,", f1);
			fputs("M1Current,", f1);
			fputs("M1Speed,", f1);

			fputs("M2SupplyVoltage,", f1);
			fputs("M2Current,", f1);
			fputs("M2Speed", f1);
		    fputs("\r\n", f1);
		}
		
		last_state = new_state;

        if(new_state == RUNNING){
        	// Sprite: Motor speed computation, note the encoder polarity in here
			encoder_pos_right_new = rc_get_encoder_pos(RIGHT_MOTOR);
			encoder_pos_left_new = rc_get_encoder_pos(LEFT_MOTOR);

			motor_speed_right = (double) ((encoder_pos_right_new-encoder_pos_right_old)/(CPR)*(TWO_PI)*1000000/TS)*ENC_1_POL;
			motor_speed_left = (double) ((encoder_pos_left_new-encoder_pos_left_old)/(CPR)*(TWO_PI)*1000000/TS)*ENC_2_POL;

			encoder_pos_right_old = encoder_pos_right_new;
			encoder_pos_left_old = encoder_pos_left_new;

			data_time = (double) (((double)(++count))*TS/1000000);

			// Sprite: Motor current measurement
			current_right = (double) ((double) rc_adc_volt(MOT_1_CS)/0.5); // Sprite: Datasheet says the current
			current_left  = (double) ((double) rc_adc_volt(MOT_2_CS)/0.5);// Sprite: sense resistor is 0.5 ohm

			// Sprite: It seems that there aren't any functions to read supply voltage or voltage across the motor directly
			Vs_right = (double) (rc_battery_voltage());
			Vs_left = Vs_right;
			//Vs_right = (double) (DUTY_CYCLE*rc_dc_jack_voltage());
			//Vs_left = (double) (DUTY_CYCLE*rc_dc_jack_voltage());

			// Error Checking: commented out since not really necessary
			// if (Vs_right < 0 || Vs_left < 0){
			// 	fprintf(stderr, "ERROR: can't read voltages\n"); 
			// 	return -1;
			// }

			// Sprite: Write to the file
			fprintf(f1, "%lf", data_time);
			fputc(',', f1);

			fprintf(f1, "%lf", Vs_right);
			fputc(',', f1);
			fprintf(f1, "%lf", current_right);
			fputc(',', f1);
			fprintf(f1, "%lf", motor_speed_right);
			fputc(',', f1);

			fprintf(f1, "%lf", Vs_left);
			fputc(',', f1);
			fprintf(f1, "%lf", current_left);
			fputc(',', f1);
			fprintf(f1, "%lf", motor_speed_left);
			fputc(',', f1);

			fputs("\r\n", f1);

			usleep(TS);
		}
		
	}

	fclose(f1);

	return NULL;
}

/*******************************************************************************
* int main() 
*
*******************************************************************************/
int main(){
	// always initialize cape library first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to initialize rc_initialize(), are you root?\n");
		return -1;
	}

	//set cpu freq to max performance
	rc_set_cpu_freq(FREQ_1000MHZ);

	//Sprite: Added motor initialization, pretty sure it has to be done
	printf("initializing motors...\n");
	mb_initialize_motors();
	printf("resetting encoders...\n");
	rc_set_encoder_pos(1, 0);
	rc_set_encoder_pos(2, 0);

	// start data thread to take data
	printf("starting data thread... \n");
	pthread_t  data_thread;
	pthread_create(&data_thread, NULL, data_loop, (void*) NULL);

	// done initializing so set state to RUNNING
	rc_set_state(RUNNING);




	//TODO: run your motor tests here while the data thread takes data
	//Sprite: run right and left motors with 50% duty cycle.
	while(rc_get_state()!=EXITING){
		// handle other states
		if(rc_get_state()==RUNNING){
			//run right and left forward for 2s
			mb_set_motor(RIGHT_MOTOR, DUTY_CYCLE);
			mb_set_motor(LEFT_MOTOR, DUTY_CYCLE);
			rc_nanosleep(2E9);
			//Pause for 2s
			mb_set_motor(RIGHT_MOTOR, 0.0);
			mb_set_motor(LEFT_MOTOR, 0.0);
			rc_nanosleep(2E9);
			//run right and left backwards for 2s
			mb_set_motor(RIGHT_MOTOR, -DUTY_CYCLE);
			mb_set_motor(LEFT_MOTOR, -DUTY_CYCLE);
			rc_nanosleep(2E9);

			//stop motors for 3s
			mb_set_motor(LEFT_MOTOR, 0.0);
			mb_set_motor(RIGHT_MOTOR, 0.0);
			rc_nanosleep(3E9);
		}
		else if(rc_get_state()==PAUSED){
			// do other things
		}
		// sleep
		usleep(100000);
	}





	rc_set_state(EXITING); // Sprite: this extra line may be redundant. 
	                       // Sprite: Maybe we are not supposed to run the test in a while loop
	// exit cleanly
	mb_disable_motors();
	rc_cleanup();
	 
	return 0;
}