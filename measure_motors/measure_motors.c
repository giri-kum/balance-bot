/*******************************************************************************
* measure_motors.c
*
* 
*******************************************************************************/
#include "../balancebot/balancebot.h"

FILE * f1;

void* data_loop(void* ptr){
	f1 = fopen("motor.dat", "w");
	while(1){
	//TODO: take your data here
	}

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

	// start data thread to take data
	printf("starting data thread... \n");
	pthread_t  data_thread;
	pthread_create(&data_thread, NULL, data_loop, (void*) NULL);

	// done initializing so set state to RUNNING
	rc_set_state(RUNNING);

	//TODO: run your motor tests here while the data thread takes data

	rc_set_state(EXITING);
	
	// exit cleanly
	mb_disable_motors();
	rc_cleanup();
	 
	return 0;
}