#include "mb_controller.h"

/*******************************************************************************
* int mb_initialize()
*
* this initializes all the PID controllers from the configuration file
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/
int mb_initialize_controller(){

    mb_load_controller_config();
    
    //TODO: initialize your controller here
    //Sprite:
    right_pid = PID_Init(right_pid_params.kp, right_pid_params.ki, right_pid_params.kd, right_pid_params.dFilterHz, SAMPLE_RATE_HZ); //defined in mb_defs.h
    left_pid = PID_Init(left_pid_params.kp, left_pid_params.ki, left_pid_params.kd, left_pid_params.dFilterHz, SAMPLE_RATE_HZ);
    return 0;
}

/*******************************************************************************
* int mb_load_controller_config()
*
* this provides a basic configuration load routine
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/


int mb_load_controller_config(){
    FILE* file = fopen(CFG_PATH, "r");
    if (file == NULL){
        printf("Error opening pid.cfg\n");
    }

    //TODO: You can add to or modify the cfg file as you like
    // This is here as an example

    fscanf(file, "%f %f %f %f", 
        &left_pid_params.kp,
        &left_pid_params.ki,
        &left_pid_params.kd,
        &left_pid_params.dFilterHz
        );

    // Sprite: for debugging purposes
    //printf("%f, %f, %f, %f", left_pid_params.kp, left_pid_params.ki, left_pid_params.kd, left_pid_params.dFilterHz);



    fscanf(file, "%f %f %f %f",
        &right_pid_params.kp,
        &right_pid_params.ki,
        &right_pid_params.kd,
        &right_pid_params.dFilterHz
        );

    fclose(file);
    return 0;
}

/*******************************************************************************
* int mb_controller_update()
* 
* TODO: Write your cascaded PID controller here
* take inputs from the global mb_state
* write outputs to the global mb_state
*
* return 0 on success
*
*******************************************************************************/

int mb_controller_update(mb_state_t* mb_state){
    //TODO: update your controller each timestep, called by 
    // the IMU interrupt function
    // Sprite:
    float error;
    if (mb_state->alpha < 0){
        error = -3.14 - (mb_state->alpha);
    }
    else{
        error = 3.14 - (mb_state->alpha); // in radian
    }
    
    mb_state->right_cmd = ((float) ENC_1_POL)*(PID_Compute(right_pid, error));
    mb_state->left_cmd = ((float) ENC_2_POL)*(PID_Compute(left_pid, error));

    return 0;
}


/*******************************************************************************
* int mb_destroy_controller()
* 
* TODO: Free all resources associated with your controller
*
* return 0 on success
*
*******************************************************************************/

int mb_destroy_controller(){
    //TODO: free the memory for your controller if needed
    return 0;
}