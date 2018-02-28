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
    out_pid = PID_Init(out_pid_params.kp, out_pid_params.ki, out_pid_params.kd, out_pid_params.dFilterHz, SAMPLE_RATE_HZ);
    PID_SetOutputLimits(out_pid, -0.3, 0.3);

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

    fscanf(file, "%f %f %f %f",
        &right_pid_params.kp,
        &right_pid_params.ki,
        &right_pid_params.kd,
        &right_pid_params.dFilterHz
        );

    fscanf(file, "%f %f %f %f",
        &out_pid_params.kp,
        &out_pid_params.kd,
        &out_pid_params.ki,
        &out_pid_params.dFilterHz
        );

    fclose(file);

    // Sprite: for debugging purposes
    printf("left_pid %f, %f, %f, %f\n", left_pid_params.kp, left_pid_params.ki, left_pid_params.kd, left_pid_params.dFilterHz);
    printf("out_pid %f, %f, %f, %f\n", out_pid_params.kp, out_pid_params.ki, out_pid_params.kd, out_pid_params.dFilterHz);

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

    float error, error_out, desired_alpha;
    
    error_out = 0.0 - mb_state->xdot;

    desired_alpha = PI - PID_Compute(out_pid, error_out);

    if (desired_alpha > PI){
        desired_alpha = desired_alpha - TWO_PI;
    }

    // Sprite: hard coded the upright position, will delete later
    desired_alpha = 3.12;

    if (desired_alpha < 0){
        if (mb_state->alpha < 0){
            error = desired_alpha - (mb_state->alpha);
        }
        else{
            error = TWO_PI+desired_alpha - (mb_state->alpha); // in radian
        }
    }
    else{
        if (mb_state->alpha < 0){
            error = desired_alpha - TWO_PI - (mb_state->alpha);
        }
        else{
            error = desired_alpha - (mb_state->alpha); // in radian
        }
    }

   
    
    mb_state->right_cmd = ((float) ENC_1_POL)*SPEED_RATIO*(PID_Compute(right_pid, error));
    mb_state->left_cmd = ((float) ENC_2_POL)*(PID_Compute(left_pid, error));
    
    // Sprite: for debugging purposes
    mb_state->right_pid_p = right_pid->pTerm;
    mb_state->left_pid_p = left_pid->pTerm;
    mb_state->right_pid_i = right_pid->iTerm;
    mb_state->left_pid_i = left_pid->iTerm;
    mb_state->right_pid_d = right_pid->dTerm;
    mb_state->left_pid_d = left_pid->dTerm;
    mb_state->error = error;
    mb_state->desired_alpha = desired_alpha;
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
