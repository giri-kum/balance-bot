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
    dqueue_length = 30;
    dintialize_queue(0);
    //TODO: initialize your controller here
    //Sprite:
    in_pid = PID_Init(in_pid_params.kp, in_pid_params.ki, in_pid_params.kd, in_pid_params.dFilterHz, SAMPLE_RATE_HZ); //defined in mb_defs.h
    out_pid = PID_Init(out_pid_params.kp, out_pid_params.ki, out_pid_params.kd, out_pid_params.dFilterHz, SAMPLE_RATE_HZ);
    PID_SetOutputLimits(out_pid, -PI, PI);

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
        &in_pid_params.kp,
        &in_pid_params.ki,
        &in_pid_params.kd,
        &in_pid_params.dFilterHz
        );

    fscanf(file, "%f %f %f %f",
        &out_pid_params.kp,
        &out_pid_params.ki,
        &out_pid_params.kd,
        &out_pid_params.dFilterHz
        );

    fscanf(file, "%f",&compensator);

    fclose(file);

    // Sprite: for debugging purposes
    printf("in_pid %f, %f, %f, %f\n", in_pid_params.kp, in_pid_params.ki, in_pid_params.kd, in_pid_params.dFilterHz);
    printf("out_pid %f, %f, %f, %f\n", out_pid_params.kp, out_pid_params.ki, out_pid_params.kd, out_pid_params.dFilterHz);
    printf("compensator %f\n", compensator);

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

int mb_controller_update(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints){
    //TODO: update your controller each timestep, called by 
    // the IMU interrupt function
    // Sprite:added mb_setpoints as one of the argument, fwd_velocity

    // Sprite: initialize local variables
    float error, error_out, desired_alpha, output;
    int in_true;
    in_true = 1;
    // Sprite: Compute error for the outer loop
    error_out = mb_setpoints->fwd_velocity - mb_state->xdot;

    // Sprite: Added filter for error_out (moving average of 30)
    dpush_queue(error_out);
    error_out = daverage_queue();

    // Sprite: Compute input (desired-alpha) for the inner loop
    desired_alpha = mb_state->equilibrium_point - PID_Compute(out_pid, error_out, 0);

    // Sprite: Compute error for the inner loop
    error = desired_alpha - mb_state->alpha;

    // Sprite: Compute PID output for the motor
    output = compensate(PID_Compute(in_pid, error, in_true));
    mb_state->right_cmd = ((float) ENC_1_POL)*SPEED_RATIO*output;
    mb_state->left_cmd = ((float) ENC_2_POL)*output;
    

    // Sprite: for debugging purposes, PID terms for the inner loop
    mb_state->in_pid_p = in_pid->pTerm;
    mb_state->out_pid_p = out_pid->pTerm;
    mb_state->in_pid_i = in_pid->iTerm;
    mb_state->out_pid_i = out_pid->iTerm;
    mb_state->in_pid_d = in_pid->dTerm;
    mb_state->out_pid_d = out_pid->dTerm;
    mb_state->error = error;
    mb_state->desired_alpha = desired_alpha;
    return 0;
}


float compensate(float command)
{
    float result;
if(command < 0)
    {
        if(command>-0.2)
            result = command - compensator;
        else 
            result = command;
        if(result < -1)
            result = -1.0;
    }
else
    {
        if(command<0.2)
            result = command + compensator;
        else 
            result = command;
        if(result > 1)
            result = 1.0;
    }
    return result;
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

void dpush_queue(float value)
{
    int i = 0;
    for(i = 0; i<dqueue_length-1;i++)
        {
            dfilter_queue[i]= dfilter_queue[i+1];
        }
    dfilter_queue[dqueue_length-1] = value;
}
void dintialize_queue(float value)
{
    int i = 0;
    dfilter_queue[0] = value;
    for(i = 0; i<dqueue_length-1;i++)
        {
            dfilter_queue[i+1]= dfilter_queue[i];
        }
}

float daverage_queue()
{
    float sum = 0;
    int i = 0;
    for (i = 0; i < dqueue_length; ++i)
    {
        sum += dfilter_queue[i];
    }
    return sum/dqueue_length;
}