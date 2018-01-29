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
    intialize_queue(0,out_filter_queue,out_queue_length);
    //TODO: initialize your controller here
    //Sprite:
    in_pid = PID_Init(in_pid_params.kp, in_pid_params.ki, in_pid_params.kd, in_pid_params.dFilterHz, SAMPLE_RATE_HZ); //defined in mb_defs.h
    out_pid = PID_Init(out_pid_params.kp, out_pid_params.ki, out_pid_params.kd, out_pid_params.dFilterHz, SAMPLE_RATE_HZ);
    turn_pid = PID_Init(turn_pid_params.kp, turn_pid_params.ki, turn_pid_params.kd, turn_pid_params.dFilterHz, SAMPLE_RATE_HZ);

    PID_SetOutputLimits(out_pid, -PI, PI);
    PID_SetIntegralLimits(out_pid, -PI/10, PI/10);

    out_Filter = rc_empty_filter();

    float dt, time_constant;
    dt = 1.0/(SAMPLE_RATE_HZ);
    time_constant = 1.0/(TWO_PI*(out_FilterHz)); // Sprite: TWO_PI is defined in rc_usefulincludes.h, which is included in mb_pid.h

    rc_first_order_lowpass(&(out_Filter), dt, time_constant); // Sprite: int rc_first_order_lowpass(rc_filter_t* f, float dt, float time_constant)

    time_constant = 1.0/(TWO_PI*(turn_FilterHz)); // Sprite: TWO_PI is defined in rc_usefulincludes.h, which is included in mb_pid.h

    rc_first_order_lowpass(&(turn_Filter), dt, time_constant); // Sprite: int rc_first_order_lowpass(rc_filter_t* f, float dt, float time_constant)

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
    float temp;
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

    fscanf(file, "%f %f %f %f",
        &turn_pid_params.kp,
        &turn_pid_params.ki,
        &turn_pid_params.kd,
        &turn_pid_params.dFilterHz
        );

    fscanf(file, "%f",&compensator);
    fscanf(file, "%f",&temp);
    out_queue_length = (int) temp;
    fscanf(file, "%f",&temp);
    turn_queue_length = (int) temp;
    fscanf(file, "%f",&out_FilterHz);
    fscanf(file, "%f",&turn_FilterHz);
    fscanf(file, "%d",&outerloop_rate);
    fscanf(file, "%d",&sensor_scheme);
    fscanf(file, "%f",&gyrodometry_threshold);
    fclose(file);

    // Sprite: for debugging purposes
    printf("in_pid %f, %f, %f, %f\n", in_pid_params.kp, in_pid_params.ki, in_pid_params.kd, in_pid_params.dFilterHz);
    printf("out_pid %f, %f, %f, %f\n", out_pid_params.kp, out_pid_params.ki, out_pid_params.kd, out_pid_params.dFilterHz);
    printf("turn_pid %f, %f, %f, %f\n", turn_pid_params.kp, turn_pid_params.ki, turn_pid_params.kd, turn_pid_params.dFilterHz);
    printf("compensator %f\n", compensator);
    printf("out queue length %d\n", out_queue_length);
    printf("turn queue length %d\n", turn_queue_length);
    printf("out_FilterHz =  %f\n", out_FilterHz);
    printf("turn_FilterHz =  %f\n", turn_FilterHz);
    printf("outerloop_rate =  %f\n", SAMPLE_RATE_HZ / (float) outerloop_rate);
    printf("sensor_scheme =  %d\n", sensor_scheme);
    printf("gyrodometry_threshold =  %f\n", gyrodometry_threshold);
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
    float error, error_out, error_turn, output, desired_alpha, turn_output,left_u,right_u;
    int in_true, out_true, turn_true;
    in_true = 1;
    out_true = 0;
    turn_true = 0;
    // Sprite: Compute error for the outer loop
    error_out = mb_setpoints->fwd_velocity - mb_state->xdot;
    error_turn = mb_setpoints->turn_velocity - mb_state->thetadot;
    error_out = rc_march_filter(&(out_Filter), error_out); 
    error_turn = rc_march_filter(&(turn_Filter), error_turn); 

    // Sprite: Added filter for error_out (moving average of 30), switched to low-pass filter
 //   push_queue(error_out,out_filter_queue,out_queue_length);
 //   error_out = average_queue(out_filter_queue,out_queue_length);

 //   push_queue(error_turn,turn_filter_queue,turn_queue_length);
 //   error_turn = average_queue(turn_filter_queue,turn_queue_length);

    // Sprite: Compute input (desired-alpha) for the inner loop
    desired_alpha = mb_state->equilibrium_point - PID_Compute(out_pid, error_out, in_true);

    if (((mb_state->count % (outerloop_rate+1)) == 0) || ((mb_state->count % (outerloop_rate+1)) == outerloop_rate)){
        mb_state->desired_alpha = desired_alpha;
        mb_state->count = 0;
    }

    // Sprite: Compute error for the inner loop
    error = mb_state->desired_alpha - mb_state->alpha;

    // Sprite: Compute PID output for the motor
    output = compensate(PID_Compute(in_pid, error, out_true));
    turn_output = PID_Compute(turn_pid, error_turn, turn_true);

    left_u = output+ turn_output;
    right_u = output-turn_output;
    if(left_u > 1)
        left_u = 1.0;
    else if(left_u < -1)
        left_u = -1.0;
    if(right_u > 1)
        right_u = 1.0;
    else if(right_u < -1)
        right_u = -1.0;
    
    mb_state->right_cmd = ((float) ENC_1_POL)*right_u;
    mb_state->left_cmd = ((float) ENC_2_POL)*left_u;

    mb_state->count += 1;

    // Sprite: for debugging purposes, PID terms for the inner loop
    mb_state->in_pid_p = in_pid->pTerm;
    mb_state->out_pid_p = out_pid->pTerm;
    mb_state->turn_pid_p = turn_pid->pTerm;
    mb_state->in_pid_i = in_pid->iTerm;
    mb_state->out_pid_i = out_pid->iTerm;
    mb_state->turn_pid_i = turn_pid->iTerm;
    mb_state->in_pid_d = in_pid->dTerm;
    mb_state->out_pid_d = out_pid->dTerm;
    mb_state->turn_pid_d = turn_pid->dTerm;
    mb_state->error = error;
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

void push_queue(float value,float *filter_queue, int queue_length)
{
    int i = 0;
    for(i = 0; i<queue_length-1;i++)
        {
            filter_queue[i]= filter_queue[i+1];
        }
    filter_queue[queue_length-1] = value;
}
void intialize_queue(float value,float *filter_queue, int queue_length)
{
    int i = 0;
    filter_queue[0] = value;
    for(i = 0; i<queue_length-1;i++)
        {
            filter_queue[i+1]= filter_queue[i];
        }
}

float average_queue(float *filter_queue, int queue_length)
{
    float sum = 0;
    int i = 0;
    for (i = 0; i < queue_length; ++i)
    {
        sum += filter_queue[i];
    }
    return sum/queue_length;
}

float median_queue(float *filter_queue, int queue_length)
{
    float sum = 0;
    int i = 0;
    for (i = 0; i < queue_length; ++i)
    {
        sum += filter_queue[i];
    }
    return sum/queue_length;
}