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

    intialize_queue(0,out_filter_queue,out_queue_length);
    //TODO: initialize your controller here
    //Sprite:
    in_pid = PID_Init(in_pid_params.kp, in_pid_params.ki, in_pid_params.kd, in_pid_params.dFilterHz, SAMPLE_RATE_HZ); //defined in mb_defs.h
    out_pid = PID_Init(out_pid_params.kp, out_pid_params.ki, out_pid_params.kd, out_pid_params.dFilterHz, SAMPLE_RATE_HZ);
    turn_pid = PID_Init(turn_pid_params.kp, turn_pid_params.ki, turn_pid_params.kd, turn_pid_params.dFilterHz, SAMPLE_RATE_HZ);
    position_pid = PID_Init(position_pid_params.kp, position_pid_params.ki, position_pid_params.kd, position_pid_params.dFilterHz, SAMPLE_RATE_HZ);
    heading_pid = PID_Init(heading_pid_params.kp, heading_pid_params.ki, heading_pid_params.kd, heading_pid_params.dFilterHz, SAMPLE_RATE_HZ);
    states = 2;
    waypoint_number = 0;
    sign = 1.0;
    PID_SetOutputLimits(position_pid, -max_fwd, max_fwd);
    PID_SetOutputLimits(heading_pid, -max_turn, max_turn);
    
    PID_SetOutputLimits(out_pid, -PI, PI);
    PID_SetIntegralLimits(out_pid, -PI/10, PI/10);

    PID_SetOutputLimits(turn_pid, -PI/6, PI/6);
    PID_SetIntegralLimits(turn_pid, -PI/10, PI/10);

    out_Filter = rc_empty_filter();
    turn_Filter = rc_empty_filter();

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
    fscanf(file, "%f %f %f %f",
        &position_pid_params.kp,
        &position_pid_params.ki,
        &position_pid_params.kd,
        &position_pid_params.dFilterHz
        );
    fscanf(file, "%f %f %f %f",
        &heading_pid_params.kp,
        &heading_pid_params.ki,
        &heading_pid_params.kd,
        &heading_pid_params.dFilterHz
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
    fscanf(file, "%f",&calibrate_imu);
    fscanf(file, "%d",&use_optitrack);
    fscanf(file, "%d",&competition);
    fscanf(file, "%f %f",&max_fwd, &max_turn);
    fclose(file);

    // Sprite: for debugging purposes
    printf("in_pid %f, %f, %f, %f\n", in_pid_params.kp, in_pid_params.ki, in_pid_params.kd, in_pid_params.dFilterHz);
    printf("out_pid %f, %f, %f, %f\n", out_pid_params.kp, out_pid_params.ki, out_pid_params.kd, out_pid_params.dFilterHz);
    printf("turn_pid %f, %f, %f, %f\n", turn_pid_params.kp, turn_pid_params.ki, turn_pid_params.kd, turn_pid_params.dFilterHz);
    printf("position_pid %f, %f, %f, %f\n", position_pid_params.kp, position_pid_params.ki, position_pid_params.kd, position_pid_params.dFilterHz);
    printf("heading_pid %f, %f, %f, %f\n", heading_pid_params.kp, heading_pid_params.ki, heading_pid_params.kd, heading_pid_params.dFilterHz);
    printf("compensator %f\n", compensator);
    printf("out queue length %d\n", out_queue_length);
    printf("turn queue length %d\n", turn_queue_length);
    printf("out_FilterHz =  %f\n", out_FilterHz);
    printf("turn_FilterHz =  %f\n", turn_FilterHz);
    printf("outerloop_rate =  %f\n", SAMPLE_RATE_HZ / (float) outerloop_rate);
    printf("sensor_scheme =  %d\n", sensor_scheme);
    printf("gyrodometry_threshold =  %f\n", gyrodometry_threshold);
    printf("calibrate_imu =  %f\n", calibrate_imu);
    printf("use_optitrack =  %d\n", use_optitrack);
    printf("competition =  %d\n", competition);
    printf("max_fwd =  %f, max_turn = %f \n", max_fwd,max_turn);
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
    if(mb_setpoints->manual_ctl!=1) // autonomous control
    {
        if(waypoint_number >= 0)
            statemachine(mb_state,mb_setpoints);
    }
    
    balance(mb_state, mb_setpoints->fwd_velocity, mb_setpoints->turn_velocity);
return 0; 
}

void statemachine(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints)
{
    if(states == 2)
    {
        if(waypoint_number == total_waypoints)
            {
                if(competition == 4)  
                    waypoint_number = -2;
                else if (competition == 1)
                    waypoint_number = 0;
                else
                    {
                        waypoint_number = -1;
                        mb_setpoints->fwd_velocity = 0;
                        mb_setpoints->turn_velocity = 0;
                    }
            }
        else
            {
                mb_setpoints->position[0] = mb_waypoints[waypoint_number].position[0];
                mb_setpoints->position[1] = mb_waypoints[waypoint_number].position[1];
                waypoint_number++;
                printf("\n New waypoint set! waypoint_number %d :%f %f \n", waypoint_number, mb_setpoints->position[0], mb_setpoints->position[1]);
            }   
        states = 0;
    }
    else
        states = get_rtr_state(mb_state,mb_setpoints);
    
}

int get_rtr_state(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints)
{
    mb_setpoints->heading = atan2((mb_setpoints->position[1]-mb_state->odometry_y),(mb_setpoints->position[0]-mb_state->odometry_x));
    mb_setpoints->distance = sqrt(pow(mb_setpoints->position[0]-mb_state->odometry_x,2) + pow(mb_setpoints->position[1]-mb_state->odometry_y,2));
    if (states == 0)
    {
        if(mb_setpoints->heading < 0 && ((mb_setpoints->heading - mb_state->theta) < -PI/2 || (mb_setpoints->heading - mb_state->theta) > PI/2))
        {
            mb_setpoints->heading = mb_setpoints->heading + PI;
            sign = -1.0;
            mb_setpoints->distance = sign*mb_setpoints->distance;
        }
        else if (mb_setpoints->heading >= 0 && ((mb_setpoints->heading - mb_state->theta) < -PI/2 || (mb_setpoints->heading - mb_state->theta) > PI/2))
        {
            mb_setpoints->heading = mb_setpoints->heading - PI;
            sign = -1.0;
            mb_setpoints->distance = sign*mb_setpoints->distance;
        }
        else
            sign = 1.0;
    }
    else if (states == 1)
    {
        mb_setpoints->distance = sign*mb_setpoints->distance;
    }

    
    switch(states)
    {
        case 0: // from idle state to rotation 1 state
                {
                 heading_controller(mb_state,mb_setpoints);
                 mb_setpoints->fwd_velocity = 0;
                 if(mb_state->error_heading < tolerance_angle && mb_state->error_heading > -tolerance_angle)
                    return 1;
                 else
                    return 0;
                 } 
        case 1: // from idle rotation 1 to translation state
                {
                 position_controller(mb_state,mb_setpoints);
                 //heading_controller(mb_state,mb_setpoints);
                 mb_setpoints->turn_velocity = 0;
                 if(mb_state->error_position < tolerance_position && mb_state->error_position > -tolerance_position)
                    return 2;
                 else
                    return 1;
                }
        case 2: // from translation to rotation 2 state
                {

                return 0;
                }
        case 3: // from rotation 2 to idle state
                {

                return 0;
                }
        default: return 0;
    }
}


void position_controller(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints)
{
    float error_position;
    int position_true = 0;
    error_position = mb_setpoints->distance;
    if (error_position < -0.3||error_position > 0.3)
    {
        mb_setpoints->fwd_velocity = PID_Compute(position_pid, error_position, position_true);
    }
    else
    {
        mb_setpoints->fwd_velocity = 0.5* PID_Compute(position_pid, error_position, position_true);
    }
    
    mb_state->position_pid_p = mb_setpoints->fwd_velocity;
    mb_state->error_position = error_position;
}

void heading_controller(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints)
{
    float error_heading;
    int heading_true = 0;
    error_heading = mb_setpoints->heading - mb_state->theta;
    error_heading = rc_march_filter(&heading_pid->dFilter, error_heading);
    mb_setpoints->turn_velocity = PID_Compute(heading_pid, error_heading, heading_true);
    mb_state->heading_pid_p = mb_setpoints->turn_velocity;
    mb_state->error_heading = error_heading;
}

void balance(mb_state_t* mb_state, float fwd_velocity, float turn_velocity)
{
    float error, error_out, error_turn, output, desired_alpha, turn_output,left_u,right_u;
    int in_true, out_true, turn_true;
    in_true = 1;
    out_true = 0;
    turn_true = 0;
    error_out = fwd_velocity - mb_state->xdot;
    error_turn = turn_velocity - mb_state->thetadot;
    error_out = rc_march_filter(&(out_Filter), error_out); 
    error_turn = rc_march_filter(&(turn_Filter), error_turn); 

    // Sprite: Compute input (desired-alpha) for the inner loop
    desired_alpha = mb_state->equilibrium_point - PID_Compute(out_pid, error_out, in_true);

    if (((mb_state->count % (outerloop_rate+1)) == 0) || ((mb_state->count % (outerloop_rate+1)) == outerloop_rate)){
        mb_state->desired_alpha = desired_alpha;
    }

    // Sprite: Compute error for the inner loop
    error = mb_state->desired_alpha - mb_state->alpha;

    // Sprite: Compute PID output for the motor
    output = compensate(PID_Compute(in_pid, error, out_true));
    turn_output = PID_Compute(turn_pid, error_turn, turn_true);

    if (((mb_state->count % (outerloop_rate+1)) == 0) || ((mb_state->count % (outerloop_rate+1)) == outerloop_rate)){
        mb_state->turn_output = turn_output;
        mb_state->count = 0;
    }

    left_u = output+ mb_state->turn_output;
    right_u = output-mb_state->turn_output;
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