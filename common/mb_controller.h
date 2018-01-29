#ifndef MB_CONTROLLER_H
#define MB_CONTROLLER_H

#include "mb_pid.h"
#include "mb_defs.h"
#include "mb_structs.h"
#define CFG_PATH "pid.cfg"

int mb_initialize_controller();
int mb_load_controller_config();
int mb_controller_update(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints);
int mb_destroy_controller();
float compensate(float command);
float compensator;
pid_parameters_t in_pid_params;
pid_parameters_t out_pid_params;
pid_parameters_t turn_pid_params;
pid_parameters_t position_pid_params;
pid_parameters_t heading_pid_params;
float calibrate_imu;
int out_queue_length;
float out_filter_queue[100];
int turn_queue_length;
float turn_filter_queue[100];
int outerloop_rate;

void intialize_queue(float value,float *filter_queue, int queue_length);
float average_queue(float *filter_queue, int queue_length);
void push_queue(float value,float *filter_queue, int queue_length);
float median_queue(float *filter_queue, int queue_length);
PID_t *in_pid;
PID_t *out_pid;
PID_t *turn_pid;
PID_t *position_pid;
PID_t *heading_pid;

int sensor_scheme;
int use_optitrack;
float gyrodometry_threshold;
float out_FilterHz;
float turn_FilterHz;
rc_filter_t out_Filter;
rc_filter_t turn_Filter; 
  


#endif

