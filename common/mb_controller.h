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
int dqueue_length;
float dfilter_queue[100];
void dpush_queue(float value);
void dintialize_queue(float value);
float daverage_queue();
PID_t *in_pid;
PID_t *out_pid;


#endif

