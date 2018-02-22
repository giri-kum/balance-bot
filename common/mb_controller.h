#ifndef MB_CONTROLLER_H
#define MB_CONTROLLER_H

#include "mb_pid.h"
#include "mb_defs.h"
#include "mb_structs.h"
#define CFG_PATH "pid.cfg"

int mb_initialize_controller();
int mb_load_controller_config();
int mb_controller_update(mb_state_t* mb_state);
int mb_destroy_controller();

pid_parameters_t left_pid_params;
pid_parameters_t right_pid_params;

PID_t *right_pid;
PID_t *left_pid;

#endif

