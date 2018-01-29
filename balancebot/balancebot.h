#ifndef BB_H
#define BB_H

// usefulincludes is a collection of common system includes
#include <rc_usefulincludes.h> 
// main roboticscape API header
#include <roboticscape.h>
#include <stdlib.h>
#include <string.h>
#include "../common/mb_defs.h"
#include "../common/mb_structs.h"
#include "../common/mb_motors.h"
#include "../common/mb_pid.h"
#include "../common/mb_controller.h"
#include "../common/mb_odometry.h"

//Includes for LCM and MOCAP
#include <lcm/lcm.h>
#include "../lcmtypes/balancebot_gate_t.h"
#include "../lcmtypes/pose_xyt_t.h"
#include "../lcmtypes/balancebot_msg_t.h"


// global variables
rc_imu_data_t imu_data;
pthread_mutex_t state_mutex;
mb_state_t mb_state;
mb_setpoints_t mb_setpoints;
mb_odometry_t mb_odometry;

float prev_imu_theta;
uint64_t current_time, start_time;
FILE * f;


//lcm variables
lcm_t * lcm;


// functions
void balancebot_controller();
float wrap_angle(float value);
//threads
void* setpoint_control_loop(void* ptr);
void* printf_loop(void* ptr);

//LCM thread
void* lcm_subscribe_loop(void* ptr);

//LCM handler function
void optitrack_message_handler(const lcm_recv_buf_t* rbuf,
                               const char* channel,
                               const balancebot_msg_t* msg,
                               void* userdata);

int mb_load_setpoint_config();
#endif