#ifndef MB_STRUCTS_H
#define MB_STRUCTS_H
#include "../lcmtypes/balancebot_gate_t.h"
#include "../lcmtypes/pose_xyt_t.h"
#include "../lcmtypes/balancebot_msg_t.h"

typedef struct mb_state mb_state_t;
struct mb_state{
    // raw sensor inputs
    float   alpha;             // body angle (rad)
    float   theta;             // heading  (rad)
    int     left_encoder;      // left encoder counts since last reading
    int     right_encoder;     // right encoder counts since last reading

    //outputs
    float   left_cmd;  //left wheel command [-1..1]
    float   right_cmd; //right wheel command [-1..1]

    balancebot_msg_t* bb_msg;
    float   opti_x;
    float   opti_y;
    float   opti_theta;


    //TODO: Add more variables to this state as needed
};

typedef struct mb_setpoints mb_setpoints_t;
struct mb_setpoints{

    float fwd_velocity; // fwd velocity in m/s
    float turn_velocity; // turn velocity in rad/s
    int manual_ctl;
};

typedef struct mb_odometry mb_odometry_t;
struct mb_odometry{

    float x;        //x position from initialization in m
    float y;        //y position from initialization in m
    float theta;    //orientation from initialization in rad
};

#endif