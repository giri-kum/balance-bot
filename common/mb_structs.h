#ifndef MB_STRUCTS_H
#define MB_STRUCTS_H
#include "../lcmtypes/balancebot_gate_t.h"
#include "../lcmtypes/pose_xyt_t.h"
#include "../lcmtypes/balancebot_msg_t.h"

typedef struct mb_state mb_state_t;
struct mb_state{
    // raw sensor inputs
    float   alpha;             // body angle (rad)
    float   imu_theta;             // heading  (rad)
    //float   odometry_theta;             // heading  (rad)
    float   theta;
    int     left_encoder;      // left encoder counts since last reading
    int     right_encoder;     // right encoder counts since last reading
    float   thetadot;           
    float   xdot;
    float   odometry_x;
    float   odometry_y;
    int     sensor_scheme;
    //outputs
    float   left_cmd;  //left wheel command [-1..1]
    float   right_cmd; //right wheel command [-1..1]

    balancebot_msg_t* bb_msg;
    float   opti_x;
    float   opti_y;
    float   opti_theta;


    //TODO: Add more variables to this state as needed
    // Sprite: added for debugging purposes
    float   in_pid_p;
    float   out_pid_p;
    float   turn_pid_p;
    float   position_pid_p;
    float   heading_pid_p;
        

    float   in_pid_i;
    float   out_pid_i;
    float   turn_pid_i;
    float   in_pid_d;
    float   out_pid_d;
    float   turn_pid_d;

    float   error;
    float   theta_calc;
    float   imu_deltheta;
    float   odometry_deltheta;
    float   desired_alpha;
    float   equilibrium_point;
    int   count; //Sprite: added count for running outer-loop at a slower rate
    float turn_output; //Sprite: added turn_output because now the velocity turning control loop is slower
    float   error_position;
    float   error_heading;

};

typedef struct mb_setpoints mb_setpoints_t;
struct mb_setpoints{

    float fwd_velocity; // fwd velocity in m/s
    float turn_velocity; // turn velocity in rad/s
    float position[2];
    float distance;
    float heading;
    int manual_ctl;
};

typedef struct mb_waypoints mb_waypoints_t;
struct mb_waypoints{
    float position[2];
    float heading;
};

typedef struct mb_odometry mb_odometry_t;
struct mb_odometry{

    float x;        //x position from initialization in m
    float y;        //y position from initialization in m
    float theta;    //orientation from initialization in rad
    float del_x;
    float del_y;
    float final_deltheta;
};

#endif
