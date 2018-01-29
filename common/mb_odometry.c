/*******************************************************************************
* mb_odometry.c
*
* TODO: Implement these functions to add odometry and dead rekoning 
*
*******************************************************************************/

#include "../balancebot/balancebot.h"

#define DTHETA_THRESH 0.001
// #define XDOT_THRESH	  

void mb_initialize_odometry(mb_odometry_t* mb_odometry, float x, float y, float theta){
	mb_odometry->x = x;
	mb_odometry->y = y;
	mb_odometry->theta = theta;
	mb_odometry->del_x = 0;
	mb_odometry->del_y = 0;
	mb_odometry->del_theta = 0;
}

void mb_update_odometry(mb_odometry_t* mb_odometry, mb_state_t* mb_state){
	float dr, dl;

	dr = (mb_state->right_encoder)*WHEEL_DIAMETER*PI/CPR;
	dl = (mb_state->left_encoder)*WHEEL_DIAMETER*PI/CPR;

	mb_odometry->del_x = (dr+dl)/2.0;
	mb_odometry->del_y = 0;
	mb_odometry->del_theta = (dr-dl)/WHEEL_BASE;

	mb_odometry->x = mb_odometry->x + mb_odometry->del_x;
	mb_odometry->y = mb_odometry->y + mb_odometry->del_y;
	mb_odometry->theta = mb_odometry->theta + mb_odometry->del_theta;

	mb_state->xdot = mb_odometry->del_x/DT;
	mb_state->odometry_thetadot = mb_odometry->del_theta/DT;

}


float mb_clamp_radians(float angle){
    return 0;
}