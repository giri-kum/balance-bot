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
	mb_odometry->final_deltheta = 0;
}

void mb_update_odometry(mb_odometry_t* mb_odometry, mb_state_t* mb_state, int sensor_scheme){
	float dr, dl, diff;

	dr = (mb_state->right_encoder)*WHEEL_DIAMETER*PI/CPR;
	dl = (mb_state->left_encoder)*WHEEL_DIAMETER*PI/CPR;

	mb_odometry->del_x = (dr+dl)/2.0;
	mb_odometry->del_y = 0;


	mb_odometry->x = mb_odometry->x + mb_odometry->del_x*cos(mb_odometry->theta) - mb_odometry->del_y*sin(mb_odometry->theta);
	mb_odometry->y = mb_odometry->y + mb_odometry->del_x*sin(mb_odometry->theta) + mb_odometry->del_y*cos(mb_odometry->theta);
	mb_state->odometry_deltheta = (dr-dl)/WHEEL_BASE;
	diff = mb_state->imu_deltheta - mb_state->odometry_deltheta;

	switch (sensor_scheme){
		case 0:
			mb_odometry->final_deltheta = mb_state->odometry_deltheta;
			break;
		case 1:
			mb_odometry->final_deltheta = mb_state->imu_deltheta;
			break;
		case 2:
			if (diff > gyrodometry_threshold || diff < -gyrodometry_threshold){
				mb_odometry->final_deltheta = mb_state->imu_deltheta;
			}
			else{
				mb_odometry->final_deltheta = mb_state->odometry_deltheta;
			}
			break;
	}

	mb_odometry->theta = mb_clamp_radians(mb_odometry->theta + mb_odometry->final_deltheta);

	mb_state->xdot = mb_odometry->del_x/DT;
	mb_state->thetadot = mb_odometry->final_deltheta/DT;
	mb_state->theta = mb_odometry->theta;
	mb_state->odometry_x = mb_odometry->x;
	mb_state->odometry_y = mb_odometry->y;

}


float mb_clamp_radians(float angle){
	// while(angle>PI)
	// 	angle = angle-PI;
	// while(angle<-PI)
	// 	angle = angle+PI;
	angle = fmod(angle+PI,TWO_PI);
	if (angle<0.0){
		angle += TWO_PI;
	}
	return angle-PI;
}