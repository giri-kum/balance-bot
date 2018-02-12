/*******************************************************************************
* mb_odometry.c
*
* TODO: Implement these functions to add odometry and dead rekoning 
*
*******************************************************************************/

#include "../balancebot/balancebot.h"

#define DTHETA_THRESH 0.001

void mb_initialize_odometry(mb_odometry_t* mb_odometry, float x, float y, float theta){
}

void mb_update_odometry(mb_odometry_t* mb_odometry, mb_state_t* mb_state){
}


float mb_clamp_radians(float angle){
    return 0;
}