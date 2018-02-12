/*******************************************************************************
* mb_defs.h
*
*   defines for your bot
*   You will need to fill this in based on the data sheets, schematics, etc. 
*      and your specific configuration...
* 
*******************************************************************************/
#ifndef MB_DEFS_H
#define MB_DEFS_H

//TODO: You will need to fill in these defines 
// by gathering data from the datasheets
// and making measurements of your system

#define DEFAULT_PWM_FREQ        25000 // period of motor drive pwm
#define LEFT_MOTOR              0     // id of left motor
#define RIGHT_MOTOR             0     // id of right motor
#define MDIR1                   1    // see schematic for gpio #s
#define MDIR2                   1    //    "
#define MOT_BRAKE_EN            1    //    "
#define MOT_1_POL               0    // polarity of motor 1
#define MOT_2_POL               0    // polarity of motor 2
#define ENC_1_POL               0    // polarity of encoder 1
#define ENC_2_POL               0    // polarity of encoder 2
#define MOT_1_CS                0    // analog in of motor 1 current sense
#define MOT_2_CS                0    // analog in of motor 2 current sense
#define GEAR_RATIO              1  // gear ratio of motor
#define ENCODER_RES             1  // encoder counts per motor shaft revolution
#define WHEEL_DIAMETER          1 // diameter of wheel in meters
#define WHEEL_BASE              1  // wheel separation distance in meters
#define FWD_VEL_SENSITIVITY     1   // sensitivity of RC control for moving
#define TURN_VEL_SENSITIVITY    1   // sensitivity of RC control for turning
#define SAMPLE_RATE_HZ          100   // main filter and control loop speed
#define DT                      0.01  // 1/sample_rate
#define PRINTF_HZ               10    // rate of print loop
#define RC_CTL_HZ               25    // rate of RC data update

#endif
