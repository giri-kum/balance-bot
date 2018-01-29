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
#define LEFT_MOTOR              2     // id of left motor
#define RIGHT_MOTOR             1     // id of right motor
#define MDIR1                   60    // see schematic for gpio 1.28
#define MDIR2                   48    // see schematic for gpio 1.16
#define MOT_BRAKE_EN            20    // see schematic for gpio 0.20
#define MOT_1_POL               1    // polarity of motor 1
#define MOT_2_POL               1    // polarity of motor 2
#define ENC_1_POL               -1    // polarity of encoder 1
#define ENC_2_POL               1    // polarity of encoder 2
#define MOT_1_CS                0    // analog in of motor 1 current sense
#define MOT_2_CS                1    // analog in of motor 2 current sense
#define GEAR_RATIO              20.4  // gear ratio of motor
#define ENCODER_RES             48  // encoder counts per motor shaft revolution
#define CPR                     979.2 // Sprite: counts per revolution = 20.4*48
#define WHEEL_DIAMETER          0.08 // diameter of wheel in meters
#define WHEEL_BASE              0.2  // wheel separation distance in meters
#define FWD_VEL_SENSITIVITY     0.25   // sensitivity of RC control for moving
#define TURN_VEL_SENSITIVITY    2.5   // sensitivity of RC control for turning
#define SAMPLE_RATE_HZ          100   // main filter and control loop speed
#define DT                      0.01  // 1/sample_rate
#define PRINTF_HZ               10    // rate of print loop
#define RC_CTL_HZ               1    // rate of RC data update
//#define SPEED_RATIO 		1    //	22.45/25.02 // Motor2/Motor1 - where Motor 2 is left motor and Motor 1 is right motor
#endif
