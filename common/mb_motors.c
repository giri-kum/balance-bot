/*******************************************************************************
* mb_motors.c
*
* Control up to 2 DC motordrivers
*
*******************************************************************************/
#include "mb_motors.h"
//#include "../libraries/roboticscape.h" I don't think we even need to include this
// global initialized flag
int mb_motors_initialized = 0;

/*******************************************************************************
* int mb_initialize_motors()
* 
* set up gpio assignments, pwm channels, and make sure motors are left off.
* GPIO exporting must be done previously with simple_init_gpio()
* initialized motors should start disabled
*******************************************************************************/
int mb_initialize_motors(){

    #ifdef DEBUG
    printf("Initializing: PWM\n");
    #endif
    if(rc_pwm_init(1,DEFAULT_PWM_FREQ)){
        printf("ERROR: failed to initialize hrpwm1\n");
        return -1;
    }
    mb_motors_initialized = 1;
    #ifdef DEBUG
    printf("motors initialized...\n");
    #endif

    mb_set_motor_all(0);

    return 0;
}


/*******************************************************************************
* mb_enable_brake()
* 
* turns on the brake pin to enable the brake feature
* returns 0 on success, -1 on failure
*******************************************************************************/
int mb_enable_brake(){
    if(mb_motors_initialized==0){
        printf("ERROR: trying to enable brake before motors have been initialized\n");
        return -1;
    }
    //TODO: enable braking
    
    rc_gpio_set_value_mmap(MOT_BRAKE_EN,HIGH);
    return 0;
}

/*******************************************************************************
* mb_disable_brake()
* 
* turns off the brake pin to disable the brake feature
* returns 0 on success, -1 on failure
*******************************************************************************/
int mb_disable_brake(){
    if(mb_motors_initialized==0){
        printf("ERROR: trying to disable brake before motors have been initialized\n");
        return -1;
    }
    //TODO: disable braking
    rc_gpio_set_value_mmap(MOT_BRAKE_EN,LOW);
    return 0;
}


/*******************************************************************************
* int mb_disable_motors()
* 
* disables PWM output signals
* returns 0 on success
*******************************************************************************/
int mb_disable_motors(){
    if(mb_motors_initialized==0){
        printf("ERROR: trying to disable motors before motors have been initialized\n");
        return -1;
    }
    //TODO: disable motors
    rc_pwm_close(1);
    // Unexports subsys 1 to pull it into low-power state
    return 0;
}


/*******************************************************************************
* int mb_set_motor(int motor, float duty)
* 
* set a motor direction and power
* motor is from 1 to 2, duty is from -1.0 to +1.0
* returns 0 on success
*******************************************************************************/
int mb_set_motor(int motor, float duty){
    
    if(mb_motors_initialized==0)
    {
        printf("ERROR: trying to rc_set_motor_all before they have been initialized\n");
        return -1;
    }

    if ((duty<-1.0 || duty >1.0))
    {
        printf("ERROR: duty cycle out of range\n");
        return -1;
    }

    //TODO: make your motors run
    if(motor == 1)
    {
        if(duty<0)
        {
            rc_pwm_set_duty_mmap(1,'A',-duty);
            rc_gpio_set_value_mmap(MDIR1,HIGH);
            //printf("1A -ve\n");
        }
        else
        {
            rc_pwm_set_duty_mmap(1,'A',duty);
            rc_gpio_set_value_mmap(MDIR1,LOW);
            //printf("1A +ve\n");
        }
        return 0;
    }
        
    else if (motor == 2)
    {
        if(duty<0)
        {
            rc_pwm_set_duty_mmap(1,'B',-duty);
            rc_gpio_set_value_mmap(MDIR2,HIGH);
            //printf("1B -ve\n");
        }
        else
        {
            rc_pwm_set_duty_mmap(1,'B',duty);
            rc_gpio_set_value_mmap(MDIR2,LOW);
            //printf("1B +ve\n"); 
        }
        return 0;
    }
    else
    {
        printf("ERROR: Wrong motor id\n");
        return -1;
    }
       
}

/*******************************************************************************
* int mb_set_motor_all(float duty)
* 
* applies the same duty cycle argument to both motors
*******************************************************************************/
int mb_set_motor_all(float duty){
    if(mb_motors_initialized==0){
        printf("ERROR: trying to rc_set_motor_all before they have been initialized\n");
        return -1;
    }

    mb_set_motor(RIGHT_MOTOR, duty);
    mb_set_motor(LEFT_MOTOR, duty);
    
    return 0;
}
