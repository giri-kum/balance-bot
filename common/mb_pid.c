/*******************************************************************************
* mb_pid.c
*
* TODO: implement these functions to build a generic PID controller
*       with a derivative term low pass filter
*
*******************************************************************************/

#include "mb_pid.h"

#define MAX_OUTPUT 1.0
#define MIN_OUTPUT -1.0
#define ITERM_MIN -1.0
#define ITERM_MAX 1.0

PID_t * PID_Init(float Kp, float Ki, float Kd, float dFilterHz, float updateHz) {
  // zero initial values
  PID_t *pid =  malloc(sizeof(PID_t));
  // Sprite: 
  pid->kp = Kp;
  pid->ki = Ki;
  pid->kd = Kd;

  pid->pidInput = 0.0;
  pid->pidOutput = 0.0;

  pid->pTerm = 0.0;
  pid->iTerm = 0.0;
  pid->dTerm = 0.0;

  pid->prevInput = 0.0;

  pid->iTermMin = ITERM_MIN;
  pid->iTermMax = ITERM_MAX;
  pid->outputMin = MIN_OUTPUT;
  pid->outputMax = MAX_OUTPUT; 

  pid->dFilter = rc_empty_filter(); // Sprite: rc_filter_t rc_empty_filter()

  pid->dFilterHz = dFilterHz;
  pid->updateHz = updateHz;
 
  float dt, time_constant;
  dt = 1.0/(pid->updateHz);
  time_constant = 1.0/(TWO_PI*(pid->dFilterHz)); // Sprite: TWO_PI is defined in rc_usefulincludes.h, which is included in mb_pid.h

  rc_first_order_lowpass(&(pid->dFilter), dt, time_constant); // Sprite: int rc_first_order_lowpass(rc_filter_t* f, float dt, float time_constant)

  return pid;
}

float PID_Compute(PID_t* pid, float error) {
  // Sprite: set up some local variables for computation
  // Sprite: if needed, later will change dt to be real time
  float dt;
  dt = 1.0/(pid->updateHz);
  //time_constant = 1.0/(TWO_PI*(pid->dFilterHz)); // Sprite: TWO_PI is defined in rc_usefulincludes.h, which is included in mb_pid.h

  // Sprite: assign error to pidInput
  pid->pidInput = error;

  // Sprite: compute pTerm
  pid->pTerm = (pid->kp)*(pid->pidInput);

  // Sprite: compute iTerm
  pid->iTerm = (pid->ki)*((pid->pidInput)*dt + (pid->iTerm));

  if(pid->iTerm > pid->iTermMax){
  	pid->iTerm = pid->iTermMax;
  }
  else if(pid->iTerm < pid->iTermMin){
  	pid->iTerm = pid->iTermMin;
  }

  // Sprite: compute dTerm
  pid->dTerm = (pid->kd)*(((pid->pidInput) - (pid->prevInput))/dt);
  // Sprite: go through the low-pass filter
  // pid->dTerm = rc_march_filter(&(pid->dFilter), pid->dTerm); //Sprite: float rc_march_filter(rc_filter_t* f, float new_input)

  // Sprite: compute PID output
  pid->pidOutput = (pid->pTerm) + (pid->iTerm) + (pid->dTerm);

  if(pid->pidOutput > pid->outputMax){
  	pid->pidOutput = pid->outputMax;
  }
  else if(pid->pidOutput < pid->outputMin){
  	pid->pidOutput = pid->outputMin;
  }
  
  // Sprite: assign the current error to previous input 
  pid->prevInput = error;
  
  return pid->pidOutput;
}

void PID_SetTunings(PID_t* pid, float Kp, float Ki, float Kd) {
  //scale gains by update rate in seconds for proper units
}

void PID_SetOutputLimits(PID_t* pid, float min, float max){
  pid->outputMin = min;
  pid->outputMax = max; 
}

void PID_SetIntegralLimits(PID_t* pid, float min, float max){
  pid->iTermMin = min;
  pid->iTermMax = max;
}

void PID_ResetIntegrator(PID_t* pid){
  pid->iTerm = 0.0;
}

void PID_SetDerivativeFilter(PID_t* pid, float dFilterHz){
}

void PID_SetUpdateRate(PID_t* pid, float updateHz){
}