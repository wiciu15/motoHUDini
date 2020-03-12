/*
Most important values device-related are here
 */

#ifndef MOTOHUD_CONFIG_H_
#define MOTOHUD_CONFIG_H_

///////USER PARAMETERS////////////
#define UPDATE_DELAY_MS			500     //screen refresh delay

///////SPEEDOMETER PARAMETERS/////////
#define NUM_OF_PULSES_PER_REV	4        //number of pulses from hall sensor per 1 wheel revolution
#define WHEEL_CIRCUMFERENCE     1.4356	//wheel circumference in meters

////////STEPPER PARAMETERS///////////
//use power-on calibration of dial to tweak these values//
#define STEPPER_MAX_DELAY   		70		//lower value speeds up  the motor and reduce deceleration(50-100)
#define STEPPER_MIN_DELAY   		16		//bigger value slows down the motor (16-30), too low can make it lose steps
#define STEPPER_ACCELERATION		2		//lower value makes motor accelerate slower (1-3)
#define STEPPER_DECCELERATION		300		//lower value makes motor stop faster (100-500)


#endif


