/*
 * driver.cpp
 *
 *  Created on: Mar 18, 2023
 *      Author: yaa3k
 */

#include "pwm.hpp"

void PWM::out(float val){
	if(val < min || max < val)val  = 0;

	if(polarity_inv) val *= -1;

	__HAL_TIM_SET_COMPARE(tim, ch, (val - min)*diff_inv*tim_period);
}
