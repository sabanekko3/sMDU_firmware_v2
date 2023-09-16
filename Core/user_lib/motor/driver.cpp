/*
 * driver.cpp
 *
 *  Created on: Mar 27, 2023
 *      Author: yaa3k
 */
#include "driver.hpp"


void DRIVER::out(int angle,float power){
	pwm_u.out(power*math.cos_table(angle));
	pwm_v.out(power*math.cos_table(angle-PHASE_120));
	pwm_w.out(power*math.cos_table(angle+PHASE_120));
}
void DRIVER::out(uvw_t uvw){
	pwm_u.out(uvw.u);
	pwm_v.out(uvw.v);
	pwm_w.out(uvw.w);
}


void DRIVER::pwms_start(void){
	HAL_GPIO_WritePin(en_port,en_pin,GPIO_PIN_SET);
	pwm_u.start();
	pwm_v.start();
	pwm_w.start();
}
void DRIVER::pwms_stop(void){
	HAL_GPIO_WritePin(en_port,en_pin,GPIO_PIN_RESET);
	pwm_u.stop();
	pwm_v.stop();
	pwm_w.stop();
}
