/*
 * board_taks.cpp
 *
 *  Created on: 2023/09/14
 *      Author: yaa3k
 */

#include "board_task.hpp"

sMDU_task::sMDU_task(sMDU_element_t el):
		//LED
		R(el.LED_timer,el.LED_pwm_ch[0],el.LED_timer_period,0,1,false),
		G(el.LED_timer,el.LED_pwm_ch[1],el.LED_timer_period,0,1,false),
		B(el.LED_timer,el.LED_pwm_ch[2],el.LED_timer_period,0,1,false),
		//driver
		driver(el.driver_el),
		//analog
		analog(el.analog_el),
		//encoders
		as5600_enc(el.i2c),
		ab_liner_enc(analog),
		motor(&driver,&analog),
		//communicate
		can(el.can,el.can_rx_fifo)
		{
}

void sMDU_task::setup(void){

}
