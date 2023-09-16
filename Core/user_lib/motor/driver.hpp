/*
 * driver.hpp
 *
 *  Created on: Mar 27, 2023
 *      Author: yaa3k
 */

#ifndef INC_DRIVER_HPP_
#define INC_DRIVER_HPP_

#include "../board_data.hpp"
#include "pwm.hpp"
#include "motor_math.hpp"

typedef struct driver_element{
	//MOTOR PWM parameters
	TIM_HandleTypeDef *PWM_timer;
	uint32_t PWM_ch[3];
	uint32_t PWM_timer_period;

	//MOTOR EN pin
	GPIO_TypeDef *EN_port;
	uint32_t EN_ch;
}driver_element_t;

class Driver{
private:
	PWM pwm_u;
	PWM pwm_v;
	PWM pwm_w;

	GPIO_TypeDef *en_port;
	const uint16_t en_pin;
public:
	Driver(driver_element_t el):
		pwm_u(el.PWM_timer,el.PWM_ch[0],el.PWM_timer_period,-1,1,true),
		pwm_v(el.PWM_timer,el.PWM_ch[1],el.PWM_timer_period,-1,1,true),
		pwm_w(el.PWM_timer,el.PWM_ch[2],el.PWM_timer_period,-1,1,true),
		en_port(el.EN_port),
		en_pin(el.EN_ch){}

	void out(int angle,float power);
	void out(uvw_t uvw);

	void pwms_start(void);
	void pwms_stop(void);
};



#endif /* INC_DRIVER_HPP_ */
