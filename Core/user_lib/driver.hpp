/*
 * driver.hpp
 *
 *  Created on: Mar 27, 2023
 *      Author: yaa3k
 */

#ifndef INC_DRIVER_HPP_
#define INC_DRIVER_HPP_

#include "board_data.hpp"
#include "pwm.hpp"
#include "motor_math.hpp"

class DRIVER : motor_math{
private:
	PWM& pwm_u;
	PWM& pwm_v;
	PWM& pwm_w;

	GPIO_TypeDef *en_port;
	uint16_t en_pin;

	motor_math math;

public:
	DRIVER(PWM &_pwm_u,PWM &_pwm_v,PWM &_pwm_w,
			GPIO_TypeDef *_en_port,uint16_t _en_pin)
		: pwm_u(_pwm_u),pwm_v(_pwm_v),pwm_w(_pwm_w),
		  	  en_port(_en_port),en_pin(_en_pin){}

	void out(int angle,float power);
	void out(uvw_t uvw);

	void pwms_start(void);
	void pwms_stop(void);
};



#endif /* INC_DRIVER_HPP_ */
