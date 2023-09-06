/*
 * motor_measure.hpp
 *
 *  Created on: 2023/09/03
 *      Author: yaa3k
 */

#ifndef USER_LIB_MOTOR_MEASURE_HPP_
#define USER_LIB_MOTOR_MEASURE_HPP_

#include "board_data.hpp"
#include "driver.hpp"
#include "pwm.hpp"
#include "motor_math.hpp"
#include "analog_sens.hpp"
#include "encoder.hpp"

class MOTOR_measure{
private:
	TIM_HandleTypeDef *tim;
	DRIVER &driver;
	ANALOG_SENS &analog;
public:
	MOTOR_measure(TIM_HandleTypeDef *_tim,DRIVER &_driver,ANALOG_SENS &_analog)
		:tim(_tim),driver(_driver),analog(_analog){}

	float measure_R(float duty);
	float measure_L(float R,float duty);
};


#endif /* USER_LIB_MOTOR_MEASURE_HPP_ */
