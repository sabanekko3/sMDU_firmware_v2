/*
 * motor_measure.cpp
 *
 *  Created on: 2023/09/03
 *      Author: yaa3k
 */

#include "motor_measure.hpp"

float MOTOR_measure::measure_R(float duty){

	driver.out({duty,0,0});
	HAL_Delay(100);

	float R = 0.0f;
	for(int i = 0; i < 16; i++){
		R += (analog.get_power_v()*(duty*0.5)) /analog.get_i_uvw().u;
		HAL_Delay(1);
	}

	R *= (2.0/3.0) / 16.0;
	driver.out({0,0,0});
	return R;
}

float MOTOR_measure::measure_L(float R,float duty){
	driver.out({0,0,0});
	HAL_Delay(100);

	HAL_TIM_Base_Start(tim);

	R *= 3/2;
	float i_th = (analog.get_power_v()*duty*0.5)/R * (1-1/M_E);

	//limit:50ms
	uint16_t timer_count_limit = 50 * 1000;
	uint16_t timer_count = 0;

	driver.out({duty,0,0});
	__HAL_TIM_SET_COUNTER(tim,0);

	while(analog.get_i_uvw().u < i_th){
		timer_count = __HAL_TIM_GET_COUNTER(tim);

		if(timer_count_limit < timer_count){
			return false;
		}
	}

	driver.out({0,0,0});
	float L = (float)timer_count * 1.0e-06 * R;

	return L*2/3;
}


