/*
 * table.hpp
 *
 *  Created on: Mar 18, 2023
 *      Author: yaa3k
 */

#ifndef INC_PWM_HPP_
#define INC_PWM_HPP_

#include "board_data.hpp"

class PWM{
private:
	TIM_HandleTypeDef *tim;
	const uint32_t ch;
	const uint32_t tim_period;
	const float min;
	const float max;
	const bool polarity_inv;
	float diff_inv;

public:
	PWM(TIM_HandleTypeDef *_tim,uint32_t _ch,uint32_t _tim_period,float _min,float _max,bool _polarity_inv)
		: tim(_tim),ch(_ch),tim_period(_tim_period),min(_min),max(_max),polarity_inv(_polarity_inv){
		diff_inv = 1/(max - min);
	}

	void out(float val);//-1~1

	//inline functions
	uint32_t get_compare_val(void){
		return __HAL_TIM_GET_COMPARE(tim, ch);
	}
	void start(void){
		HAL_TIM_PWM_Start(tim, ch);
		__HAL_TIM_SET_COMPARE(tim, ch,0);
	}
	void stop(void){
		HAL_TIM_PWM_Stop(tim, ch);
		__HAL_TIM_SET_COMPARE(tim, ch,0);
	}
};


#endif


