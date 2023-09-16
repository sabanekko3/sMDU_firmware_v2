/*
 * analog_sens.cpp
 *
 *  Created on: Aug 19, 2023
 *      Author: yaa3k
 */

#include "analog_sens.hpp"

void AnalogSens::init(void){
	for(uint8_t i = 0; i < 16; i++){
		u_i_sens_init_val += *u_i_sens;
		w_i_sens_init_val += *w_i_sens;
		HAL_Delay(1);
	}
	u_i_sens_init_val /= 16;
	w_i_sens_init_val /= 16;
}

//#define HPF_ACTIVE

uvw_t AnalogSens::get_i_uvw(void){
	uvw_t i;
	i.u = -(*u_i_sens - u_i_sens_init_val) * i_gain;
	i.w = -(*w_i_sens - w_i_sens_init_val) * i_gain;
	i.v = -i.w-i.u;
	return i;
}

float AnalogSens::get_power_v(void){
	return (float)adc_dma[(int)ADC_data::POWER_V]*v_gain;
}
