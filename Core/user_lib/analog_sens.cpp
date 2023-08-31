/*
 * analog_sens.cpp
 *
 *  Created on: Aug 19, 2023
 *      Author: yaa3k
 */

#include "analog_sens.hpp"

void ANALOG_SENS::init(void){
	dma_start();
	HAL_Delay(100);
	for(uint8_t i = 0; i < 16; i++){
		for(uint8_t j = 0; j < (int)ADC_data::n; j++){
			adc_init[j] += adc_dma[j];
		}
		HAL_Delay(1);
	}
	for(uint8_t i = 0; i < (int)ADC_data::n; i++){
		adc_init[i] /= 16;
	}
}

//#define HPF_ACTIVE

uvw_t ANALOG_SENS::get_i_uvw(void){
	uvw_t i;
	i.u = -(adc_dma[(int)ADC_data::U_I] - adc_init[(int)ADC_data::U_I]) * i_gain;
	i.v = -(adc_dma[(int)ADC_data::V_I] - adc_init[(int)ADC_data::V_I]) * i_gain;
	i.w = -(adc_dma[(int)ADC_data::W_I] - adc_init[(int)ADC_data::W_I]) * i_gain;
	i.v = -i.w-i.u;
	//i.v = -i.w-i.u;

	return i;
}

float ANALOG_SENS::get_power_v(void){
	return (float)adc_dma[(int)ADC_data::POWER_V]*v_gain;
}

void ANALOG_SENS::dma_start(void){
	adc1.start(&adc_dma[(int)ADC_data::V_I]);
	adc2.start(&adc_dma[(int)ADC_data::W_I]);
}

void ANALOG_SENS::dma_stop(void){
	adc1.stop();
	adc2.stop();
}

