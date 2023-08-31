/*
 * adc.hpp
 *
 *  Created on: Apr 7, 2023
 *      Author: yaa3k
 */

#ifndef DRIVER_ADC_HPP_
#define DRIVER_ADC_HPP_

#include "board_data.hpp"
#include "motor_math.hpp"

class ADC_DMA{
private:
	ADC_HandleTypeDef *adc;
	const uint32_t data_n;
	uint16_t *data;

public:
	ADC_DMA(ADC_HandleTypeDef *_adc,uint32_t _data_n):
		adc(_adc),data_n(_data_n){

		for(uint32_t i=0; i<data_n;i++){
			data[i]=0;
		}
	}

	void start(uint16_t *data){
		HAL_ADC_Start_DMA(adc, (uint32_t*)data, data_n);
	}
	void stop(void){
		HAL_ADC_Stop_DMA(adc);
	}
};



#endif /* DRIVER_ADC_HPP_ */
