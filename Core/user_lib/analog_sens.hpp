/*
 * analog_sens.hpp
 *
 *  Created on: Aug 19, 2023
 *      Author: yaa3k
 */

#ifndef DRIVER_ANALOG_SENS_HPP_
#define DRIVER_ANALOG_SENS_HPP_

#include "board_data.hpp"
#include "motor_math.hpp"
#include "adc.hpp"

enum class ADC_data{
	//adc1
	V_I,
	RBM_H2,
	//adc2
	W_I,
	U_I,
	POWER_V,
	RBM_H1,
	n,

	adc1_n = 2,
	adc2_n = 4
};

class ANALOG_SENS{
private:
	ADC_DMA &adc1;
	ADC_DMA &adc2;
	const float i_gain;
	const float v_gain;
	uint16_t adc_init[(int)ADC_data::n] = {0};
	uint16_t adc_dma[(int)ADC_data::n] = {0};

	void ADC_DMA_start(ADC_TypeDef *adc);
	void ADC_DMA_stop(ADC_TypeDef *adc);
public:
	ANALOG_SENS(ADC_DMA &_adc1,ADC_DMA &_adc2,float _i_gain,float _v_gain)
		:adc1(_adc1),adc2(_adc2),i_gain(_i_gain),v_gain(_v_gain){}

	void init(void);
	uvw_t get_i_uvw(void);

	float get_power_v(void);

	void dma_start(void);
	void dma_stop(void);

	//inline functions
	template<typename T>
	uint16_t get_raw(T d){
		return adc_dma[(int)d];
	}
};


#endif /* DRIVER_ANALOG_SENS_HPP_ */
