/*
 * analog_sens.hpp
 *
 *  Created on: Aug 19, 2023
 *      Author: yaa3k
 */

#ifndef DRIVER_ANALOG_SENS_HPP_
#define DRIVER_ANALOG_SENS_HPP_

#include "../board_data.hpp"
#include "../motor_math.hpp"

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

typedef struct analog_sens_element{
	uint16_t *u_i_sens;
	uint16_t *w_i_sens;

	uint16_t *battery_v_sens;

	float voltage_gain;
	float current_gain;
}analog_sens_element_t;


class AnalogSens{
private:
	const float i_gain;
	const float v_gain;
	
	uint16_t *u_i_sens;
	uint16_t *w_i_sens;

	uint16_t *battery_v_sens;

	uint16_t u_i_sens_init_val;
	uint16_t w_i_sens_init_val;

	void ADC_DMA_start(ADC_TypeDef *adc);
	void ADC_DMA_stop(ADC_TypeDef *adc);
public:
	AnalogSens(analog_sens_element_t el):
		u_i_sens(el.u_i_sens),
		w_i_sens(el.w_i_sens),
		battery_v_sens(el.battery_v_sens),
		i_gain(el.current_gain),
		v_gain(el.voltage_gain){}

	void init(void);
	uvw_t get_i_uvw(void);

	float get_power_v(void);

	//inline functions
	template<typename T>
	uint16_t get_raw(T d){
		return adc_dma[(int)d];
	}
};


#endif /* DRIVER_ANALOG_SENS_HPP_ */
