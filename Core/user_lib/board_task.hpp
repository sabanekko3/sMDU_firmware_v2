/*
 * board_task.hpp
 *
 *  Created on: 2023/09/14
 *      Author: yaa3k
 */

#ifndef USER_LIB_BOARD_TASK_HPP_
#define USER_LIB_BOARD_TASK_HPP_

#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "motor/motor.hpp"
#include "communicate/can.hpp"

class BoardTask{
public:
	virtual void setup(void) = 0;
	virtual void loop(void) = 0;
};

typedef struct sMDU_element{
	//LED parameters
	TIM_HandleTypeDef *LED_timer;
	uint32_t LED_pwm_ch[3];
	uint32_t LED_timer_period;

	//driver
	driver_element_t driver_el;

	//ADC
	analog_sens_element_t analog_el;

	//ENCODER
	//for as5600
	I2C_HandleTypeDef *i2c;

	//timer for motor measure
	TIM_HandleTypeDef *measure_timer;

	//communicate
	CAN_HandleTypeDef *can;
	uint32_t can_rx_fifo;

}sMDU_element_t;

class sMDU_Task:BoardTask{
private:
	//led
	PWM R;
	PWM G;
	PWM B;

	//pwm
	DRIVER driver;

	ANALOG_SENS analog;

	AS5600 as5600_enc;
	AB_LINER ab_liner_enc;

	BLDC_motor bldc_motor;

	CAN_COM can;
public:
	sMDU_Task(sMDU_element_t el);

	void setup(void)override;
	void loop(void)override;
};


#endif /* USER_LIB_BOARD_TASK_HPP_ */
