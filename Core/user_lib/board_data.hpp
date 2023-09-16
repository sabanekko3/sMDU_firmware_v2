/*
 * board_data.hpp
 *
 *  Created on: 2023/04/03
 *      Author: yaa3k
 */

#ifndef DRIVER_BOARD_DATA_HPP_
#define DRIVER_BOARD_DATA_HPP_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <array>


#include "../inc/main.h"

#define sMDU_V2

#define TIM7_INT
#define TIM7_FRQ 10000

enum class PHASE{
	U,
	V,
	W
};

enum class REG_ID{
	CAN_ID,             //r (set via UART)
	MOTOR_TYPE,         //r/w
	ENC_TYPE,           //r/w
	BATT_VOL,            //r
	FOC_GAIN_PI,         //r/w
	FOC_GAIN_D,         //r/w
	DQ_TARGET,
	DQ_I,
	SPD_GAIN_PI,         //r/w
	SPD_GAIN_D,         //r/w
	SPD_TARGET,
	SPD,
	POS,
	POS_SUM,              //r
	I_LIMIT,            //r/w
};

typedef struct{
	float kp;
	float ki;
	float kd;
}pid_gain_t;

enum class MOTOR_type{
	NOP,
	DC,
	FOC,
	BLDC_FORCED_COMM,
	FOC_SENSORLESS,
};

enum class ENC_type{
	AS5600,
	AB_LINER,
	n,
};


#endif /* DRIVER_BOARD_DATA_HPP_ */
