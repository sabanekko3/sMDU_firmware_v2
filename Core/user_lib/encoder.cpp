/*
 * encoder.cpp
 *
 *  Created on: 2023/04/11
 *      Author: yaa3k
 */

#include "encoder.hpp"

///////////////////////////////////////////////////////////////////////////////////////////////////
//AS5600///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
void AS5600::init(void){
	uint8_t txdata = 0x0c;
	HAL_I2C_Master_Transmit(i2c, as5600_id<<1, &txdata, 1, 100);
}
void AS5600::calibrate(const int angle){
	read_start();
	while(!new_data_is_available());

	if(angle == 0){
		if(calibration_count == 0){
			origin = get_raw();
		}else{
			origin_search_sum += get_raw()-origin;
		}
		calibration_count ++;
	}
}
void AS5600::calibrate_finish(void){
	if(calibration_count != 0){
		origin += origin_search_sum / calibration_count;
	}
}

void AS5600::read_start(void){
	data_is_new = false;
	HAL_I2C_Master_Receive_IT(i2c, as5600_id<<1, enc_val, 2);
}

void AS5600::read_completion_task(void){
	turn_count = check_turn(get_raw());
	data_is_new = true;
}

uint16_t AS5600::get_e_angle(void){
	int32_t angle = (get_raw()-origin)&0x3FF;
	return ((angle*motor_pole*0x3FF/resolution)&0x3FF);
}
int32_t AS5600::get_e_angle_sum(void){
	int32_t div_tmp = (get_raw()*motor_pole)/resolution;
	return (turn_count*motor_pole+div_tmp)*1024*motor_pole + get_e_angle();
}

sincos_t AS5600::get_e_sincos(void){
	sincos_t data;
	uint16_t angle = get_e_angle();

	data.sin = math.sin_table(angle);
	data.cos = math.cos_table(angle);
}

//private function
int32_t AS5600::check_turn(uint16_t angle){
	if((angle > (resolution>>2)) && (angle_old > (resolution>>2)*3)){
		turn_count ++;
	}else if((angle > (resolution>>2)*3) && (angle_old > (resolution>>2))){
		turn_count --;
	}
	angle_old = angle;
}
///////////////////////////////////////////////////////////////////////////////////////////////////
//AB LINER HALL SENSOR(for robotmaster)////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
void AB_LINER::init(void){
	data_is_new = true;
}
void AB_LINER::calibrate(const int angle){
	uint16_t h1 = get_raw(HALL_SENS::H2);
	uint16_t h2 = get_raw(HALL_SENS::H2);

	if(max[(int)HALL_SENS::H1] < h1) max[(int)HALL_SENS::H1] = h1;
	if(max[(int)HALL_SENS::H2] < h2) max[(int)HALL_SENS::H2] = h2;

	if(min[(int)HALL_SENS::H1] > h1) min[(int)HALL_SENS::H1] = h1;
	if(min[(int)HALL_SENS::H2] > h2) min[(int)HALL_SENS::H2] = h2;

	raw_to_regular[(int)HALL_SENS::H1] = 2.0f/(float)(max[(int)HALL_SENS::H1]-min[(int)HALL_SENS::H1]);
	move_to_centor[(int)HALL_SENS::H1] = -(max[(int)HALL_SENS::H1]-min[(int)HALL_SENS::H1])/2 - min[(int)HALL_SENS::H1];

	raw_to_regular[(int)HALL_SENS::H2] = 2.0f/(float)(max[(int)HALL_SENS::H2]-min[(int)HALL_SENS::H2]);
	move_to_centor[(int)HALL_SENS::H2] = -(max[(int)HALL_SENS::H2]-min[(int)HALL_SENS::H2])/2 - min[(int)HALL_SENS::H2];
}
void AB_LINER::calibrate_finish(void){
	//nop
}

void AB_LINER::read_start(void){
	turn_count = check_turn(get_e_sincos());
}

void AB_LINER::read_completion_task(void){
	//nop
}

uint16_t AB_LINER::get_e_angle(void){
	sincos_t sincos_data = get_e_sincos();
	return math.fast_atan2_angle(sincos_data.sin,sincos_data.cos);
}
int32_t AB_LINER::get_e_angle_sum(void){
	sincos_t sincos_data = get_e_sincos();
	turn_count = check_turn(sincos_data);
	return (float)turn_count*1024 + math.fast_atan2_angle(sincos_data.sin,sincos_data.cos);
}

sincos_t AB_LINER::get_e_sincos(void){
	sincos_t data;
	data.sin = -(float)(analog.get_raw(ADC_data::RBM_H2)+move_to_centor[(int)HALL_SENS::H2])*raw_to_regular[(int)HALL_SENS::H2];
	data.cos = (float)(analog.get_raw(ADC_data::RBM_H1)+move_to_centor[(int)HALL_SENS::H1])*raw_to_regular[(int)HALL_SENS::H1];
	return data;
}

//private function
int32_t AB_LINER::check_turn(sincos_t sincos_data){
	int enc_phase = 0;
	enc_phase |= (sincos_data.sin >= 0.0f) ? 0b01:0b00;
	enc_phase |= (sincos_data.cos >= 0.0f) ? 0b10:0b00;

	if(enc_phase == 0b10 && enc_phase_log == 0b11){
		turn_count --;
	}else if(enc_phase == 0b11 && enc_phase_log == 0b10){
		turn_count ++;
	}
	enc_phase_log = enc_phase;
}


