/*
 * encoder.hpp
 *
 *  Created on: 2023/04/10
 *      Author: yaa3k
 */

#ifndef DRIVER_ENCODER_HPP_
#define DRIVER_ENCODER_HPP_

#include "board_data.hpp"
#include "motor_math.hpp"
#include "analog_sens.hpp"



class ENCODER
{
public:
	//for encoder initialize
    virtual void init(void) = 0;
    virtual void set_motor_pole(uint32_t pole) = 0;
    virtual void calibrate(const int angle) = 0;
    virtual void calibrate_finish(void) = 0;

    virtual bool new_data_is_available(void) = 0;

    virtual void read_start(void) = 0;
    virtual void read_completion_task(void) = 0;

    //get data
    //e_angle:電気角
    virtual uint16_t get_e_angle(void) = 0;
    virtual int32_t get_e_angle_sum(void) = 0;
    virtual sincos_t get_e_sincos(void) = 0;

    virtual ENC_type get_type(void) = 0;
};




//AS5600//////////////////////////////////////////////////////////////////////////////
class AS5600 : public ENCODER{
private:
	constexpr static ENC_type type = ENC_type::AS5600;
	I2C_HandleTypeDef *i2c;

	motor_math math;

	static constexpr uint16_t as5600_id = 0x36;
	static constexpr uint16_t resolution = 0xFFF;

	uint32_t motor_pole = 1;

	bool data_is_new = false;
	uint8_t enc_val[2] = {0};

	int turn_count = 0;
	uint16_t angle_old = 0;

	int16_t origin = 0;

	//for search origin
	int32_t calibration_count = 0;
	int32_t origin_search_sum = 0;

	int32_t check_turn(uint16_t);
	uint16_t get_raw(void){
		return (enc_val[0]<<8)|enc_val[1];
	}
public:
	AS5600(I2C_HandleTypeDef *_i2c)
			:i2c(_i2c){}

	void init(void) override;
	void calibrate(const int angle) override;
	void calibrate_finish(void) override;
	void set_motor_pole(uint32_t pole) override{
		motor_pole = pole;
	}

	bool new_data_is_available(void) override{
		return data_is_new;
	}


	void read_start(void) override ;
	void read_completion_task(void) override;

	//e_angle:電気角
	uint16_t get_e_angle(void) override;
	int32_t get_e_angle_sum(void) override;

	sincos_t get_e_sincos(void) override;

	ENC_type get_type(void)override{
		return type;
	}
};

//AB LINER HALL SENSOR(for robotmaster)///////////////////////////////////////////////
enum class HALL_SENS{
	H1,
	H2
};

class AB_LINER : public ENCODER{
private:
	ENC_type type = ENC_type::AB_LINER;
	ANALOG_SENS &analog;

	motor_math math;

	//parametor for get_sincos
	float raw_to_regular[2]={0};
	int16_t move_to_centor[2]={0};

	int32_t turn_count = 0;
	uint32_t enc_phase_log = 0;

	//for calibrate
	uint16_t max[2] = {0,0};
	uint16_t min[2] = {6000,6000};

	bool data_is_new = false;

	//inline functions
	uint16_t get_raw(HALL_SENS sens){
		return analog.get_raw(sens);
	}
	int32_t check_turn(sincos_t sincos_data);
public:
	AB_LINER(ANALOG_SENS &_analog):analog(_analog){}

	void init(void) override;
	void calibrate(const int angle) override;
	void calibrate_finish(void) override;

	void set_motor_pole(uint32_t pole) override{
		//nop
	}

	void read_start(void) override ;
	void read_completion_task(void) override;

	bool new_data_is_available(void) override{
		return data_is_new;
	}

	//e_angle:電気角
	uint16_t get_e_angle(void) override;
	int32_t get_e_angle_sum(void) override;
	sincos_t get_e_sincos(void) override;

	ENC_type get_type(void)override{
		return type;
	}
};




#endif /* DRIVER_ENCODER_HPP_ */
