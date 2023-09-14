/*
 * motor_math.hpp
 *
 *  Created on: 2023/04/03
 *      Author: yaa3k
 */

#ifndef DRIVER_MOTOR_MATH_HPP_
#define DRIVER_MOTOR_MATH_HPP_

#include "board_data.hpp"

//angle:電気角などを表すのに使用

#define TABLE_SIZE 1024
#define TABLE_SIZE_2 512
#define TABLE_SIZE_4 256
#define PHASE_90 256
#define PHASE_120 341

typedef struct dq{
	float d;
	float q;
}dq_t;
typedef struct ab{
	float a;
	float b;
}ab_t;
typedef struct uvw{
	float u;
	float v;
	float w;
}uvw_t;
typedef struct sincos{
	float sin;
	float cos;
}sincos_t;


#define ARM_MATH_CM4
//#define __FPU_PRESENT
#include "arm_const_structs.h"
#include "arm_math.h"

//motor_math/////////////////////////////////////////////////////////////
class motor_math{
private:
	static float table[TABLE_SIZE_4+1];

	static constexpr float ANGLE_TO_RAD = (2*M_PI)/TABLE_SIZE;
	static constexpr float RAD_TO_ANGLE = TABLE_SIZE/(2*M_PI);
public:
	motor_math(void);

	static void dq_from_uvw(uvw_t input,uint16_t deg_e,dq_t *out);
	static void dq_from_uvw(uvw_t input,sincos_t param,dq_t *out);
	static void uvw_from_dq(dq_t input,uint16_t deg_e,uvw_t *out);
	static void uvw_from_dq(dq_t input,sincos_t param,uvw_t *out);

	static float fast_atan2_rad(const float _x,const float _y);
	static float sin_table(int angle);

	//inline functions
	static float cos_table(int angle){
		return sin_table(angle + PHASE_90);
	}
	static float sin_table(float rad){
		return sin_table(rad_to_angle(rad));
	}
	static float cos_table(float rad){
		return cos_table(rad_to_angle(rad));
	}
	static int rad_to_angle(float rad){
		return rad * RAD_TO_ANGLE;
	}
	static float angle_to_rad(int angle){
		return 	(float)angle * ANGLE_TO_RAD;
	}
	static uint16_t fast_atan2_angle(float _x,float _y){
		float rad = fast_atan2_rad(_x,_y);
		if(rad < 0) rad = 2*M_PI + rad;
		return rad * RAD_TO_ANGLE;
	}

	template<typename T>
	static T clamp(T data, T min, T max){
		data = data > max ? max : data;
		data = data < min ? min : data;
		return data;
	}
};

//PID/////////////////////////////////////////////////////////////
class PID{
private:
	motor_math math;
	const float pid_freq;
	float kp = 0;
	float ki = 0;
	float kd = 0;
	float k_anti_windup = 0;
	float error = 0;
	float error_sum = 0;
	float old_error = 0;

	//output limit
	float limit_min = -FLT_MAX;
	float limit_max = FLT_MAX;
public:
	PID(float _pid_freq):pid_freq(_pid_freq){}
	float calc(float target,float feedback);

	//inline functions
	void set_gain(float _kp,float _ki,float _kd){
		kp = _kp;

		ki = _ki/pid_freq;
		kd = _kd*pid_freq;

		k_anti_windup = 1/kp;
	}
	void set_limit(float min,float max){
		limit_min = min;
		limit_max = max;
	}
	void reset(void){
		error = 0;
		error_sum = 0;
		old_error = 0;
	}
};

//filters/////////////////////////////////////////////////////////////
class LPF{
private:
	float data = 0;
	const float k = 0;
public:
	//set gain 0~100
	LPF(float _k):k(_k){}
	float calc(float input){
		data = input*k+(1.0f-k)*data;
		return data;
	}
	void reset(void){
		data = 0;
	}
};

template<class T>
class HPF{
private:
	T data = 0;
	const T k = 0;
public:
	HPF(T _k):k(_k){}

	//inline functions
	T calc(T input){
		data = input*k + (100-k)*data;
		return input - data;
	}
	void reset(void){
		data = 0;
	}
};


#endif /* DRIVER_MOTOR_MATH_HPP_ */
