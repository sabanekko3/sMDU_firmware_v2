/*
 * motor.cpp
 *
 *  Created on: Jun 27, 2023
 *      Author: yaa3k
 */
#include "motor.hpp"

extern TIM_HandleTypeDef htim17;


void MOTOR::init(uint32_t _motor_pole){
	assert(motor_pole > 0);
	motor_pole = _motor_pole;
	//pwm driver setting
	driver.pwms_start();

	//adc start
	analog.init();
	analog.dma_start();

	//encoder reset
	enc[(int)enc_type]->set_motor_pole(motor_pole);
	enc[(int)enc_type]->init();

	enc_calibration(0.1);

//	motor_R = measure_R(0.5);
//	motor_L = measure_L(motor_R,0.5);
//
//	//printf("R:%f[Ohm],L:%f[mH]\r\n",motor_R,motor_L*1000);
//
//	//gain setting
//	float kp = motor_R/analog.get_power_v();
//	pid_d.set_gain(kp,5,0);
//	pid_q.set_gain(kp,5,0);
//
//	for(int i = 0; i < 8; i++){
//		angle_log[i] = enc.get_e_angle_sum();
//	}
}

void MOTOR::print_debug(void){
	//sincos_t sincos_val = enc.get_e_sincos();
	//printf("%4.3f,%4.3f\r\n",test.d,test.q);
	//printf("%4.3f,%4.3f,%4.3f,%4.3f\r\n",i_dq.d,i_dq.q,v_dq.d,v_dq.q);
	//printf("%4.3f,%4.3f,%4.3f,%5.2f,%4.3f,%4.3f\r\n",i_uvw.u,i_uvw.v,i_uvw.w,adc.get_power_v(),i_dq.d,i_dq.q);
	//printf("%4.3f,%4.3f,%4.3f\r\n",i_uvw.u,i_uvw.v,i_uvw.w);
	//printf("%4.3f,%4.3f,%4.3f,%4.3f\r\n",enc.get_e_sincos().sin,enc.get_e_sincos().cos,math.sin_t(angle_e),math.cos_t(angle_e));
	//for(int i = 0;i<(int)ADC_data::n;i++) printf("%d,",adc.get_raw(i));
	//printf("%d\r\n",enc.get_e_angle_sum());
	//printf("%4.3f\r\n",motor_speed);
	//printf("%4.2f,%4.2f,%4.2f,%4.2f\r\n",v_dq.d,v_dq.q,-motor_speed*motor_L*i_dq.q,motor_speed*(motor_L*i_dq.d+motor_Ke));
	//printf("%d\r\n",enc.get_e_angle());
	//printf("%d,%d,%d\r\n",adc.get_raw(ADC_data::U_I),adc.get_raw(ADC_data::V_I));
	//printf("%4.3f\r\n",i_dq_target.q);
	//printf("%4.3f,%4.3f,%4.3f\r\n",sincos_val.sin,sincos_val.cos,fast_atan2_rad(sincos_val.sin,sincos_val.cos));
	//printf("%4.3f,%4.3f,%d\r\n",sincos_val.sin,sincos_val.cos,angle_e);

	//printf("%4.3f,%4.3f,%4.3f\r\n",rad_log[top&0x7],rad_log[(top+1)&0x7],rad_diff);
	//printf("%d\r\n",__HAL_TIM_GET_COUNTER(&htim17));
}


void MOTOR::control(void){
	LL_GPIO_SetOutputPin(GPIOA,GPIO_PIN_13);

	i_uvw = analog.get_i_uvw();
	float batt_v_inv = 1.0f/analog.get_power_v();
	sincos_t enc_sincos = enc[(int)enc_type]->get_e_sincos();

//	dq_from_uvw(i_uvw,enc_sincos, &i_dq);
//
//	v_dq.d = pid_d.calc(i_dq_target.d,i_dq.d);
//	v_dq.q = pid_q.calc(i_dq_target.q,i_dq.q);
//
//	//anti interference control
////	v_dq.d += -motor_speed*motor_L*i_dq.q;
////	v_dq.q += motor_speed*(motor_L*i_dq.d + motor_Ke);
//
//	pwm_dq.d = v_dq.d*batt_v_inv;
//	pwm_dq.q = v_dq.q*batt_v_inv;
//
//	uvw_from_dq(pwm_dq,enc_sincos, &pwm_uvw);
//	driver.out(pwm_uvw);

	//
	static float _angle = 0;
	driver.out(angle_e,0.05f);
	_angle += 1;
	angle_e = (int)_angle;
	LL_GPIO_ResetOutputPin(GPIOA,GPIO_PIN_13);
}


void MOTOR::enc_calibration(float duty){
	//encoder calibration///////////////
	//move to origin
	for(float f = 0; f<duty*0.2f; f+=0.001f){
		driver.out(0,f);
		HAL_Delay(1);
	}
	driver.out(0,duty);
	HAL_Delay(300);
	enc[(int)enc_type]->calibrate(0);

	//turn foward 360deg
	for(int i = 0; i < TABLE_SIZE_2; i++){
		driver.out(i,duty);
		enc[(int)enc_type]->calibrate(i);
		HAL_Delay(1);
	}
	driver.out(TABLE_SIZE_2,duty);
	HAL_Delay(300);
	enc[(int)enc_type]->calibrate(TABLE_SIZE/2);

	//turn reverse 360deg
	for(int i = TABLE_SIZE_2; i >= 0; i--){
		driver.out(i,duty);
		enc[(int)enc_type]->calibrate(i);
		HAL_Delay(1);
	}
	driver.out(0,duty);
	HAL_Delay(300);
	enc[(int)enc_type]->calibrate(0);

	//turn reverse 360deg
	for(int i = 0; i >= -TABLE_SIZE_2; i--){
		driver.out(i,duty);
		enc[(int)enc_type]->calibrate(i);
		HAL_Delay(1);
	}
	driver.out(-TABLE_SIZE_2,0.1f);
	HAL_Delay(300);
	enc[(int)enc_type]->calibrate(-TABLE_SIZE/2);

	//turn foward 360deg
	for(int i = -TABLE_SIZE_2; i < 0; i++){
		driver.out(i,duty);
		enc[(int)enc_type]->calibrate(i);
		HAL_Delay(1);
	}
	driver.out(0,duty);
	HAL_Delay(300);
	enc[(int)enc_type]->calibrate(0);

	enc[(int)enc_type]->calibrate_finish();

	driver.out(0,0);
}

float MOTOR::measure_R(float duty){

	driver.out({duty,0,0});
	HAL_Delay(100);

	float R = 0.0f;
	for(int i = 0; i < 16; i++){
		R += (analog.get_power_v()*(duty*0.5)) /analog.get_i_uvw().u;
		HAL_Delay(1);
	}

	R *= (2.0/3.0) / 16.0;
	driver.out({0,0,0});
	motor_R = R;
	return motor_R;
}



float MOTOR::measure_L(float R,float duty){
	driver.out({0,0,0});
	HAL_Delay(100);


	R *= 3/2;
	float i_th = (analog.get_power_v()*duty*0.5)/R * (1-1/M_E);

	//limit:50ms
	uint16_t timer_count_limit = 50 * 1000;
	uint16_t timer_count = 0;

	driver.out({duty,0,0});
	__HAL_TIM_SET_COUNTER(&htim17,0);

	while(analog.get_i_uvw().u < i_th){
		timer_count = __HAL_TIM_GET_COUNTER(&htim17);

		if(timer_count_limit < timer_count){
			driver.out({0,0,0});
			return false;
		}
	}

	float L = (float)timer_count * 1.0e-06 * R;

	motor_L = L*2/3;
	return motor_L;
}

float MOTOR::measure_speed_rad(float freq){
	//measure speed
	angle_log[top] = enc[(int)enc_type]->get_e_angle_sum();
	int32_t angle_diff_tmp = angle_log[top] - angle_log[top+1];
	top = (top+1)&0x7;


	if(abs(angle_log[top] - angle_log[(top-1)&0x7]) < TABLE_SIZE_4*3){
		angle_diff = speed_filter.calc(angle_diff_tmp);
	}

	motor_speed_rad = math.angle_to_rad((float)angle_diff*freq*0.125);

	return motor_speed_rad;
}
