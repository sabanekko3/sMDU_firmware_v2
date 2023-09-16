/*
 * can.hpp
 *
 *  Created on: 2023/07/15
 *      Author: yaa3k
 */

#ifndef DRIVER_CAN_HPP_
#define DRIVER_CAN_HPP_

#include "../board_data.hpp"
#include "communicate.hpp"

#define CAN_RX_BUFF_N 16
#define CAN_RX_BUFF_AND 0xF

enum class filter_mode{
	only_std,
	only_ext,
	std_and_ext,
};

class CanCom : Communication{
private:
	CAN_HandleTypeDef *can;
	const uint32_t rx_fifo;

	data_frame_t rx_buff[CAN_RX_BUFF_N];
	uint32_t head = 0;
	uint32_t tail = 0;

public:
	CanCom(CAN_HandleTypeDef *_can,uint32_t _rx_fifo)
	:can(_can),rx_fifo(_rx_fifo){
	}

	//can tx functions/////////////////////////////
	bool tx(const data_frame_t &tx_data) override;
	//inline functions
	uint32_t tx_available(void) override{
		return HAL_CAN_GetTxMailboxesFreeLevel(can);
	}

	//can rx fuctions//////////////////////////////
	uint32_t rx_available(void) override;
	bool rx(data_frame_t &rx_frame) override;
	void rx_interrupt_task(void);

	//can filter setting///////////////////////////
	void set_filter_mask(uint32_t id,uint32_t mask,filter_mode mode,bool as_std);
	void set_filter_free(void);
};


#endif /* DRIVER_CAN_HPP_ */
