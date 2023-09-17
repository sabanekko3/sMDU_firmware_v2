/*
 * can.hpp
 *
 *  Created on: 2023/07/15
 *      Author: yaa3k
 */

#ifndef DRIVER_CAN_HPP_
#define DRIVER_CAN_HPP_

#include "../board_data.hpp"
#include "../ring_buffer.hpp"


class CanCom{
private:
	CAN_HandleTypeDef *can;
	const uint32_t rx_fifo;

	RingBuffer<can_frame_t,RingBuffer::buff_size::SIZE16> rx_buff;

public:
	CanCom(CAN_HandleTypeDef *_can,uint32_t _rx_fifo)
	:can(_can),rx_fifo(_rx_fifo){
	}

	typedef struct{
		uint8_t data[8]={0};
		size_t size=0;
		uint32_t id=0;
		bool is_ext_id=false;
		bool is_remote=false;
	}can_frame_t;

	enum class filter_mode{
		only_std,
		only_ext,
		std_and_ext,
	};

	//can tx functions/////////////////////////////
	bool tx(const can_frame_t &tx_data);
	//inline functions
	uint32_t tx_available(void){
		return HAL_CAN_GetTxMailboxesFreeLevel(can);
	}

	//can rx fuctions//////////////////////////////
	uint32_t rx_available(void){
		return rx_buff.get_free_level();
	}
	bool rx(can_frame_t &rx_frame);
	void rx_interrupt_task(void);

	//can filter setting///////////////////////////
	void set_filter_mask(uint32_t id,uint32_t mask,filter_mode mode,bool as_std);
	void set_filter_free(void);
};


#endif /* DRIVER_CAN_HPP_ */
