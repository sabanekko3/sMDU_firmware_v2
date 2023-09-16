/*
 * communicate.hpp
 *
 *  Created on: 2023/07/15
 *      Author: yaa3k
 */

#ifndef USER_LIB_COMMUNICATE_HPP_
#define USER_LIB_COMMUNICATE_HPP_

#include "../board_data.hpp"

typedef struct{
	uint8_t data[8]={0};
	size_t size=0;
	uint32_t id=0;
	bool is_ext_id=false;
	bool is_remote=false;

	bool is_free=true;
}data_frame_t;

class Communication{
public:
	virtual bool tx(const data_frame_t &tx_data) = 0;
	virtual uint32_t tx_available(void) = 0;

    virtual bool rx(data_frame_t &rx_frame) = 0;
	virtual uint32_t rx_available(void) = 0;
}

#endif /* USER_LIB_COMMUNICATE_HPP_ */