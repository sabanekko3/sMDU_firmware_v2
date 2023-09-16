/*
 * uart.hpp
 *
 *  Created on: 2023/07/15
 *      Author: yaa3k
 */

#ifndef USER_LIB_COM_UART_HPP_
#define USER_LIB_COM_UART_HPP_

#include "../board_data.hpp"
#include "communicate.hpp"

#define CAN_RX_BUFF_N 16
#define CAN_RX_BUFF_AND 0xF

class UartCom : Communication{
private:
    UART_HandleTypeDef *uart;
    
    void encode_COBS(const uint8_t *input, size_t input_size, uint8_t *output);
    void decode_COBS(const uint8_t *input, size_t input_size, uint8_t *output);
public:
    //uart tx functions/////////////////////////////
	bool tx(const data_frame_t &tx_data) override;
	//inline functions
	uint32_t tx_available(void) override

	//uart rx fuctions//////////////////////////////
	uint32_t rx_available(void) override;
	bool rx(data_frame_t &rx_frame) override;

	void rx_interrupt_task(void);
}

#endif /* USER_LIB_COM_UART_HPP_ */
