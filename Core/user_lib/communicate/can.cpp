/*
 * can.cpp
 *
 *  Created on: 2023/07/15
 *      Author: yaa3k
 */

#include "can.hpp"

//tx//////////////////////////////////////////////////////////////////////
bool CanCom::tx(can_frame_t &tx_data){
	uint32_t mailbox_num;
	if(!tx_available()){
		return false;
	}
	CAN_TxHeaderTypeDef tx_header;

	if(tx_data.is_ext_id){
		tx_header.ExtId = tx_data.id;
		tx_header.IDE = CAN_ID_EXT;
	}else{
		tx_header.StdId = tx_data.id;
		tx_header.IDE = CAN_ID_STD;
	}

	if(tx_data.is_remote){
		tx_header.RTR = CAN_RTR_REMOTE;
	}else{
		tx_header.RTR = CAN_RTR_DATA;
	}

	tx_header.DLC = tx_data.size;
	tx_header.TransmitGlobalTime = DISABLE;

	HAL_CAN_AddTxMessage(can, &tx_header, tx_data.data, &mailbox_num);
	return true;
}

//rx//////////////////////////////////////////////////////////////////////////////////////////
void CanCom::rx_interrupt_task(void){
	CAN_RxHeaderTypeDef rx_header;

	HAL_CAN_GetRxMessage(can, rx_fifo, &rx_header, rx_buff[head].data);
	can_frame_t rx_data={
		.is_remote = (rx_header.RTR == CAN_RTR_DATA)? false : true;
		.id = (rx_header.IDE == CAN_ID_STD)? rx_header.StdId : rx_header.ExtId;
		.is_ext_id = (rx_header.IDE == CAN_ID_STD)? false : true;
		.size = rx_header.DLC;
	}

	rx_buff.push(rx_data);
}
bool CanCom::rx(can_frame_t &rx_frame){
	return rx_buff.pop(rx_frame);
}

//filter///////////////////////////////////////////////////////////////////////////////////
void CanCom::set_filter_mask(uint32_t id,uint32_t mask,filter_mode mode,bool as_std){
	CAN_FilterTypeDef filter;
	uint32_t filter_id;
	uint32_t filter_mask;
	switch(mode){
	case filter_mode::only_std:
		if(as_std){
			filter_id = id << 21;
			filter_mask = mask << 21 | 0x4;
		}else{
			filter_id = id << 3;
			filter_mask = mask << 3 | 0x4;
		}
		break;
	case filter_mode::only_ext:
		if(as_std){
			filter_id = id << 21 | 0x4;
			filter_mask = mask << 21 | 0x4;
		}else{
			filter_id = id << 3 | 0x4;
			filter_mask = mask << 3 | 0x4;
		}
		break;
	case filter_mode::std_and_ext:
		if(as_std){
			filter_id = id << 21;
			filter_mask = mask << 21;
		}else{
			filter_id = id << 3;
			filter_mask = mask << 3;
		}

	}

	filter.FilterIdHigh         = filter_id >> 16;
	filter.FilterIdLow          = filter_id;
	filter.FilterMaskIdHigh     = filter_mask >> 16;
	filter.FilterMaskIdLow      = filter_mask;
	filter.FilterScale          = CAN_FILTERSCALE_32BIT; // 32モード
	filter.FilterFIFOAssignment = rx_fifo;      // FIFO0へ格納
	filter.FilterBank           = 0;
	filter.FilterMode           = CAN_FILTERMODE_IDMASK; // IDマスクモード
	filter.SlaveStartFilterBank = 14;
	filter.FilterActivation     = ENABLE;

	HAL_CAN_ConfigFilter(can, &filter);
}
void CanCom::set_filter_free(void){
	CAN_FilterTypeDef filter;
	filter.FilterIdHigh         = 0;
	filter.FilterIdLow          = 0;
	filter.FilterMaskIdHigh     = 0;
	filter.FilterMaskIdLow      = 0;
	filter.FilterScale          = CAN_FILTERSCALE_32BIT;
	filter.FilterFIFOAssignment = rx_fifo;
	filter.FilterBank           = 0;
	filter.FilterMode           = CAN_FILTERMODE_IDMASK;
	filter.SlaveStartFilterBank = 14;
	filter.FilterActivation     = ENABLE;
	HAL_CAN_ConfigFilter(can, &filter);
}
