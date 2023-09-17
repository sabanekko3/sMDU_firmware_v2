#include "uart.hpp"

void UartCom::tx(const data_frame_t &tx_data){

}
uint32_t UartCom::tx_available(void){

}

uint32_t UartCom::rx_available(void){

}
bool UartCom::rx(data_frame_t &rx_frame){

}
void UartCom::rx_interrupt_task(void){

}


//private funcitons////////////////////////////////////////////////////////////////////////////////
void UartCom::encode_COBS(const uint8_t *input, size_t input_size, uint8_t *output) {
    int read_index = 0;
    int write_index = 1;
    int code_index = 0;
    uint8_t code = 1;

    while (read_index < input_size) {
        if (input[read_index] == 0x00) {
            output[code_index] = code;
            code = 1;
            code_index = write_index++;
            read_index++;
        } else {
            output[write_index++] = input[read_index++];
            code++;
            if (code == 0xFF) {
                output[code_index] = code;
                code = 1;
                code_index = write_index++;
            }
        }
    }
    output[code_index] = code;
    output[write_index] = 0x00;  // COBSパケットの終端を示す0x00を追加
}

void UartCom:decode_COBS(const uint8_t *input, size_t input_size, uint8_t *output) {
    int read_index = 0;
    int write_index = 0;

    while (read_index < input_size) {
        uint8_t code = input[read_index];
        if (code == 0x00) {
            break;  // COBSパケットの終端に達した場合、終了
        }
        read_index++;

        for (int i = 0; i < code - 1; i++) {
            output[write_index++] = input[read_index++];
        }

        if (code < 0xFF && read_index < input_size) {
            output[write_index++] = 0x00;  // 0x00を挿入
        }
    }
}