/*
 * ring_buffer.hpp
 *
 *  Created on: 2023/07/15
 *      Author: yaa3k
 */

#ifndef USER_LIB_RING_BUFFER_HPP_
#define USER_LIB_RING_BUFFER_HPP_

#include <stdbool.h>

template<typename T,buff_size SIZE>
class RingBuffer{
private:
    int head = 0;
    int tail = 0;
    T data_buff[(int)SIZE];
    bool data_is_free[(int)SIZE] = {true};
public:
    enum class buff_size{
        SIZE2 = 2,
        SIZE4 = 4,
        SIZE8 = 8,
        SIZE16 = 16,
        SIZE32 = 32,
        SIZE64 = 64,
        SIZE128 = 128,
    };

    bool push(const T &input){
        data_buff[head] = input;
        int tmp = head;
        head = (head+1) & ((size)SIZE-1);
        if(data_is_free[tmp]){
            data_is_free[tmp] = false;
            return true;
        }else{
            return false;
        }
    }

    bool pop(T &output){
        if(data_is_free[tail]){
            tail = (tail + 1) & ((int)SIZE-1);
            return false;
        }else{
            output = data_buff[tail];
            tail = (tail + 1) & ((int)SIZE-1);
            data_is_free = true;
            return true;
        }
    }
    int get_free_level(void){
        int count = 0;
        for(int i = 0; i < (int)SIZE; i++){
            if(data_is_free[i]) count ++;
        }
        return count;
    }
}

#endif /* USER_LIB_RING_BUFFER_HPP_ */