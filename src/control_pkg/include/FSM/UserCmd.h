#ifndef USER_CMD_H
#define USER_CMD_H

#include <pthread.h>
#include <iostream>  
#include <unistd.h> 
#include "CSerialPort/SerialPort.h"
#include "EnumClassList.h"
#include <termios.h>

typedef struct __packed{
        uint16_t ch0;
        uint16_t ch1;
        uint16_t ch2;
        uint16_t ch3;
        uint8_t s1;
        uint8_t s2;
}RC_Ctl_t;

class UserCmd
{
public:
    UserCmd();
    ~UserCmd();
    static UserCmd* GetInstance() {
        static UserCmd instance; // 静态局部变量，保证唯一且线程安全（C++11+）
        return &instance;
    }
    UserValue GetUserValue();

    static void *KeyBoardInit(void * arg);
    void KeyBoardGet();
    pthread_t           _thread;
    pthread_mutex_t     _mutex;
    UserValue           _user_value = UserValue::PASSIVE;
    itas109::CSerialPort _serial;
    uint8_t             _srerial_data[18];
    RC_Ctl_t            R_Data;
};
#endif