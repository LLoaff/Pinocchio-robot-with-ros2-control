#ifndef USER_CMD_H
#define USER_CMD_H

#include <pthread.h>
#include <iostream>  
#include <unistd.h> 
#include "CSerialPort/SerialPort.h"
#include "EnumClassList.h"
#include <termios.h>

// 设置键盘 非阻塞、无回显、无需回车
void set_keyboard_nonblock() {
    struct termios attr;
    tcgetattr(0, &attr);
    attr.c_cc[VMIN] = 0;              // 读键盘时：没有数据立刻返回
    attr.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &attr);
}

// 非阻塞读键盘
int kbhit() {
    unsigned char ch;
    if (read(0, &ch, 1) == 1) {
        return ch; // 返回按键值
    }
    return 0;      // 无按键
}
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

UserCmd::UserCmd()
{
    _serial.init("/dev/ttyUSB0",
        100000 ,
        itas109::ParityEven,
        itas109::DataBits8,
        itas109::StopOne);
        _serial.close();
        _serial.open();
        
        if(!_serial.isOpen()){
            printf("open fail--------- reason:%s\n",strerror(errno));
            _serial.close();
        }
    pthread_mutex_init(&_mutex,NULL);
    pthread_create(&_thread,NULL,UserCmd::KeyBoardInit,this);
    
 
}

void* UserCmd::KeyBoardInit(void * arg)
{
    UserCmd * user = static_cast<UserCmd *>(arg);
    user ->KeyBoardGet();
    return NULL;
}

void UserCmd::KeyBoardGet()
{
    // set_keyboard_nonblock();
    char key=0;
        while(true)
        {
            // key = kbhit();
            // if (key != 0) {
            //     pthread_mutex_lock(&_mutex);
            //     switch (key) {
            //         case 'p': _user_value = UserValue::PASSIVE; break;
            //         case 's': _user_value = UserValue::STAND;   break;
            //         case 'f': _user_value = UserValue::FREE;    break;
            //     }
            //     pthread_mutex_unlock(&_mutex);
            // }
            // key = getchar();
            // switch (key)
            // {
            // case 'p':
            //     pthread_mutex_lock(&_mutex);
            //     _user_value = UserValue::PASSIVE;
            //     pthread_mutex_unlock(&_mutex);
            //     break;
            // case 's':
            //     pthread_mutex_lock(&_mutex);
            //     _user_value = UserValue::STAND;
            //     pthread_mutex_unlock(&_mutex);
            //     break;
            // case 'f':
            //     pthread_mutex_lock(&_mutex);
            //     _user_value = UserValue::FREE;
            //     pthread_mutex_unlock(&_mutex);
            //     break;
            // default:
            //     break;
            // }

            if(_serial.getReadBufferUsedLen()>=18){
                pthread_mutex_lock(&_mutex);

                _serial.readData(_srerial_data,18);

                R_Data.ch0 = ((int16_t)_srerial_data[0] | ((int16_t)_srerial_data[1] << 8)) & 0x07FF;
                R_Data.ch1 = (((int16_t)_srerial_data[1] >> 3) | ((int16_t)_srerial_data[2] << 5))& 0x07FF;
                R_Data.ch2 = (((int16_t)_srerial_data[2] >> 6) | ((int16_t)_srerial_data[3] << 2) |((int16_t)_srerial_data[4] << 10)) & 0x07FF;
                R_Data.ch3 = (((int16_t)_srerial_data[4] >> 1) | ((int16_t)_srerial_data[5]<<7)) &0x07FF;
                R_Data.s1 = ((_srerial_data[5] >> 4) & 0x000C) >> 2;
                R_Data.s2 = ((_srerial_data[5] >> 4) & 0x0003);

                
                pthread_mutex_unlock(&_mutex);
            }
        }
}

UserValue UserCmd::GetUserValue()
{
    // std::cout<<"s1: "<< (int)R_Data.s1<<std::endl;
    // std::cout<<"s2: "<< (int)R_Data.s2<<std::endl;

    if(R_Data.s1 == 3 && R_Data.s2 ==3){
        _user_value = UserValue::PASSIVE;
    }
    else if(R_Data.s1 == 3 && R_Data.s2 ==1){
        _user_value = UserValue::FREE;
    }
    else if(R_Data.s1 == 1 && R_Data.s2 ==3){
        _user_value = UserValue::STAND;
    }
    else if(R_Data.s1 == 1 && R_Data.s2 ==2){
        _user_value = UserValue::TROTTING;
    }
    else if(R_Data.s1 == 1 && R_Data.s2 ==1){
        _user_value = UserValue::TROTTING;
    }
    pthread_mutex_lock(&_mutex);
    UserValue val = _user_value;
    pthread_mutex_unlock(&_mutex);
    return val;
}

UserCmd::~UserCmd()
{
    pthread_mutex_destroy(&_mutex);
    pthread_join(_thread, NULL);
    // _serial.close();
}
#endif