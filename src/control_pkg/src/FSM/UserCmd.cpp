#include "FSM/UserCmd.h"

UserCmd::UserCmd()
{
    _serial.init("/dev/ttyS3",
        100000 ,
        itas109::ParityEven,
        itas109::DataBits8,
        itas109::StopOne);
        _serial.close();
        _serial.open();
        
        if(!_serial.isOpen()){
            printf("open fail--------- reason\n");
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
            // case 'b':
            //     pthread_mutex_lock(&_mutex);
            //     _user_value = UserValue::BALANCE;
            //     pthread_mutex_unlock(&_mutex);
            //     break;
            // case 't':
            //     pthread_mutex_lock(&_mutex);
            //     _user_value = UserValue::TROTTING;
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
    // else if(R_Data.s1 == 1 && R_Data.s2 ==1){
    //     _user_value = UserValue::BALANCE;
    // }
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