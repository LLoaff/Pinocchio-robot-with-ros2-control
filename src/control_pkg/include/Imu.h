#ifndef IMU_H
#define IMU_H

#include "CSerialPort/SerialPort.h"
#include <iostream>  
#include <unistd.h> 
#include "eigen3/Eigen/Dense"
#include "mathtool.h"
#include <syslog.h>

class Imu
{
public:
    Imu();
    ~Imu();

    void Imu_Update();
    Eigen::Matrix<float,3,3> GetRotMat();
    Eigen::Matrix<float,3,1> GetAcc();
    Eigen::Matrix<float,3,1> GetGyro();
    Eigen::Matrix<float,4,1> GetQuat();
    Eigen::Matrix<float,3,1> getAccGlobal();
    Eigen::Matrix<float,3,1> getGyroGlobal();
    float getYaw();
    float getDYaw();

    float quaternion[4];    // w, x, y, z
    float gyroscope[3];
    float accelerometer[3];
    uint8_t data_buff[42];
private:
    itas109::CSerialPort _serial;

};

Imu::Imu()
{
    _serial.init("/dev/ttyS4",
        1000000 ,
        itas109::ParityNone,
        itas109::DataBits8,
        itas109::StopOne);
        _serial.close();
        _serial.open();
        
        if(!_serial.isOpen()){
            printf("open fail--------- \n");
            _serial.close();
        }
}

void Imu::Imu_Update(){

    uint8_t send_data=0x0a;
    int counter=10;
    _serial.writeData(&send_data,1);
    while(_serial.getReadBufferUsedLen()==0){
        counter--;
        if(counter == 0){
            #ifdef LOWSTATE_DEBUG
            syslog(LOG_PERROR,"Imu No Rec !!!");
            #endif
            // std::cout<< "Imu No Rec !!!"<<std::endl;
            break;
        }
    }
    if(_serial.getReadBufferUsedLen()>0){
        _serial.readData(data_buff,42);
        if(data_buff[0]==0x0a && data_buff[41]==0x0b){
            memcpy(&quaternion[0], &data_buff[1], 4);
            memcpy(&quaternion[1], &data_buff[5], 4);
            memcpy(&quaternion[2], &data_buff[9], 4);
            memcpy(&quaternion[3], &data_buff[13], 4);

            memcpy(&accelerometer[0], &data_buff[17], 4);
            memcpy(&accelerometer[1], &data_buff[21], 4);
            memcpy(&accelerometer[2], &data_buff[25], 4);

            memcpy(&gyroscope[0], &data_buff[29], 4);
            memcpy(&gyroscope[1], &data_buff[33], 4);
            memcpy(&gyroscope[2], &data_buff[37], 4);
            #ifdef LOWSTATE_DEBUG
            syslog(LOG_INFO,"imu: q:%.3f %.3f %.3f %.3f - ac:%.3f %.3f %.3f - gy:%.3f %.3f %.3f "
                ,quaternion[0],quaternion[1],quaternion[2],quaternion[3],accelerometer[0],accelerometer[1],accelerometer[2],gyroscope[0],gyroscope[1],gyroscope[2]);
            #endif
        }
    }
}

Eigen::Matrix<float,3,3> Imu::GetRotMat(){
    Eigen::Matrix<float, 4, 1> quat;
    quat << quaternion[0],quaternion[1],quaternion[2],quaternion[3];
    return Quat2RotMat(quat);
}

Eigen::Matrix<float,3,1> Imu::GetAcc(){
    Eigen::Matrix<float,3,1> a;
    a<< accelerometer[0],accelerometer[1],accelerometer[2];
    return a;
}

Eigen::Matrix<float,3,1> Imu::GetGyro(){
    Eigen::Matrix<float,3,1> gryo;
    gryo<< gyroscope[0],gyroscope[1],gyroscope[2];
    return gryo;
}

Eigen::Matrix<float,4,1> Imu::GetQuat(){
    Eigen::Matrix<float,4,1> q;
    q<< quaternion[0],quaternion[1],quaternion[2],quaternion[3];
    return q;
}
Eigen::Matrix<float,3,1> Imu::getAccGlobal(){
        return GetRotMat() * GetAcc();
}

Eigen::Matrix<float,3,1> Imu::getGyroGlobal(){
    return GetRotMat() * GetGyro();
}

float Imu::getYaw(){
    return rotMatToRPY(GetRotMat())(2);
}

float Imu::getDYaw(){
    return getGyroGlobal()(2);
}

Imu::~Imu(){
    _serial.close();
}





#endif