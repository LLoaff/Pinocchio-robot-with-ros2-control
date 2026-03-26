#ifndef KENIMATICS_NORMAL_SOLUTION_H
#define KENIMATICS_NORMAL_SOLUTION_H

#include <eigen3/Eigen/Dense> // 稠密矩阵头文件
#include <math.h> 
#include "mathtool.h"
#include "FSM/EnumClassList.h"
#include "LowState.h"
#include <iostream>



                            // group用来区别是第几条腿
Eigen::Matrix<float,3,1> GetPos_H(uint8_t group, float theta1, float theta2, float theta3)
{
    Eigen::Matrix<float,3,1> matrix;
    float l1 = _labad_;
    float l2 = -_lhip_;
    float l3 = -_lknee_;
    
    if(group == 0 || group == 2)
        l1 = -l1;

    matrix(0) = l3 * sin(theta2+theta3) + l2 * sin(theta2) ;
    matrix(1) = -l3 * sin(theta1)*cos(theta2 + theta3) + l1 * cos(theta1) - l2 * cos(theta2) * sin(theta1);
    matrix(2) = l3 * cos(theta1) * cos(theta2+theta3) + l1 * sin(theta1) + l2* cos(theta1) * cos(theta2);

    return matrix;
}

Eigen::Matrix<float,3,1> GetPos_B(uint8_t group, float theta1, float theta2, float theta3)
{
    Eigen::Matrix<float,3,1> matrix;
    float length = _length_;
    float weigh = _weigh_;
    float l1 = _labad_;
    float l2 = -_lhip_;
    float l3 = -_lknee_;
    
    if(group == 0 || group == 2){
        l1 = -l1;
    }
    
    if(group == 0){
        weigh = -_weigh_;
    }
    else if(group == 3){
        length = -_length_;
    }
    else if(group == 2){
        weigh = -_weigh_;
        length = -_length_;
    }

    matrix(0) = l3 * sin(theta2+theta3) + l2 * sin(theta2) +length;
    matrix(1) = -l3 * sin(theta1)*cos(theta2 + theta3) + l1 * cos(theta1) - l2 * cos(theta2) * sin(theta1)+weigh;
    matrix(2) = l3 * cos(theta1) * cos(theta2+theta3) + l1 * sin(theta1) + l2* cos(theta1) * cos(theta2);

    return matrix;
}

Eigen::Matrix<float,3,1> GetPos_S(Eigen::Matrix<float,3,1> init_pos,uint8_t id,float theta1, float theta2, float theta3)
{
    Eigen::Matrix<float,3,1> matrix_s;

    matrix_s = GetPos_B(id,theta1,theta2,theta3) - init_pos;

    return matrix_s;
}

Eigen::Matrix<float,3,1> Pos_Speed(int group,float theta1, float theta2, float theta3 , float th1_s , float th2_s , float th3_s)
{
    float l1 = _labad_;
    float l2 = -_lhip_;
    float l3 = -_lknee_;

    if(group == 0 || group == 2)
        l1 = -l1;

    Eigen::Matrix<float,3,1> w;
    Eigen::Matrix<float,3,1> pos_s;
    Eigen::Matrix<float,3,3> Jac;

    Jac<< 0, 
          l3*cos(theta2+theta3)+l2*cos(theta2), 
          l3*cos(theta2+theta3),

          -l1*sin(theta1)-l2*cos(theta1)*cos(theta2)-l3*cos(theta1)*cos(theta2+theta3),
          l2*sin(theta1)*sin(theta2)+l3*sin(theta1)*sin(theta2+theta3),
          l3*sin(theta1)*sin(theta2+theta3),

          l1*cos(theta1)-l2*sin(theta1)*cos(theta2)-l3*sin(theta1)*cos(theta2+theta3),
          -l2*cos(theta1)*sin(theta2)-l3*cos(theta1)*sin(theta2+theta3),
          -l3*cos(theta1)*sin(theta2+theta3);
    w<< th1_s,th2_s,th3_s;
    pos_s = Jac * w;
    // syslog(LOG_INFO, "J11=%.6f, J12=%.6f, J13=%.6f", Jac(0,0), Jac(0,1), Jac(0,2));
    // syslog(LOG_INFO, "J21=%.6f, J22=%.6f, J23=%.6f", Jac(1,0), Jac(1,1), Jac(1,2));
    // syslog(LOG_INFO, "J31=%.6f, J32=%.6f, J33=%.6f", Jac(2,0), Jac(2,1), Jac(2,2));
    // syslog(LOG_INFO, "角速度向量：w1=%.6f, w2=%.6f, w3=%.6f", th1_s, th2_s, th3_s);
    return pos_s;
}

// 计算以机身为全局坐标系的
Eigen::Matrix<double,3,4> GetFeetPos2BODY(LowState &lowstate , FrameType frame){
    Eigen::Matrix<double,3,4> feetpos;
    if(frame == FrameType::GLOBAL){
        for(int i(0);i<4;++i){  
           feetpos.col(i) = GetPos_B(i,lowstate.Motor_Angle[3*i+0],lowstate.Motor_Angle[3*i+1],lowstate.Motor_Angle[3*i+2]).cast<double>();
        }
        feetpos = lowstate._imu.GetRotMat().cast<double>()*feetpos;
        // std::cout<<"rotmat:"<< lowstate._imu.GetRotMat().cast<double>() << std::endl;
    }
    else if(frame == FrameType::BODY){
        for(int i(0);i<4;++i){  
           feetpos.col(i) = GetPos_B(i,lowstate.Motor_Angle[3*i+0],lowstate.Motor_Angle[3*i+1],lowstate.Motor_Angle[3*i+2]).cast<double>();
        }
    }
    else if(frame == FrameType::HIP){
        for(int i(0);i<4;++i){  
           feetpos.col(i) = GetPos_H(i,lowstate.Motor_Angle[3*i+0],lowstate.Motor_Angle[3*i+1],lowstate.Motor_Angle[3*i+2]).cast<double>();
        }
    }
    else {
        std::cout << "[ERROR] Frame error " << std::endl;
        exit(-1);
    }
    return feetpos;
}

Eigen::Matrix<double,3,1> GetFeetPos2BODY(LowState &lowstate,int i , FrameType frame){
    Eigen::Matrix<double,3,1> feetpos;
    if(frame == FrameType::GLOBAL){
        feetpos = GetPos_B(i,lowstate.Motor_Angle[3*i+0],lowstate.Motor_Angle[3*i+1],lowstate.Motor_Angle[3*i+2]).cast<double>();
        feetpos = lowstate._imu.GetRotMat().cast<double>()*feetpos;
        // std::cout<<"rotmat:"<< lowstate._imu.GetRotMat().cast<double>() << std::endl;
    }
    else if(frame == FrameType::BODY){
        feetpos = GetPos_B(i,lowstate.Motor_Angle[3*i+0],lowstate.Motor_Angle[3*i+1],lowstate.Motor_Angle[3*i+2]).cast<double>();
    }
    else if(frame == FrameType::HIP){
        feetpos = GetPos_H(i,lowstate.Motor_Angle[3*i+0],lowstate.Motor_Angle[3*i+1],lowstate.Motor_Angle[3*i+2]).cast<double>();
    }
    else {
        std::cout << "[ERROR] Frame error " << std::endl;
        exit(-1);
    }
    return feetpos;
}

Eigen::Matrix<double,3,4> GetFeetSpeed2BODY(LowState &lowstate , FrameType frame){
    Eigen::Matrix<double,3,4>  vel;
    for(int i(0);i<4;++i){
        vel.col(i) = Pos_Speed(i,lowstate.Motor_Angle[3*i+0],lowstate.Motor_Angle[3*i+1],lowstate.Motor_Angle[3*i+2],
                               lowstate._motor_data[3*i+0].dq,lowstate._motor_data[3*i+1].dq,lowstate._motor_data[3*i+2].dq).cast<double>();
    }
    if(frame == FrameType::GLOBAL){
        Eigen::Matrix<double,3,4>  pos = GetFeetPos2BODY(lowstate,FrameType::GLOBAL);
        vel += skew(lowstate._imu.GetGyro().cast<double>()) * pos;     // skew是反对称矩阵，实现叉乘  速度 = 线速度+绕轴旋转角速度
    }
    else if((frame == FrameType::BODY) || (frame == FrameType::HIP)){
    }
    else{
        std::cout << "[ERROR] Frame error" << std::endl;
        exit(-1);
    }
    return vel;
}

#endif


