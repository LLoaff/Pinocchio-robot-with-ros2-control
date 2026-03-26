#ifndef LOWSTATE_H
#define LOWSTATE_H

#include "unitreeMotor/unitreeMotor.h"
#include <syslog.h>
#include <math.h>
#include "Imu.h"
#include <eigen3/Eigen/Dense>

float Angle2Radian(float angle) // 角度转弧度
{
  return angle/180*M_PI;
}
float Radian2Angle(float radian)// 弧度转角度
{
  return radian/M_PI*180;
}

struct Angle_Initialization     // 手动标定电机角度
{
  float Left_Front_Calf_Angle_Init_Value = -141.774;
  float Left_Front_Thigh_Angle_Init_Value = 64.83;
  float Left_Front_Hip_Angle_Init_Value = 30;

  float Right_Front_Calf_Angle_Init_Value = -141.774;
  float Right_Front_Thigh_Angle_Init_Value = 64.83;
  float Right_Front_Hip_Angle_Init_Value = -30;

  float Left_Back_Calf_Angle_Init_Value = -141.774;
  float Left_Back_Thigh_Angle_Init_Value = 64.83;
  float Left_Back_Hip_Angle_Init_Value = 30;

  float Right_Back_Calf_Angle_Init_Value = -141.774;
  float Right_Back_Thigh_Angle_Init_Value = 64.83;
  float Right_Back_Hip_Angle_Init_Value = -30;
};

class LowState
{
public:
  
    LowState()
      {
        
        Angle_Initialization_Variable.Left_Front_Calf_Angle_Init_Value = Angle2Radian(Angle_Initialization_Variable.Left_Front_Calf_Angle_Init_Value);
        Angle_Initialization_Variable.Left_Front_Thigh_Angle_Init_Value = Angle2Radian(Angle_Initialization_Variable.Left_Front_Thigh_Angle_Init_Value);
        Angle_Initialization_Variable.Left_Front_Hip_Angle_Init_Value = Angle2Radian(Angle_Initialization_Variable.Left_Front_Hip_Angle_Init_Value);
    
        Angle_Initialization_Variable.Right_Front_Calf_Angle_Init_Value = Angle2Radian(Angle_Initialization_Variable.Right_Front_Calf_Angle_Init_Value);
        Angle_Initialization_Variable.Right_Front_Thigh_Angle_Init_Value = Angle2Radian(Angle_Initialization_Variable.Right_Front_Thigh_Angle_Init_Value);
        Angle_Initialization_Variable.Right_Front_Hip_Angle_Init_Value = Angle2Radian(Angle_Initialization_Variable.Right_Front_Hip_Angle_Init_Value);

        Angle_Initialization_Variable.Left_Back_Calf_Angle_Init_Value = Angle2Radian(Angle_Initialization_Variable.Left_Back_Calf_Angle_Init_Value);
        Angle_Initialization_Variable.Left_Back_Thigh_Angle_Init_Value = Angle2Radian(Angle_Initialization_Variable.Left_Back_Thigh_Angle_Init_Value);
        Angle_Initialization_Variable.Left_Back_Hip_Angle_Init_Value = Angle2Radian(Angle_Initialization_Variable.Left_Back_Hip_Angle_Init_Value);

        Angle_Initialization_Variable.Right_Back_Calf_Angle_Init_Value = Angle2Radian(Angle_Initialization_Variable.Right_Back_Calf_Angle_Init_Value);
        Angle_Initialization_Variable.Right_Back_Thigh_Angle_Init_Value = Angle2Radian(Angle_Initialization_Variable.Right_Back_Thigh_Angle_Init_Value);
        Angle_Initialization_Variable.Right_Back_Hip_Angle_Init_Value = Angle2Radian(Angle_Initialization_Variable.Right_Back_Hip_Angle_Init_Value);

        for(uint8_t i=0 ; i<12; i++)
        {
          _motor_data[i].motorType = MotorType::GO_M8010_6;
        }
      }
    Eigen::Matrix<float,3,4> getQ();
    Eigen::Matrix<float,12,1> getQ12();
    Eigen::Matrix<float,12,1> getW12();


    MotorData   _motor_data[12];// 接受电机原始数据
    struct Angle_Initialization Angle_Initialization_Variable;
    float Motor_Init_Angle[12]; // 存储开机时电机初始角度  弧度制
    float Motor_Angle[12];      // 存储后续所有电机角度    弧度制
    // float Motor_W[12];          // 存储后续所有电机角速度    弧度制
    Imu         _imu;
};

Eigen::Matrix<float,3,4> LowState::getQ(){
  Eigen::Matrix<float,3,4> qLegs;
  for(int i(0); i < 4; ++i){
      qLegs.col(i)(0) = Motor_Angle[3*i    ];
      qLegs.col(i)(1) = Motor_Angle[3*i + 1];
      qLegs.col(i)(2) = Motor_Angle[3*i + 2];
  }
  return qLegs;
}

Eigen::Matrix<float,12,1> LowState::getQ12(){
  Eigen::Matrix<float,12,1> qLegs;
  for(int i(0); i < 4; ++i){
      qLegs(3*i  ) = Motor_Angle[3*i  ];
      qLegs(3*i+1) = Motor_Angle[3*i+1];
      qLegs(3*i+2) = Motor_Angle[3*i+2];
  }
  return qLegs;
}

Eigen::Matrix<float,12,1> LowState::getW12(){
  Eigen::Matrix<float,12,1> w;
  for(int i(0); i < 4; ++i){
      w(3*i  ) = _motor_data[3*i  ].dq ;
      w(3*i+1) = _motor_data[3*i+1].dq ;
      w(3*i+2) = _motor_data[3*i+2].dq;
  }
  return w;
}


#endif