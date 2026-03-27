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
    MotorData   _motor_data[12];// 接受电机原始数据
    struct Angle_Initialization Angle_Initialization_Variable;
    Imu         _imu;
};


#endif