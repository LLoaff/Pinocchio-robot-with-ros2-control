// #ifndef MOTORLOWCONTROL_H
// #define MOTORLOWCONTROL_H

// #include <unistd.h>
// #include "serialPort/SerialPort.h"
// #include "unitreeMotor/unitreeMotor.h"
// #include <syslog.h>
// #include <math.h>

// extern SerialPort  serial;

// struct Angle_Initialization     // 手动标定电机角度
// {
//   float Left_Front_Calf_Angle_Init_Value = -141.774;
//   float Left_Front_Thigh_Angle_Init_Value = 64.83;
//   float Left_Front_Hip_Angle_Init_Value = 30;

//   float Right_Front_Calf_Angle_Init_Value = -141.774;
//   float Right_Front_Thigh_Angle_Init_Value = 64.83;
//   float Right_Front_Hip_Angle_Init_Value = -30;

//   float Left_Back_Calf_Angle_Init_Value = -141.774;
//   float Left_Back_Thigh_Angle_Init_Value = 64.83;
//   float Left_Back_Hip_Angle_Init_Value = 30;

//   float Right_Back_Calf_Angle_Init_Value = -141.774;
//   float Right_Back_Thigh_Angle_Init_Value = 64.83;
//   float Right_Back_Hip_Angle_Init_Value = -30;
// };

// struct User_Cmd
// {
//   unsigned short id;
//   float q;
//   float dq;
//   float kp;
//   float kd;
//   float tau;
// };

// struct Motor_State
// {
//   float q;
//   float dq;
//   float tau;
//   float temp;
// };

// extern float Motor_Init_Angle[12];// 存储开机时电机的角度
// extern float Motor_Angle[12]; //存储后续所有电机角度   (角度归一化处理后)
// extern User_Cmd Left_Front_Calf;
// extern User_Cmd Left_Front_Thigh;
// extern User_Cmd Left_Front_Hip;

// extern User_Cmd Right_Front_Calf;
// extern User_Cmd Right_Front_Thigh;
// extern User_Cmd Right_Front_Hip;

// extern User_Cmd Left_Back_Calf;
// extern User_Cmd Left_Back_Thigh;
// extern User_Cmd Left_Back_Hip;

// extern User_Cmd Right_Back_Calf;
// extern User_Cmd Right_Back_Thigh;
// extern User_Cmd Right_Back_Hip;

// extern Motor_State Left_Front_Calf_Sate;
// extern Motor_State Left_Front_Thigh_Sate;
// extern Motor_State Left_Front_Hip_Sate;

// extern Motor_State Right_Front_Calf_Sate;
// extern Motor_State Right_Front_Thigh_Sate;
// extern Motor_State Right_Front_Hip_Sate;

// extern Motor_State Left_Back_Calf_Sate;
// extern Motor_State Left_Back_Thigh_Sate;
// extern Motor_State Left_Back_Hip_Sate;

// extern Motor_State Right_Back_Calf_Sate;
// extern Motor_State Right_Back_Thigh_Sate;
// extern Motor_State Right_Back_Hip_Sate;

// extern struct Angle_Initialization Angle_Initialization_Variable;
// extern struct User_Cmd Motor_User_Cmd[12];
// extern struct Motor_State Motor_State_Data[12]; // 存储电机真实未归一化的角度

// float Angle2Radian(float angle);
// float Radian2Angle(float radian);

// void Angle_Initializate();  // 十二个电机初始化函数 ， 把所有电机开机时角度存储到Motor_Init_Angle中

// void Motor_Angle_Update( User_Cmd (&user_cmd)[12]);   // 更新十二个电机角度数据到Motor_Angle 变量中




// #endif