#ifndef LOWCMD_H
#define LOWCMD_H

#include <unistd.h>
#include "serialPort/SerialPort.h"
#include <syslog.h>
#include <math.h>
#include "LowState.h"
#include <eigen3/Eigen/Dense> 
#include "mathtool.h"


float Limit_Angle[6] = {
    -50 / 180.0 *M_PI, -141.774 / 180.0 *M_PI,
     80 / 180.0 *M_PI, -40 / 180.0 *M_PI,
     40 / 180.0 *M_PI, -40 / 180.0 *M_PI
};

class LowCmd
{
    public:
        LowState& getInternalState() { return _state; }
        LowCmd()
        {
            serial = new SerialPort("/dev/ttyUSB0");

            #if defined(LOWCMD_DEBUG) || defined(LOWSTATE_DEBUG)
                    openlog("debug",LOG_PID | LOG_CONS , LOG_USER);
            #endif

            _cmd[5].q =_state.Angle_Initialization_Variable.Left_Front_Calf_Angle_Init_Value;
            _cmd[4].q =_state.Angle_Initialization_Variable.Left_Front_Thigh_Angle_Init_Value;
            _cmd[3].q =_state.Angle_Initialization_Variable.Left_Front_Hip_Angle_Init_Value;
            
            _cmd[2].q =_state.Angle_Initialization_Variable.Right_Front_Calf_Angle_Init_Value;
            _cmd[1].q =_state.Angle_Initialization_Variable.Right_Front_Thigh_Angle_Init_Value;
            _cmd[0].q =_state.Angle_Initialization_Variable.Right_Front_Hip_Angle_Init_Value;

            _cmd[11].q =_state.Angle_Initialization_Variable.Left_Back_Calf_Angle_Init_Value;
            _cmd[10].q =_state.Angle_Initialization_Variable.Left_Back_Thigh_Angle_Init_Value;
            _cmd[9].q =_state.Angle_Initialization_Variable.Left_Back_Hip_Angle_Init_Value;

            _cmd[8].q =_state.Angle_Initialization_Variable.Right_Back_Calf_Angle_Init_Value;
            _cmd[7].q =_state.Angle_Initialization_Variable.Right_Back_Thigh_Angle_Init_Value;
            _cmd[6].q =_state.Angle_Initialization_Variable.Right_Back_Hip_Angle_Init_Value;

            for(int i=0; i<12;i++ )
            {
                _cmd[i].motorType = MotorType::GO_M8010_6;
                _cmd[i].mode =queryMotorMode(MotorType::GO_M8010_6,MotorMode::FOC);
                _cmd[i].id   = i;
                _cmd[i].kp   = 0;
                _cmd[i].kd   = 0;
                _cmd[i].dq   = 0;
                _cmd[i].tau  = 0;

                if(serial->sendRecv(& _cmd[i],& _state._motor_data[i] ))
                {
                    _state._motor_data[i].q = _state._motor_data[i].q / 6.33;

                    _state.Motor_Init_Angle[i] = _state._motor_data[i].q ;
                    switch(i)
                    {
                        case 0:
                            _state._motor_data[i].dq = _state._motor_data[i].dq / 6.33;
                            _state.Motor_Angle[i]=
                            _state.Angle_Initialization_Variable.Right_Front_Hip_Angle_Init_Value +(_state._motor_data[i].q - _state.Motor_Init_Angle[i]);
                            break;
                        case 1:
                            _state._motor_data[i].dq = -_state._motor_data[i].dq / 6.33;
                            _state.Motor_Angle[i]=
                            _state.Angle_Initialization_Variable.Right_Front_Thigh_Angle_Init_Value -(_state._motor_data[i].q  - _state.Motor_Init_Angle[i]);
                            break;
                        case 2:
                            _state._motor_data[i].dq = _state._motor_data[i].dq / 6.33;
                            _state.Motor_Angle[i]=
                            _state.Angle_Initialization_Variable.Right_Front_Calf_Angle_Init_Value +(_state._motor_data[i].q  - _state.Motor_Init_Angle[i]);
                            break;
                        case 3:
                            _state._motor_data[i].dq = _state._motor_data[i].dq / 6.33;
                            _state.Motor_Angle[i]=
                            _state.Angle_Initialization_Variable.Left_Front_Hip_Angle_Init_Value +(_state._motor_data[i].q  - _state.Motor_Init_Angle[i]);
                            break;
                        case 4:
                            _state._motor_data[i].dq = _state._motor_data[i].dq / 6.33;
                            _state.Motor_Angle[i]=
                            _state.Angle_Initialization_Variable.Left_Front_Thigh_Angle_Init_Value +(_state._motor_data[i].q  - _state.Motor_Init_Angle[i]);
                            break;
                        case 5:
                            _state._motor_data[i].dq = -_state._motor_data[i].dq / 6.33;
                            _state.Motor_Angle[i]=
                            _state.Angle_Initialization_Variable.Left_Front_Calf_Angle_Init_Value -(_state._motor_data[i].q  - _state.Motor_Init_Angle[i]);
                            break;
                        case 6:
                            _state._motor_data[i].dq = -_state._motor_data[i].dq / 6.33;
                            _state.Motor_Angle[i]=
                            _state.Angle_Initialization_Variable.Right_Back_Hip_Angle_Init_Value -(_state._motor_data[i].q - _state.Motor_Init_Angle[i]);
                            break;
                        case 7:
                            _state._motor_data[i].dq = -_state._motor_data[i].dq / 6.33;
                            _state.Motor_Angle[i]=
                            _state.Angle_Initialization_Variable.Right_Back_Thigh_Angle_Init_Value -(_state._motor_data[i].q - _state.Motor_Init_Angle[i]);
                            break;
                        case 8:
                            _state._motor_data[i].dq = _state._motor_data[i].dq / 6.33;
                            _state.Motor_Angle[i]=
                            _state.Angle_Initialization_Variable.Right_Back_Calf_Angle_Init_Value +(_state._motor_data[i].q - _state.Motor_Init_Angle[i]);
                            break;
                        case 9:
                            _state._motor_data[i].dq = -_state._motor_data[i].dq / 6.33;
                            _state.Motor_Angle[i]=
                            _state.Angle_Initialization_Variable.Left_Back_Hip_Angle_Init_Value -(_state._motor_data[i].q - _state.Motor_Init_Angle[i]);
                            break;
                        case 10:
                            _state._motor_data[i].dq = _state._motor_data[i].dq / 6.33;
                            _state.Motor_Angle[i]=
                            _state.Angle_Initialization_Variable.Left_Back_Thigh_Angle_Init_Value +(_state._motor_data[i].q - _state.Motor_Init_Angle[i]);
                            break;
                        case 11:
                            _state._motor_data[i].dq = -_state._motor_data[i].dq / 6.33;
                            _state.Motor_Angle[i]=
                            _state.Angle_Initialization_Variable.Left_Back_Calf_Angle_Init_Value -(_state._motor_data[i].q - _state.Motor_Init_Angle[i]);
                            break;
                    }
                    
                    #ifdef LOWSTATE_DEBUG
                        syslog(LOG_INFO,"init_state: id:%d -- 原始q:%.2f -- 归一q:%.2f --",i,Radian2Angle(_state._motor_data[i].q),Radian2Angle(_state.Motor_Angle[i]));
                    #endif
                    #ifdef LOWCMD_DEBUG
                    syslog(LOG_INFO,"cmd: id: %d -- q: %.2f -- dq: %.2f -- kp: %.2f -- kd: %.2f -- tau: %.3f",i,Radian2Angle(_cmd[i].q)
                    ,Radian2Angle(_cmd[i].dq),_cmd[i].kp,_cmd[i].kd,_cmd[i].tau);
                #endif
                }
                
            }
        }

        void Update()
        {
            MotorCmd tmp_cmd[12];
            _state._imu.Imu_Update();
            for(int i(0);i<12;++i)
            {
                tmp_cmd[i].motorType = MotorType::GO_M8010_6;
                tmp_cmd[i].mode =queryMotorMode(MotorType::GO_M8010_6,MotorMode::FOC);
                tmp_cmd[i].q = _cmd[i].q;
                tmp_cmd[i].id = _cmd[i].id;
                tmp_cmd[i].kp = _cmd[i].kp;
                tmp_cmd[i].kd = _cmd[i].kd;
                switch(i)
                {
                    case 0:
                        if(tmp_cmd[i].q > Limit_Angle[4])
                            tmp_cmd[i].q= Limit_Angle[4];
                        else if (tmp_cmd[i].q < Limit_Angle[5])
                            tmp_cmd[i].q = Limit_Angle[5];
                        tmp_cmd[i].q  =(_state._motor_data[i].q +(tmp_cmd[i].q - _state.Motor_Angle[i])) * 6.33;
                        tmp_cmd[i].dq =  _cmd[i].dq * 6.33;
                        tmp_cmd[i].tau =  _cmd[i].tau;

                        break;
                    case 1:
                        if(tmp_cmd[i].q > Limit_Angle[2])
                            tmp_cmd[i].q = Limit_Angle[2];
                        else if (tmp_cmd[i].q < Limit_Angle[3])
                            tmp_cmd[i].q = Limit_Angle[3];
                        tmp_cmd[i].q  =(_state._motor_data[i].q -(tmp_cmd[i].q - _state.Motor_Angle[i]) ) * 6.33;
                        tmp_cmd[i].dq =- _cmd[i].dq * 6.33;
                        tmp_cmd[i].tau = -_cmd[i].tau;

                        break;
                    case 2:
                        if(tmp_cmd[i].q > Limit_Angle[0])
                            tmp_cmd[i].q = Limit_Angle[0];
                        else if (tmp_cmd[i].q < Limit_Angle[1])
                            tmp_cmd[i].q = Limit_Angle[1];
                        tmp_cmd[i].q  =(_state._motor_data[i].q +(tmp_cmd[i].q - _state.Motor_Angle[i]) ) * 6.33;
                        tmp_cmd[i].dq = _cmd[i].dq * 6.33;
                        tmp_cmd[i].tau = _cmd[i].tau;

                        break;
                    case 3:
                        if(tmp_cmd[i].q > Limit_Angle[4])
                            tmp_cmd[i].q = Limit_Angle[4];
                        else if (tmp_cmd[i].q < Limit_Angle[5])
                            tmp_cmd[i].q = Limit_Angle[5];
                        tmp_cmd[i].q  =(_state._motor_data[i].q +(tmp_cmd[i].q - _state.Motor_Angle[i]) ) * 6.33;
                        tmp_cmd[i].dq = _cmd[i].dq * 6.33;
                        tmp_cmd[i].tau = _cmd[i].tau;

                        break;
                    case 4:
                        if(tmp_cmd[i].q > Limit_Angle[2])
                            tmp_cmd[i].q = Limit_Angle[2];
                        else if (tmp_cmd[i].q < Limit_Angle[3])
                            tmp_cmd[i].q = Limit_Angle[3];
                        tmp_cmd[i].q  =(_state._motor_data[i].q +(tmp_cmd[i].q - _state.Motor_Angle[i]) ) * 6.33;
                        tmp_cmd[i].dq = _cmd[i].dq * 6.33;
                        tmp_cmd[i].tau = _cmd[i].tau;
      
                        break;
                    case 5:
                        if(tmp_cmd[i].q > Limit_Angle[0])
                            tmp_cmd[i].q = Limit_Angle[0];
                        else if (tmp_cmd[i].q < Limit_Angle[1])
                            tmp_cmd[i].q = Limit_Angle[1];
                        tmp_cmd[i].q  =(_state._motor_data[i].q -(tmp_cmd[i].q - _state.Motor_Angle[i]) ) * 6.33;
                        tmp_cmd[i].dq = -_cmd[i].dq * 6.33;
                        tmp_cmd[i].tau = -_cmd[i].tau;

                        break;
                    case 6:
                        if(tmp_cmd[i].q > Limit_Angle[4])
                            tmp_cmd[i].q = Limit_Angle[4];
                        else if (tmp_cmd[i].q < Limit_Angle[5])
                            tmp_cmd[i].q = Limit_Angle[5];
                        tmp_cmd[i].q  =(_state._motor_data[i].q -(tmp_cmd[i].q - _state.Motor_Angle[i]) ) * 6.33;
                        tmp_cmd[i].dq = -_cmd[i].dq * 6.33;
                        tmp_cmd[i].tau =- _cmd[i].tau;

                        break;
                    case 7:
                        if(tmp_cmd[i].q > Limit_Angle[2])
                            tmp_cmd[i].q = Limit_Angle[2];
                        else if (tmp_cmd[i].q < Limit_Angle[3])
                            tmp_cmd[i].q = Limit_Angle[3];
                        tmp_cmd[i].q  =(_state._motor_data[i].q -(tmp_cmd[i].q - _state.Motor_Angle[i]) ) * 6.33;
                        tmp_cmd[i].dq = -_cmd[i].dq * 6.33;
                        tmp_cmd[i].tau = -_cmd[i].tau;

                        break;
                    case 8:
                        if(tmp_cmd[i].q > Limit_Angle[0])
                            tmp_cmd[i].q = Limit_Angle[0];
                        else if (tmp_cmd[i].q < Limit_Angle[1])
                            tmp_cmd[i].q = Limit_Angle[1];
                        tmp_cmd[i].q  =(_state._motor_data[i].q +(tmp_cmd[i].q - _state.Motor_Angle[i]) ) * 6.33;
                        tmp_cmd[i].dq = _cmd[i].dq * 6.33;
                        tmp_cmd[i].tau = _cmd[i].tau;

                        break;
                    case 9:
                        if(tmp_cmd[i].q > Limit_Angle[4])
                            tmp_cmd[i].q = Limit_Angle[4];
                        else if (tmp_cmd[i].q < Limit_Angle[5])
                            tmp_cmd[i].q = Limit_Angle[5];
                        tmp_cmd[i].q  =(_state._motor_data[i].q -(tmp_cmd[i].q - _state.Motor_Angle[i]) ) * 6.33;
                        tmp_cmd[i].dq = -_cmd[i].dq * 6.33;
                        tmp_cmd[i].tau =- _cmd[i].tau;

                        break;
                    case 10:
                        if(tmp_cmd[i].q > Limit_Angle[2])
                            tmp_cmd[i].q = Limit_Angle[2];
                        else if (tmp_cmd[i].q < Limit_Angle[3])
                            tmp_cmd[i].q = Limit_Angle[3];
                        tmp_cmd[i].q  =(_state._motor_data[i].q +(tmp_cmd[i].q - _state.Motor_Angle[i]) ) * 6.33;
                        tmp_cmd[i].dq = _cmd[i].dq * 6.33;
                        tmp_cmd[i].tau = _cmd[i].tau;

                        break;
                    case 11:
                        if(tmp_cmd[i].q > Limit_Angle[0])
                            tmp_cmd[i].q = Limit_Angle[0];
                        else if (tmp_cmd[i].q < Limit_Angle[1])
                            tmp_cmd[i].q = Limit_Angle[1];
                        tmp_cmd[i].q  =(_state._motor_data[i].q -(tmp_cmd[i].q - _state.Motor_Angle[i]) ) * 6.33;
                        tmp_cmd[i].dq = -_cmd[i].dq * 6.33;
                        tmp_cmd[i].tau =- _cmd[i].tau;

                        break;
                }
                
                if(serial->sendRecv(&tmp_cmd[i],&_state._motor_data[i]))
                {
                    _state._motor_data[i].q = _state._motor_data[i].q /6.33;
                    switch(i)
                    {
                        case 0:
                            _state._motor_data[i].dq = _state._motor_data[i].dq / 6.33;
                            _state.Motor_Angle[i]=
                            _state.Angle_Initialization_Variable.Right_Front_Hip_Angle_Init_Value +(_state._motor_data[i].q - _state.Motor_Init_Angle[i]);
                            break;
                        case 1:
                            _state._motor_data[i].dq = -_state._motor_data[i].dq / 6.33;
                            _state.Motor_Angle[i]=
                            _state.Angle_Initialization_Variable.Right_Front_Thigh_Angle_Init_Value -(_state._motor_data[i].q  - _state.Motor_Init_Angle[i]);
                            break;
                        case 2:
                            _state._motor_data[i].dq = _state._motor_data[i].dq / 6.33;
                            _state.Motor_Angle[i]=
                            _state.Angle_Initialization_Variable.Right_Front_Calf_Angle_Init_Value +(_state._motor_data[i].q  - _state.Motor_Init_Angle[i]);
                            break;
                        case 3:
                            _state._motor_data[i].dq = _state._motor_data[i].dq / 6.33;
                            _state.Motor_Angle[i]=
                            _state.Angle_Initialization_Variable.Left_Front_Hip_Angle_Init_Value +(_state._motor_data[i].q  - _state.Motor_Init_Angle[i]);
                            break;
                        case 4:
                            _state._motor_data[i].dq = _state._motor_data[i].dq / 6.33;
                            _state.Motor_Angle[i]=
                            _state.Angle_Initialization_Variable.Left_Front_Thigh_Angle_Init_Value +(_state._motor_data[i].q  - _state.Motor_Init_Angle[i]);
                            break;
                        case 5:
                            _state._motor_data[i].dq = -_state._motor_data[i].dq / 6.33;
                            _state.Motor_Angle[i]=
                            _state.Angle_Initialization_Variable.Left_Front_Calf_Angle_Init_Value -(_state._motor_data[i].q  - _state.Motor_Init_Angle[i]);
                            break;
                        case 6:
                            _state._motor_data[i].dq = -_state._motor_data[i].dq / 6.33;
                            _state.Motor_Angle[i]=
                            _state.Angle_Initialization_Variable.Right_Back_Hip_Angle_Init_Value -(_state._motor_data[i].q - _state.Motor_Init_Angle[i]);
                            break;
                        case 7:
                            _state._motor_data[i].dq = -_state._motor_data[i].dq / 6.33;
                            _state.Motor_Angle[i]=
                            _state.Angle_Initialization_Variable.Right_Back_Thigh_Angle_Init_Value -(_state._motor_data[i].q - _state.Motor_Init_Angle[i]);
                            break;
                        case 8:
                            _state._motor_data[i].dq = _state._motor_data[i].dq / 6.33;
                            _state.Motor_Angle[i]=
                            _state.Angle_Initialization_Variable.Right_Back_Calf_Angle_Init_Value +(_state._motor_data[i].q - _state.Motor_Init_Angle[i]);
                            break;
                        case 9:
                            _state._motor_data[i].dq = -_state._motor_data[i].dq / 6.33;
                            _state.Motor_Angle[i]=
                            _state.Angle_Initialization_Variable.Left_Back_Hip_Angle_Init_Value -(_state._motor_data[i].q - _state.Motor_Init_Angle[i]);
                            break;
                        case 10:
                            _state._motor_data[i].dq = _state._motor_data[i].dq / 6.33;
                            _state.Motor_Angle[i]=
                            _state.Angle_Initialization_Variable.Left_Back_Thigh_Angle_Init_Value +(_state._motor_data[i].q - _state.Motor_Init_Angle[i]);
                            break;
                        case 11:
                            _state._motor_data[i].dq = -_state._motor_data[i].dq / 6.33;
                            _state.Motor_Angle[i]=
                            _state.Angle_Initialization_Variable.Left_Back_Calf_Angle_Init_Value -(_state._motor_data[i].q - _state.Motor_Init_Angle[i]);
                            break;
                    }
                    //  #ifdef LOWCMD_DEBUG
                    // // syslog(LOG_INFO,"*****22222 id: %d -- _state._motor_data[i].q: %.2f -- tmp_cmd.q:%.2f -- "
                    // // ,i,Radian2Angle(_state._motor_data[i].q),Radian2Angle(tmp_cmd[i].q/6.33));
                    // syslog(LOG_INFO,"cmd: id: %d -- cmd_q: %.2f -- tmp_cmd.q: %.2f -- dq: %.2f -- kp: %.2f -- kd: %.2f -- tau: %.3f"
                    //     ,i,Radian2Angle(_cmd[i].q),Radian2Angle(tmp_cmd[i].q/6.33),Radian2Angle(_cmd[i].dq),_cmd[i].kp,_cmd[i].kd,_cmd[i].tau);
                    // #endif
                    #ifdef LOWSTATE_DEBUG
                       
                        syslog(LOG_INFO,"state: id:%d  -- tau:%.3f",i,_state._motor_data[i].tau);
                        
                        // syslog(LOG_INFO,"state: id:%d -- 最初q:%.2f -- 原始q:%.2f -- 归一q:%.2f -- 角速度w:%.2f -- tua:%.3f"
                        //     ,i,Radian2Angle(_state.Motor_Init_Angle[i]),Radian2Angle(_state._motor_data[i].q),_state.Motor_Angle[i],_state._motor_data[i].dq,_state._motor_data[i].tau);
                    #endif
                }   
                
            }
        }

        void SetQ(Eigen::Matrix<float,12,1> q)// 
        {
            for(int i(0); i<12; ++i){
                _cmd[i].q = q(i);
            }
        }
        void SetQ(int id,float q)       // 单独控制某一电机角度
        {
            _cmd[id].q = q;
        }

        void SetQ(int leg_id , Eigen::Matrix<float,3,1> q)
        {  
                _cmd[3*leg_id +0 ].q = q(0);
                _cmd[3*leg_id +1 ].q = q(1);
                _cmd[3*leg_id +2 ].q = q(2); 
        }

        void SetDq(int leg_id,Eigen::Matrix<float,3,1> dq)
        {
            _cmd[3*leg_id +0 ].dq = dq(0);
            _cmd[3*leg_id +1 ].dq = dq(1);
            _cmd[3*leg_id +2 ].dq = dq(2); 
        }

        void SetDq(Eigen::Matrix<float,12,1> dq)  
        {  
            for(int i=0;i<12;i++)
            {
                _cmd[i].dq = dq(i);
            }
        }

        void SetP(int leg_id,Eigen::Matrix<float,3,1> p)
        {
            _cmd[3*leg_id +0 ].kp = p(0);
            _cmd[3*leg_id +1 ].kp = p(1);
            _cmd[3*leg_id +2 ].kp = p(2); 
        }

        void SetP(Eigen::Matrix<float,12,1> p)  
        {  
            for(int i=0;i<12;i++)
            {
                _cmd[i].kp = p(i);
            }
        }

        void SetZeroP(){
            for(int i(0); i<4; ++i){
                _cmd[3*i+0].kp = 0;
                _cmd[3*i+1].kp = 0;
                _cmd[3*i+2].kp = 0;
            }
        }

        void SetD(int leg_id,Eigen::Matrix<float,3,1> d)
        {
            _cmd[3*leg_id +0 ].kd = d(0);
            _cmd[3*leg_id +1 ].kd = d(1);
            _cmd[3*leg_id +2 ].kd = d(2); 
        }

        void SetD(Eigen::Matrix<float,12,1> d)  
        {  
            for(int i=0;i<12;i++)
            {
                _cmd[i].kd = d(i);
            }
        }

        void SetZeroD(){
            for(int i(0); i<4; ++i){
                _cmd[3*i+0].kd = 0;
                _cmd[3*i+1].kd = 0;
                _cmd[3*i+2].kd = 0;
            }
        }

        void SetTau(Eigen::Matrix<float,12,1> tau, Eigen::Matrix<double,2,1> torqueLimit = Eigen::Matrix<double,2,1>(-1.3, 1.3)){
            for(int i(0); i<12; ++i){
                if(std::isnan(tau(i))){
                    printf("[ERROR] The setTau function meets Nan\n");
                }
                _cmd[i].tau = saturation(tau(i), torqueLimit);
            }
        }

        void SetTau(int leg_id,Eigen::Matrix<float,3,1> tau){
            _cmd[leg_id*3+0].tau = tau(0);
            _cmd[leg_id*3+1].tau = tau(1);
            _cmd[leg_id*3+2].tau = tau(2);
            
        }

        void SetZeroTau(int legID){
            _cmd[legID*3+0].tau = 0;
            _cmd[legID*3+1].tau = 0;
            _cmd[legID*3+2].tau = 0;
        }

        void SetZeroTau(){
            for(uint8_t i=0;i<4;i++)
            {
                _cmd[i*3+0].tau = 0;
                _cmd[i*3+1].tau = 0;
                _cmd[i*3+2].tau = 0;
            }
            
        }

        void SetZeroDq(int legID){
            _cmd[legID*3+0].dq = 0;
            _cmd[legID*3+1].dq = 0;
            _cmd[legID*3+2].dq = 0;
        }

        void SetZeroDq(){
            for(int i(0); i<4; ++i){
                SetZeroDq(i);
            }
        }

        void SetFree()
        {
            SetZeroDq();
            SetZeroTau();
        }
        
        ~LowCmd() {
            if (serial) {
                delete serial;
                serial = nullptr;
            }
        }
    // private:
        SerialPort * serial;
        MotorCmd _cmd[12];
        LowState _state;        // LowState类里 的数据是数组
};


#endif