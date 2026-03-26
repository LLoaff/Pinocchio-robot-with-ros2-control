#ifndef STAND_STATE_H
#define STAND_STATE_H

#include "ControlComponent.h"
#include "FSMState.h"
#include "EnumClassList.h"
#include "Reversal_solution.h"
#include "BalanceCtrl.h"

class Stand_State : public FSMState
{
public:
    Stand_State(ControlComponent * stand_ctrl_comp);
    void enter();
    void run();
    void exit();
    FSMStateName CheckChange();

private:
    Eigen::Matrix<float,12,1>    _target_xyz;
    Eigen::Matrix<float,12,1>    _start_xyz;
    Eigen::Matrix<float,12,1>    _target_speed;
    Eigen::Matrix<float,12,1>    _target_angle;
    Eigen::Matrix<float,12,1>    _start_angle;
    BalanceCtrl*                _balance;
    Eigen::Matrix<int,4,1>*     _conact;
    Eigen::Matrix<float,3,3>    _KP;
    Eigen::Matrix<float,3,3>    _KD;
    float                       _duration = 70;
    float                       _percent  = 0;
};

Stand_State::Stand_State(ControlComponent * stand_ctrl_comp):FSMState(stand_ctrl_comp,FSMStateName::STAND,"stand"),
_balance(stand_ctrl_comp->balCtrl),_conact(stand_ctrl_comp->_contact){}

void Stand_State::enter()
{
    Eigen::Matrix<float,3,1> dq,kp,kd,tau,speed;
    kp<< 2.0 , 2.0 , 3.5;
    kd<< 0.1 , 0.1 , 0.1;
    dq<< 30/180*M_PI, 30/180*M_PI, 30/180*M_PI;
    speed<< 0.07,0.07,0.1;
    tau<< 0 , 0 ,0;
    _target_xyz << 0,-0.12,-0.365,
                   0, 0.12,-0.365,
                   0,-0.12,-0.365,
                   0, 0.12,-0.365;
    _KP<< 8.0,  0 ,   0,
          0 ,   8.0,  0,
          0 ,   0 ,   9.5;

    _KD<< 0.5, 0,    0,
          0,   0.5,  0,
          0,   0,    0.5;
    for(int i=0;i<4;i++)
    {   

        _fsm_state_lowcmd->SetP(i,kp);
        _fsm_state_lowcmd->SetD(i,kd);
        _fsm_state_lowcmd->SetDq(i,dq);
        _fsm_state_lowcmd->SetTau(i,tau);
        _target_speed.segment(3*i,3) = speed;
        _target_angle.segment(3*i,3) =  Reversal_Solution_Update(i,_target_xyz(3*i+0),_target_xyz(3*i+1),_target_xyz(3*i+2));
        _start_angle(3*i+0)  =  _fsm_state_lowstate->Motor_Angle[i*3+0];
        _start_angle(3*i+1)  =  _fsm_state_lowstate->Motor_Angle[i*3+1];
        _start_angle(3*i+2)  =  _fsm_state_lowstate->Motor_Angle[i*3+2];
    }
    // std::cout<< "_target_angle: \n" << _target_angle<< std::endl;
    _start_xyz =vec34ToVec12(GetFeetPos2BODY(*_fsm_state_lowstate,FrameType::HIP).cast<float>());
    _fsm_state_ctrl_comp->setAllStance();
}

void Stand_State::run(){
    Eigen::Matrix<float,12,1> target_q;
    Eigen::Matrix<float,12,1> q;
    Eigen::Matrix<float,12,1> pos;
    Eigen::Matrix<float,12,1> tau;
    Eigen::Matrix<float,12,1> w;

    // 线性插值算法
    _percent += (float)1/_duration;
    _percent = _percent>1 ? 1 :  _percent;

    w = _fsm_state_ctrl_comp->_ctrl_cmd->_state.getW12();
    q = _fsm_state_ctrl_comp->_ctrl_cmd->_state.getQ12();
    pos= (1-_percent)*_start_xyz + _percent*_target_xyz;
    target_q = (1-_percent)*_start_angle + _percent*_target_angle;

    tau =_fsm_state_ctrl_comp->CalTaus(q,w,_KP,_KD,_target_xyz,_target_speed,FrameType::HIP);

    // if(_percent != 1){
        // std::cout<< "target_q:\n"<< target_q/M_PI*180 <<"---\n"<<std::endl;
        // std::cout<< "tau:"<< tau <<"---"<<std::endl;
    // }
    // std::cout<< "tau:\n"<< tau <<"---\n"<<std::endl;
    
    _fsm_state_lowcmd->SetQ(target_q);
    _fsm_state_lowcmd->SetTau(tau); 

}

void Stand_State::exit(){
    _percent = 0;
}

FSMStateName Stand_State::CheckChange(){
    UserValue user = _fsm_state_ctrl_comp->user_cmd->GetUserValue();
    if( user == UserValue::PASSIVE)
        return FSMStateName::PASSIVE;
    else if ( user == UserValue::TROTTING)
        return FSMStateName::TROTTING;
    else if ( user == UserValue::FREE_STAND)
        return FSMStateName::FREE_STAND;
    return FSMStateName::STAND;
}

#endif