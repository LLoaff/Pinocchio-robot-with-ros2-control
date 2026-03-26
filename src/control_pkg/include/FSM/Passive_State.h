#ifndef PASSIVE_STATE_H
#define PASSIVE_STATE_H

#include "ControlComponent.h"
#include "FSMState.h"
#include "EnumClassList.h"
#include "Kenimatics_normal_solution.h"
#include "mathtool.h"

class Passive_State : public FSMState
{
public:
    Passive_State(ControlComponent * passive_ctrl_comp);
    void enter();
    void run();
    void exit();
    FSMStateName CheckChange();

};

Passive_State::Passive_State(ControlComponent * passive_ctrl_comp):FSMState(passive_ctrl_comp,FSMStateName::PASSIVE,"Passive"){}

void Passive_State::enter()
{
    Eigen::Matrix<float,3,1> d,q;
    d<< 0.1 , 0.1 , 0.1;
    q<<  0 , 0 , 0;

    for(int i=0;i<4;i++)
    {
        _fsm_state_lowcmd->SetD(i,d);   
        _fsm_state_lowcmd->SetZeroDq();
        _fsm_state_lowcmd->SetZeroTau();
        _fsm_state_lowcmd->SetZeroP();
        _fsm_state_lowcmd->SetQ(i,q);
    }
    _fsm_state_ctrl_comp->setAllSwing();
}
void Passive_State::run(){
    // Eigen::Matrix<float,3,1> position;

    // for(int leg_id=0 ; leg_id<4;leg_id++)
    // {
       
    //     position = GetPos_H(
    //     leg_id,_fsm_state_lowstate->Motor_Angle[leg_id*3+0], _fsm_state_lowstate->Motor_Angle[leg_id*3+1],_fsm_state_lowstate->Motor_Angle[leg_id*3+2]);
    //     syslog(LOG_INFO,"leg_id: %d -- x:%.3f -- y:%.3f -- z:%.3f ",leg_id,position(0),position(1),position(2));
        
    // }
    // printf("ch0:%d ch1:%d ch2:%d ch3:%d  s1:%d  s2:%d\n"
    //     ,_fsm_state_ctrl_comp->user_cmd->R_Data.ch0,_fsm_state_ctrl_comp->user_cmd->R_Data.ch1,
    //      _fsm_state_ctrl_comp->user_cmd->R_Data.ch2,_fsm_state_ctrl_comp->user_cmd->R_Data.ch3,
    //      _fsm_state_ctrl_comp->user_cmd->R_Data.s1,_fsm_state_ctrl_comp->user_cmd->R_Data.s2);

    // std::cout << "passive is run" << std::endl;
}
void Passive_State::exit()
{
    for(int i=0;i<4;i++)
    {
        _fsm_state_lowcmd->SetZeroD();
        _fsm_state_lowcmd->SetZeroDq();
        _fsm_state_lowcmd->SetZeroTau();
        _fsm_state_lowcmd->SetZeroP();
    }
}

FSMStateName Passive_State::CheckChange()
{
    UserValue user = _fsm_state_ctrl_comp->user_cmd->GetUserValue();
    if( user == UserValue::STAND )
        return FSMStateName::STAND ;
    else if(user == UserValue::FREE)
        return FSMStateName::FREE;
    return FSMStateName::PASSIVE;
}

#endif