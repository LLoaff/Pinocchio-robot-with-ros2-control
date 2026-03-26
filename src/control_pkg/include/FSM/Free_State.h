#ifndef FREE_STATE_H
#define FREE_STATE_H

#include "ControlComponent.h"
#include "FSMState.h"
#include "EnumClassList.h"
class Free_State : public FSMState
{
public:
    Free_State(ControlComponent * free_ctrl_comp);
    void enter();
    void run();
    void exit();
    FSMStateName CheckChange();
private:

};

Free_State::Free_State(ControlComponent * free_ctrl_comp):FSMState(free_ctrl_comp,FSMStateName::FREE,"free"){}

void Free_State::enter()
{
    Eigen::Matrix<float,3,1> d,q;
    d<< 0.0 , 0.0 , 0.0;
    q<<  0 , 0 , 0;

    for(int i=0;i<4;i++)
    {
        _fsm_state_lowcmd->SetD(i,d);   
        _fsm_state_lowcmd->SetZeroDq(i);
        _fsm_state_lowcmd->SetZeroTau(i);
        _fsm_state_lowcmd->SetZeroP();
        _fsm_state_lowcmd->SetQ(i,q);
    }
    _fsm_state_ctrl_comp->setAllSwing();
}

void Free_State::run()
{
    // Eigen::Matrix<float,3,3> a;
    // Eigen::Matrix<float,3,1> b;
    // Eigen::Matrix<float,3,1> q;
    // Eigen::Matrix<float,3,1> w;
    // q<< _fsm_state_lowstate->Motor_Angle[6],_fsm_state_lowstate->Motor_Angle[7],_fsm_state_lowstate->Motor_Angle[8];
    // w<< _fsm_state_lowstate->_motor_data[6].dq,_fsm_state_lowstate->_motor_data[7].dq,_fsm_state_lowstate->_motor_data[8].dq;
    // _fsm_state_ctrl_comp->CalTua(2,q,w,a,a,b,b);

    // Eigen::Matrix<float,3,1> pos_w;
    // pos_w=Pos_Speed(2,_fsm_state_lowstate->Motor_Angle[6],_fsm_state_lowstate->Motor_Angle[7],_fsm_state_lowstate->Motor_Angle[8],
    //           _fsm_state_lowstate->_motor_data[6].dq,_fsm_state_lowstate->_motor_data[7].dq,_fsm_state_lowstate->_motor_data[8].dq );
    // syslog(LOG_INFO,"speed: -- x_s:%.6f -- y_s:%.6f -- z_s:%.6f ",pos_w(0), pos_w(1),pos_w(2));
    // std::cout << "free is run" << std::endl;
}

void Free_State::exit()
{

}

FSMStateName Free_State::CheckChange()
{
    UserValue user = _fsm_state_ctrl_comp->user_cmd->GetUserValue();
    if( user == UserValue::STAND )
        return FSMStateName::STAND ;
    else if(user == UserValue::FREE)
        return FSMStateName::FREE;
    else if( user == UserValue::PASSIVE)
        return FSMStateName::PASSIVE;

    return FSMStateName::FREE;
}

#endif