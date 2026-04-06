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
    Eigen::Matrix<float,3,1> d;
    d<< 0.1 , 0.1 , 0.1;
    Eigen::Matrix<float,12,1> q;
    for(int i(0);i<12;++i){
        q(i) = _fstate_ctrl->_ioros->_init_q(i);
    }

    for(int i=0;i<4;i++)
    {
        _fstate_ctrl->_ioros->SetD(i,d);
        _fstate_ctrl->_ioros->SetZeroDq();
        _fstate_ctrl->_ioros->SetZeroTau();
        _fstate_ctrl->_ioros->SetZeroP();
        _fstate_ctrl->_ioros->SetQ(q);
    }
    _fstate_ctrl->setAllSwing();
}
void Passive_State::run(){
//    std::cout<<"passive"<<std::endl;
}
void Passive_State::exit(){
}

FSMStateName Passive_State::CheckChange()
{
    UserValue user = _fstate_ctrl->user_cmd->GetUserValue();
    if( user == UserValue::STAND )
        return FSMStateName::STAND ;
    else if(user == UserValue::FREE)
        return FSMStateName::FREE;
    return FSMStateName::PASSIVE;
}

#endif