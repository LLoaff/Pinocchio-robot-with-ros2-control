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
        _fstate_ctrl->_ioros->SetD(i,d);   
        _fstate_ctrl->_ioros->SetZeroDq(i);
        _fstate_ctrl->_ioros->SetZeroTau(i);
        _fstate_ctrl->_ioros->SetZeroP();
        _fstate_ctrl->_ioros->SetQ(i,q);
    }
    _fstate_ctrl->setAllSwing();
}

void Free_State::run()
{
    std::cout << "free is run" << std::endl;
}

void Free_State::exit()
{

}

FSMStateName Free_State::CheckChange()
{
    UserValue user = _fstate_ctrl->user_cmd->GetUserValue();
    if( user == UserValue::STAND )
        return FSMStateName::STAND ;
    else if(user == UserValue::FREE)
        return FSMStateName::FREE;
    else if( user == UserValue::PASSIVE)
        return FSMStateName::PASSIVE;

    return FSMStateName::FREE;
}

#endif