#ifndef FSMSTATE_H
#define FSMSTATE_H

#include "EnumClassList.h"
#include "ControlComponent.h"
#include "LowCmd.h"

class FSMState 
{
public:
    FSMState(ControlComponent *ctrl_comp,FSMStateName fsm_state_name,std::string _state_string_name);
    virtual void enter(void)=0;
    virtual void exit(void)=0;
    virtual void run(void)=0;
    virtual FSMStateName CheckChange() {return FSMStateName::INVALID;}

    FSMStateName      _state_name;
    std::string       _state_string_name;
protected:
    ControlComponent *_fsm_state_ctrl_comp;
    LowCmd           * _fsm_state_lowcmd;
    LowState         * _fsm_state_lowstate;
};

FSMState::FSMState(ControlComponent *ctrl_comp,FSMStateName fsm_state_name,std::string state_string_name)
:_fsm_state_ctrl_comp(ctrl_comp),_state_name(fsm_state_name),_state_string_name(state_string_name)
{
    _fsm_state_lowcmd = _fsm_state_ctrl_comp->_ctrl_cmd;
    _fsm_state_lowstate = &(_fsm_state_lowcmd ->getInternalState());
}

#endif