#ifndef FSM_H
#define FSM_H

#include "ControlComponent.h"
#include "EnumClassList.h"
#include "Passive_State.h"
#include "Free_State.h"
#include "Stand_State.h"
#include "Free_Stand_State.h"
#include "Balance_State.h"
#include "FSMState.h"
#include "TimeMaker.h"
#include "Trotting_State.h"

struct FSMStateList
{
    FSMState      *     invalid;
    Passive_State *     passive;
    Free_State    *     free;
    Stand_State   *     stand;
    Free_Stand_State *  free_stand;
    Balance_State*      balance;
    Trotting_State*     trotting;
};

class FSM
{
public:
    FSM(ControlComponent *_ctrlcomp);
    ~FSM();
    void initialize();
    void run();
private:
    FSMState         * GetNextState(FSMStateName fsm_state_name);

    ControlComponent * _fsm_ctrl;
    FSMStateList       _fsm_state_list;
    FSMState         * _current_state;
    FSMState         * _next_state;
    FSMStateName       _next_state_name; // 下个状态的enum名
    FSMMode            _mode;       // 判断切换还是正常
    long long          _start_time;  
    int                _count;
};

FSM::FSM(ControlComponent *_ctrlcomp):_fsm_ctrl(_ctrlcomp)
{
    _fsm_state_list.invalid     = nullptr;
    _fsm_state_list.passive     = new Passive_State(_ctrlcomp);
    _fsm_state_list.free        = new Free_State(_ctrlcomp);
    _fsm_state_list.stand       = new Stand_State(_ctrlcomp);
    _fsm_state_list.free_stand  = new Free_Stand_State(_ctrlcomp);
    // _fsm_state_list.balance     = new Balance_State(_ctrlcomp,&_fsm_ctrl->_ctrl_cmd->_state);
    // _fsm_state_list.trotting    = new Trotting_State(_ctrlcomp);

    initialize();
}

void FSM::initialize()
{
    _current_state = _fsm_state_list.passive;
    _current_state->enter();
    _next_state = _current_state;
    _mode = FSMMode::NORMAL;
}

void FSM::run()
{
    _start_time = getSystemTime();
    _fsm_ctrl->_ioros->upDate();        // 对电机发送命令
    // _fsm_ctrl->runWaveGen();
    // _fsm_ctrl->_estimator->run();
    if(_mode == FSMMode::NORMAL)
    {
        _current_state->run();          // 当前 状态执行一次run 

        _next_state_name = _current_state->CheckChange();
        if(_next_state_name != _current_state->_state_name)
        {
            _mode = FSMMode::CHANGE;
            _next_state = GetNextState(_next_state_name);       // GetNextState函数会根据_next_state_name的enum值，返回 _fsm_state_list下不同的地址
        }
    }

    else if(_mode == FSMMode::CHANGE)
    {
        _current_state->exit();
        _current_state = _next_state;
        _current_state->enter();
        _mode = FSMMode::NORMAL;
        _current_state->run();
    }
    absoluteWait(_start_time, (long long)(_fsm_ctrl->dt * 1000000));       // dt 需初始化时手动赋值
}

FSMState* FSM::GetNextState(FSMStateName fsm_state_name)
{

    switch (fsm_state_name)
    {
    case FSMStateName::INVALID:
        return _fsm_state_list.invalid;
    break;
    case FSMStateName::PASSIVE:
        return _fsm_state_list.passive;
    case FSMStateName::FREE:
        return _fsm_state_list.free;
    case FSMStateName::STAND:
        return _fsm_state_list.stand;
    case FSMStateName::FREE_STAND:
        return _fsm_state_list.free_stand;
    // case FSMStateName::BALANCE :
    //     return _fsm_state_list.balance;
    case FSMStateName::TROTTING:
        return _fsm_state_list.trotting;
    default:
        return _fsm_state_list.invalid;
    break;
    }
}

FSM::~FSM()
{
    delete _fsm_state_list.invalid;
    delete _fsm_state_list.passive;
    delete _fsm_state_list.free;
    delete _fsm_state_list.stand;
    delete _fsm_state_list.free_stand;
    // delete _fsm_state_list.balance;
    // delete _fsm_state_list.trotting;
}







#endif