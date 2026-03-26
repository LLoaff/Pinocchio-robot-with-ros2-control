#ifndef TROTTING_STATE_H
#define TROTTING_STATE_H

#include "ControlComponent.h"
#include "FSMState.h"
#include "EnumClassList.h"
#include "Gait/GaitGenerator.h"
#include "Reversal_solution.h"

class Trotting_State : public FSMState{
public:
    Trotting_State(ControlComponent * ctrlComp);
    ~Trotting_State();
    void enter();
    void run();
    void exit();
    virtual FSMStateName CheckChange();
    void setHighCmd(double vx, double vy, double wz);

private:
    void calcTau();
    void calcQQd();
    void calcCmd();
    virtual void getUserCmd();
    void calcBalanceKp();
    bool checkStepOrNot();

    GaitGenerator *_gait;
    Estimator *_est;
    QuadrupedRobot *_robModel;
    BalanceCtrl *_balCtrl;

    Vec3    _posBody, _velBody;
    double  _yaw, _dYaw;
    Vec34   _posFeetGlobal, _velFeetGlobal;                 // 足端位置  足端速度
    Vec34   _posFeet2BGlobal;                               // 足端位置+质心位置
    RotMat  _B2G_RotMat, _G2B_RotMat;
    Vec12   _q;

    Vec3    _pcd;                                           // 存储xyz上的位移
    Vec3    _vCmdGlobal, _vCmdBody;
    double  _yawCmd, _dYawCmd;                              // _yawCmd:累积转过的角度  _dYawCmd:遥控器控制的期望转动速度
    double  _dYawCmdPast;
    Vec3    _wCmdGlobal;
    Vec34   _posFeetGlobalGoal, _velFeetGlobalGoal;
    Vec34   _posFeet2BGoal, _velFeet2BGoal;
    RotMat  _Rd;                                            // 存储当前旋转矩阵
    Vec3    _ddPcd, _dWbd;
    Vec34   _forceFeetGlobal, _forceFeetBody;
    Vec34   _qGoal, _qdGoal;
    Vec12   _tau;

    double  _gaitHeight;
    Vec3    _posError, _velError;
    Mat3    _Kpp, _Kdp, _Kdw;
    double  _kpw;
    Mat3    _KpSwing, _KdSwing;
    Vec2    _vxLim, _vyLim, _wyawLim;
    Vec4 *  _phase;
    VecInt4 *_contact;
    Vec4 _userValue;
    LowState* _lowstate;
};

Trotting_State::Trotting_State(ControlComponent * ctrlComp):FSMState(ctrlComp,FSMStateName::TROTTING,"trotting"),
_est(ctrlComp->_estimator),_phase(ctrlComp->_phase),_contact(ctrlComp->_contact),_robModel(ctrlComp->robotModel),_balCtrl(ctrlComp->balCtrl),_lowstate(&ctrlComp->_ctrl_cmd->_state){
    _gait = new GaitGenerator(ctrlComp);
    _gaitHeight = 0.06;

    _Kpp = Vec3(20, 20, 100).asDiagonal();
    _Kdp = Vec3(20, 20, 20).asDiagonal();
    _kpw = 200;
    _Kdw = Vec3(20, 20, 20).asDiagonal();

    _KpSwing = Vec3(40, 40, 40).asDiagonal();
    _KdSwing = Vec3(5, 5, 5).asDiagonal();

    _vxLim = _robModel->getRobVelLimitX();
    _vyLim = _robModel->getRobVelLimitY();
    _wyawLim = _robModel->getRobVelLimitYaw();
}

void Trotting_State::enter(){
    _pcd = _est->getPosition();
    _pcd(2) = -_robModel->getFeetPosIdeal()(2, 0); // _pcd(2)质心高度， -_robModel->getFeetPosIdeal()写死了的站立姿态 (2,0)高度数据
    _vCmdBody.setZero();
    _yawCmd = _lowstate->_imu.getYaw();
    _Rd = rotz((float)_yawCmd).cast<double>();            // _yawCmd存的是当前机身朝向角
    _wCmdGlobal.setZero();

    // _ctrlComp->ioInter->zeroCmdPanel(); // 遥控器userValue清零

    _gait->restart();
}

void Trotting_State::exit(){
    // _fsm_state_ctrl_comp->ioInter->zeroCmdPanel(); // 遥控器userValue清零
    _fsm_state_ctrl_comp->setAllSwing();
}

FSMStateName Trotting_State::CheckChange(){
    UserValue user = _fsm_state_ctrl_comp->user_cmd->GetUserValue();
    if( user == UserValue::PASSIVE)
        return FSMStateName::PASSIVE;
    else if ( user == UserValue::STAND)
        return FSMStateName::STAND;
    return FSMStateName::TROTTING;
}

Trotting_State::~Trotting_State(){
    delete _gait;
}

void Trotting_State::run(){
    _posBody = _est->getPosition();
    _velBody = _est->getVelocity();
    _posFeet2BGlobal = _est->getPosFeet2BGlobal(); 
    _posFeetGlobal = _est->getFeetPos();
    _velFeetGlobal = _est->getFeetVel();
    _B2G_RotMat = _lowstate->_imu.GetRotMat().cast<double>();
    _G2B_RotMat = _B2G_RotMat.transpose();
    _yaw = _lowstate->_imu.getYaw() ;
    _dYaw = _lowstate->_imu.getDYaw();

/* 遥控归一 */
    float ly = (_fsm_state_ctrl_comp->user_cmd->R_Data.ch3 - 1024) / 660.0;
    float lx = (_fsm_state_ctrl_comp->user_cmd->R_Data.ch2 - 1024) / 660.0;
    float ry = (_fsm_state_ctrl_comp->user_cmd->R_Data.ch1 - 1024) / 660.0;
    float rx = (_fsm_state_ctrl_comp->user_cmd->R_Data.ch0 - 1024) / 660.0;
    _userValue(0) = ly;
    _userValue(1) = lx;
    _userValue(2) = ry;
    _userValue(3) = rx;

    getUserCmd(); // 计算 期望速度
    calcCmd();    // 计算位移、转动角度，获取全局速度
            //      全局 v                      全局角速度        抬腿高度
    _gait->setGait(_vCmdGlobal.segment(0,2), _wCmdGlobal(2), _gaitHeight);
    _gait->run(_posFeetGlobalGoal, _velFeetGlobalGoal); // 生成下一刻的 足端坐标 、速度 (global)
    // std::cout<<"next_pos:\n"<< _posFeetGlobalGoal<<std::endl;

    // for(int i(0); i<4; ++i){
    //     _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _posBody);
    //     _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody); 
    // }
    // std::cout<<"next_pos:\n"<< _posFeet2BGoal<<std::endl;
    // calcTau();
    // calcQQd(); // 计算机身坐标系下 足端坐标、速度
    // std::cout<<"_q:\n"<< _qGoal<< "\n"<<std::endl;
    // std::cout<<"_dq:\n"<< _qdGoal<< "\n"<<std::endl;
    if(checkStepOrNot()){
        _fsm_state_ctrl_comp->setStartWave();
    }else{
        _fsm_state_ctrl_comp->setAllStance();
    }
    // _fsm_state_ctrl_comp->_ctrl_cmd->SetTau(_tau);
    // _fsm_state_ctrl_comp->_ctrl_cmd->SetQ(vec34ToVec12(_qGoal));
    // _fsm_state_ctrl_comp->_ctrl_cmd->SetDq(vec34ToVec12(_qdGoal));
    Eigen::Matrix<float,3,1> kp,kd;

    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 0){
            kp<< 2.0 , 2.0 , 3.5;
            kd<< 0.1 , 0.1 , 0.1;
            // _fsm_state_lowcmd->SetP(i,kp);
            // _fsm_state_lowcmd->SetD(i,kd);
        }else{
            kp<< 0.5 , 0.8 , 0.8;
            kd<< 0.1 , 0.1 , 0.1;
            // _fsm_state_lowcmd->SetP(i,kp);
            // _fsm_state_lowcmd->SetD(i,kd);
        }
    }
}

bool Trotting_State::checkStepOrNot(){
    if( (fabs(_vCmdBody(0)) > 0.03) ||
        (fabs(_vCmdBody(1)) > 0.03) ||
        (fabs(_posError(0)) > 0.08) ||
        (fabs(_posError(1)) > 0.08) ||
        (fabs(_velError(0)) > 0.05) ||
        (fabs(_velError(1)) > 0.05) ||
        (fabs(_dYawCmd) > 0.20) ){
        return true;
    }else{
        return false;
    }
}

void Trotting_State::setHighCmd(double vx, double vy, double wz){
    _vCmdBody(0) = vx;
    _vCmdBody(1) = vy;
    _vCmdBody(2) = 0; 
    _dYawCmd = wz;
}
// 期望速度 _vCmdBody：OK  全局期望速度 _vCmdGlobal：OK  全局期望角速度_wCmdGlobal：OK
// 质心位移 _pcd：OK
void Trotting_State::getUserCmd(){
    /* Movement */
    _vCmdBody(0) =  invNormalize(_userValue(0), _vxLim(0), _vxLim(1));// 换算x上期望速度
    _vCmdBody(1) = -invNormalize(_userValue(1), _vyLim(0), _vyLim(1));// 换算y上期望速度
    _vCmdBody(2) = 0;

    /* Turning */
    _dYawCmd = -invNormalize(_userValue(3), _wyawLim(0), _wyawLim(1));// 换算转动期望速度
    _dYawCmd = 0.9*_dYawCmdPast + (1-0.9) * _dYawCmd;                 // 低通滤波
    _dYawCmdPast = _dYawCmd;

    // std::cout<< "_vCmdBody: \n"<< _vCmdBody << std::endl;   
    // std::cout<< "_dYawCmd: "<< _dYawCmd << std::endl;
}

void Trotting_State::calcCmd(){
    /* Movement */
    _vCmdGlobal = _B2G_RotMat * _vCmdBody;
    // _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2(_velBody(0)-0.2, _velBody(0)+0.2)); // 对全局期望速度限幅
    // _vCmdGlobal(1) = saturation(_vCmdGlobal(1), Vec2(_velBody(1)-0.2, _velBody(1)+0.2));
    _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2(-0.15, 0.15));
    _vCmdGlobal(1) = saturation(_vCmdGlobal(1), Vec2(-0.9, 0.9));

    // std::cout<< "_velBody: \n"<< _velBody << std::endl;
    // std::cout<< "_vCmdGlobal: \n"<< _vCmdGlobal << std::endl;

    // _pcd(0) + _vCmdGlobal(0) * _fsm_state_ctrl_comp->dt:对速度积分、累加质心位置，算位移
    // 再对位移限幅
    _pcd(0) = saturation(_pcd(0) + _vCmdGlobal(0) * _fsm_state_ctrl_comp->dt, Vec2(_posBody(0) - 0.05, _posBody(0) + 0.05));
    _pcd(1) = saturation(_pcd(1) + _vCmdGlobal(1) * _fsm_state_ctrl_comp->dt, Vec2(_posBody(1) - 0.05, _posBody(1) + 0.05));

    _vCmdGlobal(2) = 0;

    /* Turning */
    _yawCmd = _yawCmd + _dYawCmd * _fsm_state_ctrl_comp->dt;// 计算累积角度

    _Rd = rotz(_yawCmd).cast<double>();
    _wCmdGlobal(2) = _dYawCmd;

    // std::cout<< "_B2G_RotMat: \n"<< _B2G_RotMat << std::endl;
    // std::cout<< "_vCmdBody: \n"<< _vCmdBody << std::endl;
    // std::cout<< "_vCmdGlobal: \n"<< _vCmdGlobal << std::endl;
    // std::cout<< "_wCmdGlobal:\n"<<_wCmdGlobal<<std::endl;
    // std::cout<< "_pcd:\n"<<_pcd<<std::endl;
    // std::cout<< "_yawCmd:"<<_yawCmd<<std::endl;
}

void Trotting_State::calcTau(){
    _posError = _pcd - _posBody;
    _velError = _vCmdGlobal - _velBody;

    _ddPcd = _Kpp * _posError + _Kdp * _velError;
    _dWbd  = _kpw*rotMatToExp(_Rd*_G2B_RotMat) + _Kdw * (_wCmdGlobal - _lowstate->_imu.getGyroGlobal().cast<double>());

    _ddPcd(0) = saturation(_ddPcd(0), Vec2(-3, 3));
    _ddPcd(1) = saturation(_ddPcd(1), Vec2(-3, 3));
    _ddPcd(2) = saturation(_ddPcd(2), Vec2(-5, 5));

    _dWbd(0) = saturation(_dWbd(0), Vec2(-40, 40));
    _dWbd(1) = saturation(_dWbd(1), Vec2(-40, 40));
    _dWbd(2) = saturation(_dWbd(2), Vec2(-10, 10));

    _forceFeetGlobal = - _balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact);

    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 0){
            _forceFeetGlobal.col(i) = _KpSwing*(_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) + _KdSwing*(_velFeetGlobalGoal.col(i)-_velFeetGlobal.col(i));
        }
    }

    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;
    _q = vec34ToVec12(_lowstate->getQ()).cast<double>();
    _tau = _fsm_state_ctrl_comp->getTau(_q.cast<float>(), _forceFeetBody);
}

void Trotting_State::calcQQd(){

    Vec34 _posFeet2B;
    _posFeet2B = GetFeetPos2BODY(*_lowstate,FrameType::BODY);
    
    for(int i(0); i<4; ++i){
        _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _posBody);
        _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody); 
    }
    
    _qGoal = vec12ToVec34(Reversal_GetQ(_posFeet2BGoal, FrameType::BODY));
    _qdGoal = vec12ToVec34(Reversal_GetQd(_posFeet2B, _velFeet2BGoal, FrameType::BODY));
}


#endif