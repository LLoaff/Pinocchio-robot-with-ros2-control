#include "FSM/Trotting_State.h"

Trotting_State::Trotting_State(ControlComponent * ctrlComp):FSMState(ctrlComp,FSMStateName::TROTTING,"trotting"),
_est(ctrlComp->_estimator),_phase(ctrlComp->_phase),_contact(ctrlComp->_contact),_robModel(ctrlComp->robotModel),_lowstate(&ctrlComp->_ioros->_state),_balCtrl(ctrlComp->balCtrl){
    _gait = new GaitGenerator(ctrlComp);
    _gaitHeight = 0.15;

    _vxLim = _robModel->getRobVelLimitX();
    _vyLim = _robModel->getRobVelLimitY();
    _wyawLim = _robModel->getRobVelLimitYaw();

    _Kpp = Vec3(400, 400, 400).asDiagonal();
    _Kdp = Vec3(10, 10, 10).asDiagonal();
    _kpw = 400; 
    _Kdw = Vec3(10, 10, 10).asDiagonal();

    _KpSwing = Vec3(20, 20, 20).asDiagonal();
    _KdSwing = Vec3(1, 1, 1).asDiagonal();

    // _KP<< 100.0,  0 ,   0,
    //       0 ,   100.0,  0,
    //       0 ,   0 ,   100;

    // _KD<< 30, 0,    0,
    //       0,   30,  0,
    //       0,   0,   30;
    _KP<< 8,  0 ,   0,
          0 ,   8,  0,
          0 ,   0 ,   8;

    _KD<< 0.7, 0,    0,
          0,   0.7,  0,
          0,   0,   0.7;
}

void Trotting_State::enter(){
 
    _pcd = _est->getPosition() ;
    _pcd(2) = -_robModel->getFeetPosIdeal()(2, 0);

    _vCmdBody.setZero();
    _wCmdGlobal.setZero();

    _gait->restart();
}

void Trotting_State::exit(){
    _fstate_ctrl->setAllSwing();
}

FSMStateName Trotting_State::CheckChange(){
    UserValue user = _fstate_ctrl->user_cmd->GetUserValue();
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
    _G2B_RotMat = rotx(_lowstate->_imu.getRoll()).cast<double>()*roty(_lowstate->_imu.getPitch()).cast<double>();
    _yaw = _lowstate->_imu.getYaw() ;
    _dYaw = _lowstate->_imu.getDYaw();

/* 遥控归一 */
    float ly = (_fstate_ctrl->user_cmd->R_Data.ch3 - 1024) / 660.0;
    float lx = (_fstate_ctrl->user_cmd->R_Data.ch2 - 1024) / 660.0;
    float ry = (_fstate_ctrl->user_cmd->R_Data.ch1 - 1024) / 660.0;
    float rx = (_fstate_ctrl->user_cmd->R_Data.ch0 - 1024) / 660.0;
    _userValue(0) = ly;
    _userValue(1) = lx;
    _userValue(2) = ry;
    _userValue(3) = rx;

    getUserCmd(); // 计算 期望速度
    calcCmd();    // 计算位移、转动角度，获取全局速度
    //         //      全局 v                      全局角速度        抬腿高度
    _gait->setGait(_vCmdGlobal.segment(0,2), _wCmdGlobal(2), _gaitHeight);
    _gait->run(_posFeetGlobalGoal, _velFeetGlobalGoal); // 生成下一刻的 足端坐标 、速度 (global)
    // std::cout<<"_posFeetGlobalGoal:\n"<< _posFeetGlobalGoal <<std::endl;
    // std::cout<<"_velFeetGlobalGoal:\n"<< _velFeetGlobalGoal <<std::endl;

   
    // std::cout<<"next_pos:\n"<< _posFeet2BGoal<<std::endl;
    // calcTau();
    calcQQd(); // 计算机身坐标系下 足端坐标、速度
    // _qGoal = vec12ToVec34 ( Reversal_GetQ (_posFeetGlobalGoal, FrameType::BODY));
    // std::cout<<"_pos:\n"<< _posFeetGlobalGoal<< "\n"<<std::endl;
    // sleep(1);
    // std::cout<<"_dq:\n"<< _qdGoal<< "\n"<<std::endl;
    if(checkStepOrNot()){
        _fstate_ctrl->setStartWave();
    }else{
        _fstate_ctrl->setAllStance();
    }
    // _fstate_ctrl->_ioros->SetTau(_tau.cast<float>());
    _fstate_ctrl->_ioros->SetQ(vec34ToVec12 (_qGoal.cast<float>()));
    // _fstate_ctrl->_ioros->SetDq(vec34ToVec12 (_qdGoal.cast<float>()));

    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 0){
            _fstate_ctrl->_ioros->setSwingGain(i);
        }else{
            _fstate_ctrl->_ioros->setStableGain(i);
        }
    }
}

bool Trotting_State::checkStepOrNot(){
    // if( (fabs(_vCmdBody(0)) > 0.03) ||
    //     (fabs(_vCmdBody(1)) > 0.03) ||
    //     (fabs(_posError(0)) > 0.08) ||
    //     (fabs(_posError(1)) > 0.08) ||
    //     (fabs(_velError(0)) > 0.08) ||
    //     (fabs(_velError(1)) > 0.08) ||
    //     (fabs(_dYawCmd) > 0.05) ){
    //     return true;
    // }else{
    //     return false;
    // }
    if( (fabs(_vCmdBody(0)) > 0.03) ||
        (fabs(_vCmdBody(1)) > 0.03) ||
        (fabs(_dYawCmd) > 0.05) ){
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
    _dYawCmd = 0.8*_dYawCmdPast + (1-0.8) * _dYawCmd;                 // 低通滤波
    _vCmdBody(0) = 0.8*_vxCmdPast+(1-0.8)*_vCmdBody(0);
    _vCmdBody(1) = 0.8*_vyCmdPast+(1-0.8)*_vCmdBody(1);


    _vxCmdPast=_vCmdBody(0);
    _vyCmdPast=_vCmdBody(1);
    _dYawCmdPast = _dYawCmd;

    // std::cout<< "_vCmdBody: \n"<< _vCmdBody << std::endl;   
    // std::cout<< "_dYawCmd: "<< _dYawCmd << std::endl;
}

void Trotting_State::calcCmd(){

    _vCmdGlobal = _vCmdBody;

    _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2(-0.5, 0.5));
    _vCmdGlobal(1) = saturation(_vCmdGlobal(1), Vec2(-0.5, 0.5));

    _vCmdGlobal(2) = 0;
    _wCmdGlobal(2) = _dYawCmd;

    // std::cout<< "_B2G_RotMat: \n"<< _B2G_RotMat << std::endl;
    // std::cout<< "_vCmdBody: \n"<< _vCmdBody << std::endl;
    // std::cout<< "_vCmdGlobal: \n"<< _vCmdGlobal << std::endl;
    // std::cout<< "_wCmdGlobal:\n"<<_wCmdGlobal<<std::endl;
    // std::cout<< "_pcd:\n"<<_pcd<<std::endl;
    // std::cout<< "_yawCmd:"<<_yawCmd<<std::endl;
}

void Trotting_State::calcTau(){
}

void Trotting_State::calcQQd(){

    Vec34 _posFeet2B;
    _posFeet2B = GetFeetPos2BODY(*_lowstate,FrameType::BODY);
    
    for(int i(0); i<4; ++i){
        _posFeet2BGoal.col(i) =_G2B_RotMat* _posFeetGlobalGoal.col(i);
        _velFeet2BGoal.col(i) = _G2B_RotMat* _velFeetGlobalGoal.col(i); 
        // _posFeet2BGoal.col(i) = (_posFeetGlobalGoal.col(i)- _posBody);
        // _velFeet2BGoal.col(i) =  (_velFeetGlobalGoal.col(i)- _velBody); 
        // std::cout<<"_posFeetGlobalGoal.col(i)- _posBody:\n"<< _posFeetGlobalGoal.col(i)- _posBody <<std::endl;
        // std::cout<<"_G2B_RotMat:\n"<< _G2B_RotMat <<std::endl;
    }

    // std::cout<<"_posFeetGlobalGoal:\n"<< _posFeetGlobalGoal <<std::endl;
    // std::cout<<"_posBody:\n"<< _posBody <<std::endl;

    std::cout<<"_posFeet2BGoal:\n"<< _posFeet2BGoal <<std::endl;
    // std::cout<<"_posFeetGlobalGoal:\n"<< _posFeetGlobalGoal <<std::endl;
    // std::cout<<"_posBody:\n"<< _posBody <<std::endl;

    // usleep(50000);
    // sleep(1);
    _qGoal = vec12ToVec34(Reversal_GetQ(_posFeet2BGoal, FrameType::BODY));
    _qdGoal = vec12ToVec34(Reversal_GetQd(_posFeet2B, _velFeet2BGoal, FrameType::BODY));
    _qqq = _fstate_ctrl->_ioros->getQ12();
    _www = _fstate_ctrl->_ioros->getW12();
    _tau = CalTaus(_qqq,_www,_KP,_KD,vec34ToVec12(_posFeet2BGoal.cast<float>()),vec34ToVec12(_velFeet2BGoal.cast<float>()),FrameType::BODY).cast<double>();
    // std::cout<<"_tau:\n"<< _tau <<std::endl;
    // std::cout<<"_qdGoal:\n"<< _qdGoal <<std::endl;

}
