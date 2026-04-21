#include "FSM/Trotting_State.h"

Trotting_State::Trotting_State(ControlComponent * ctrlComp):FSMState(ctrlComp,FSMStateName::TROTTING,"trotting"),
_est(ctrlComp->_estimator),_phase(ctrlComp->_phase),_contact(ctrlComp->_contact),_robModel(ctrlComp->robotModel),_lowstate(&ctrlComp->_ioros->_state),_balCtrl(ctrlComp->balCtrl){
    _gait = new GaitGenerator(ctrlComp);
    _gaitHeight = 0.08;

    _vxLim = _robModel->getRobVelLimitX();
    _vyLim = _robModel->getRobVelLimitY();
    _wyawLim = _robModel->getRobVelLimitYaw();

    _Kpp = Vec3(400, 400, 400).asDiagonal();
    _Kdp = Vec3(10, 10, 10).asDiagonal();
    _kpw = 400; 
    _Kdw = Vec3(10, 10, 10).asDiagonal();

    _KpSwing = Vec3(20, 20, 20).asDiagonal();
    _KdSwing = Vec3(1, 1, 1).asDiagonal();

    _KP<< 180.0,  0 ,   0,
          0 ,   180.0,  0,
          0 ,   0 ,   180;

    _KD<< 7, 0,    0,
          0,   7,  0,
          0,   0,   7;
}

void Trotting_State::enter(){
 
    _pcd = _est->getPosition() ;
    _pcd(2) = -_robModel->getFeetPosIdeal()(2, 0);

    _vCmdBody.setZero();
    _wCmdGlobal.setZero();

    _yawCmd = _lowstate->_imu.getYaw();
    _Rd = rotz((float)_yawCmd).cast<double>();            // _yawCmd存的是当前机身朝向角

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
    _posBody = _est->getPosition();
    // std::cout<<"_posBody:\n"<< _posBody <<std::endl;

    _velBody = _est->getVelocity();
    _posFeet2BGlobal = _est->getPosFeet2BGlobal();
    _posFeetGlobal = _est->getFeetPos();
    // std::cout<<"_posFeetGlobal:\n"<< _posFeetGlobal <<std::endl;

    _velFeetGlobal = _est->getFeetVel();

    _B2G_RotMat = _lowstate->_imu.GetRotMat().cast<double>();
    _G2B_RotMat = _B2G_RotMat.transpose();
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
    _dYawCmdPast = _dYawCmd;

    // std::cout<< "_vCmdBody: \n"<< _vCmdBody << std::endl;   
    // std::cout<< "_dYawCmd: "<< _dYawCmd << std::endl;
}

void Trotting_State::calcCmd(){
    /* Movement */
    _vCmdGlobal = _B2G_RotMat * _vCmdBody;
    // _vCmdGlobal = _vCmdBody;

    // _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2(_velBody(0)-0.2, _velBody(0)+0.2)); // 对全局期望速度限幅
    // _vCmdGlobal(1) = saturation(_vCmdGlobal(1), Vec2(_velBody(1)-0.2, _velBody(1)+0.2));
    _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2(-0.4, 0.4));
    _vCmdGlobal(1) = saturation(_vCmdGlobal(1), Vec2(-0.1, 0.1));

    // std::cout<< "_velBody: \n"<< _velBody << std::endl;
    // std::cout<< "_vCmdGlobal: \n"<< _vCmdGlobal << std::endl;

    // _pcd(0) + _vCmdGlobal(0) * _fsm_state_ctrl_comp->dt:对速度积分、累加质心位置，算位移
    // 再对位移限幅
    // _pcd(0) = saturation(_pcd(0) + _vCmdGlobal(0) * _fstate_ctrl->dt, Vec2(_posBody(0) - 0.05, _posBody(0) + 0.05));
    // _pcd(1) = saturation(_pcd(1) + _vCmdGlobal(1) * _fstate_ctrl->dt, Vec2(_posBody(1) - 0.05, _posBody(1) + 0.05));

    _vCmdGlobal(2) = 0;

    /* Turning */
    _yawCmd = _yawCmd + _dYawCmd * _fstate_ctrl->dt;// 计算累积角度

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
    // std::cout<<"_posError:\n"<< _posError <<std::endl;

    _velError = _vCmdGlobal - _velBody;

    _ddPcd = _Kpp * _posError + _Kdp * _velError;
    _dWbd  = _kpw*rotMatToExp(_Rd*_G2B_RotMat) + _Kdw * (_wCmdGlobal - _lowstate->_imu.getGyroGlobal().cast<double>());

    _ddPcd(0) = saturation(_ddPcd(0), Vec2(-5, 5));
    _ddPcd(1) = saturation(_ddPcd(1), Vec2(-5, 5));
    _ddPcd(2) = saturation(_ddPcd(2), Vec2(-10, 10));

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
    _q = vec34ToVec12(_fstate_ctrl->_ioros->getQ()).cast<double>();
    _tau = getTau(_q.cast<float>(), _forceFeetBody);
}

void Trotting_State::calcQQd(){

    Vec34 _posFeet2B;
    _posFeet2B = GetFeetPos2BODY(*_lowstate,FrameType::BODY);
    
    for(int i(0); i<4; ++i){
        _posFeet2BGoal.col(i) = _G2B_RotMat* (_posFeetGlobalGoal.col(i));
        _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i)); 
        // _posFeet2BGoal.col(i) = (_posFeetGlobalGoal.col(i)- _posBody);
        // _velFeet2BGoal.col(i) =  (_velFeetGlobalGoal.col(i)- _velBody); 
        // std::cout<<"_posFeetGlobalGoal.col(i)- _posBody:\n"<< _posFeetGlobalGoal.col(i)- _posBody <<std::endl;
        // std::cout<<"_G2B_RotMat:\n"<< _G2B_RotMat <<std::endl;

    }
    // std::cout<<"_posFeetGlobalGoal:\n"<< _posFeetGlobalGoal <<std::endl;
    // std::cout<<"_posBody:\n"<< _posBody <<std::endl;

    // std::cout<<"_posFeet2BGoal:\n"<< _posFeet2BGoal <<std::endl;
    // std::cout<<"_posFeetGlobalGoal:\n"<< _posFeetGlobalGoal <<std::endl;
    // std::cout<<"_posBody:\n"<< _posBody <<std::endl;

    // usleep(100000);
    // sleep(1);
    _qGoal = vec12ToVec34(Reversal_GetQ(_posFeet2BGoal, FrameType::BODY));
    _qdGoal = vec12ToVec34(Reversal_GetQd(_posFeet2B, _velFeet2BGoal, FrameType::BODY));
    // _qqq = _fstate_ctrl->_ioros->getQ12();
    // _www = _fstate_ctrl->_ioros->getW12();
    // _tau = CalTaus(_qqq,_www,_KP,_KD,vec34ToVec12(_posFeet2BGoal.cast<float>()),vec34ToVec12(_velFeet2BGoal.cast<float>()),FrameType::BODY).cast<double>();

}
