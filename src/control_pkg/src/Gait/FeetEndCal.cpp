#include "Gait/FeetEndCal.h"

FeetEndCal::FeetEndCal(ControlComponent *ctrlComp)
: _est(ctrlComp->_estimator), _lowState(&ctrlComp->_ioros->_state),_robModel(ctrlComp->robotModel){

    _Tstance  = ctrlComp->waveGen->getTstance();
    _Tswing   = ctrlComp->waveGen->getTswing();

    _kx = 0.1;
    _ky = 0.1;
    _kyaw = 0.1;

    Vec34 feetPosBody = _robModel->getFeetPosIdeal();

    for(int i(0); i<4; ++i){
        _feetRadius(i)    = sqrt( pow(feetPosBody(0, i), 2) + pow(feetPosBody(1, i), 2));
        _feetInitAngle(i) = atan2(feetPosBody(1, i), feetPosBody(0, i));
    }
}

Vec3 FeetEndCal::calFootPos(int legID, Vec2 vxyGoalGlobal, float dYawGoal, float phase){
    _bodyVelGlobal = _est->getVelocity();
    
    _bodyWGlobal = _lowState->_imu.getGyroGlobal().cast<double>();
    // std::cout<<"VelGlobal:\n"<< _bodyVelGlobal<<std::endl;
    // std::cout<<"WGlobal:\n"<< _bodyWGlobal<<std::endl;
    
    _nextStep(0) = _bodyVelGlobal(0)*(1-phase)*_Tswing + _bodyVelGlobal(0)*_Tstance/2 + _kx*(_bodyVelGlobal(0) - vxyGoalGlobal(0));
    _nextStep(1) = _bodyVelGlobal(1)*(1-phase)*_Tswing + _bodyVelGlobal(1)*_Tstance/2 + _ky*(_bodyVelGlobal(1) - vxyGoalGlobal(1));
    _nextStep(2) = 0;

    _yaw = _lowState->_imu.getYaw();
    _dYaw = _lowState->_imu.getDYaw();
    _nextYaw = _dYaw*(1-phase)*_Tswing + _dYaw*_Tstance/2 + _kyaw*(dYawGoal - _dYaw);

    _nextStep(0) += _feetRadius(legID) * cos(_yaw + _feetInitAngle(legID) + _nextYaw);  // 对应9.10 综合平移、转动的x轨迹方程
    _nextStep(1) += _feetRadius(legID) * sin(_yaw + _feetInitAngle(legID) + _nextYaw);  // 对应9.10 综合平移、转动的y轨迹方程

    // _footPos = _est->getPosition() + _nextStep;
    _footPos = _nextStep;
    _footPos(2) = 0.0;
    // std::cout<<"_footPos:\n"<< _footPos<<std::endl;

    return _footPos;
}
