#include "Gait/GaitGenerator.h"

GaitGenerator::GaitGenerator(ControlComponent *ctrlComp)
              : _waveG(ctrlComp->waveGen), _est(ctrlComp->_estimator), 
                _phase(ctrlComp->_phase), _contact(ctrlComp->_contact), 
                 _state(&ctrlComp->_ioros->_state),_robModel(ctrlComp->robotModel){
    _feetCal = new FeetEndCal(ctrlComp);
    _firstRun = true;
}

void GaitGenerator::setGait(Vec2 vxyGoalGlobal, float dYawGoal, float gaitHeight){
    _vxyGoal = vxyGoalGlobal;
    _dYawGoal = dYawGoal;
    _gaitHeight = gaitHeight;
}

void GaitGenerator::restart(){
    _firstRun = true;
    _vxyGoal.setZero();
}

void GaitGenerator::run(Vec34 &feetPos, Vec34 &feetVel){
    if(_firstRun){
        _startP = _robModel->getFeetPosIdeal();
        _firstRun = false;
    }

    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 1){
            // if((*_phase)(i) < 0.5){
            //     _startP.col(i) = _est->getFootPos(i);   // 支撑相前半周期允许随着状态微调
            // }
            feetPos.col(i) = _startP.col(i);
            feetVel.col(i).setZero();
        }
        else{
            _endP.col(i) = _feetCal->calFootPos(i, _vxyGoal, _dYawGoal, (*_phase)(i)); // 获得最终x、y的落脚点

            feetPos.col(i) = getFootPos(i); // 获取下一刻 足端的位置
            feetVel.col(i) = getFootVel(i); // 获取下一刻 足端的速度
        }
    }
    // std::cout<<"_endP:\n"<< _endP<<std::endl;
    // std::cout<<"feetPos:\n"<< feetPos<<std::endl;
    // usleep(500000);
    _pastP = feetPos;
    _phasePast = *_phase;
}

Vec3 GaitGenerator::getFootPos(int i){
    Vec3 footPos;
    footPos(0) = cycloidXYPosition(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i));
    footPos(1) = cycloidXYPosition(_startP.col(i)(1), _endP.col(i)(1), (*_phase)(i));
    footPos(2) =  cycloidZPosition(_startP.col(i)(2), _gaitHeight, (*_phase)(i));
    return footPos;
}

Vec3 GaitGenerator::getFootVel(int i){
    Vec3 footVel;
    footVel(0) = cycloidXYVelocity(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i));
    footVel(1) = cycloidXYVelocity(_startP.col(i)(1), _endP.col(i)(1), (*_phase)(i));
    footVel(2) =  cycloidZVelocity(_gaitHeight, (*_phase)(i));
    return footVel;
}

float GaitGenerator::cycloidXYPosition(float start, float end, float phase){ // 对应9.11 xy方向位移坐标公式
    float phasePI = 2 * M_PI * phase; // phase == t/T
    return (end - start)*(phasePI - sin(phasePI))/(2*M_PI) + start;
}

float GaitGenerator::cycloidXYVelocity(float start, float end, float phase){//  对应9.13 xy方向速度计算坐标公式
    float phasePI = 2 * M_PI * phase;
    return (end - start)*(1 - cos(phasePI)) / _waveG->getTswing();
}

float GaitGenerator::cycloidZPosition(float start, float h, float phase){ // 对应9.11 z方向位移坐标公式
    float phasePI = 2 * M_PI * phase;
    return h*(1 - cos(phasePI))/2 + start;
}

float GaitGenerator::cycloidZVelocity(float h, float phase){//  对应9.13 z方向速度计算坐标公式
    float phasePI = 2 * M_PI * phase;
    return h*M_PI * sin(phasePI) / _waveG->getTswing();
}
