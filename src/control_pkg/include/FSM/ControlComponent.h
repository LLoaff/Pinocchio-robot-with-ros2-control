#ifndef CONTROLCOMPONENT_H
#define CONTROLCOMPONENT_H

#include "LowCmd.h"
#include "LowState.h"
#include <eigen3/Eigen/Dense>
#include "Kenimatics_normal_solution.h"
#include "UserCmd.h"
#include "Estimator.h"
#include "BalanceCtrl.h"
#include "Gait/WaveGenerator.h"
#include "unitreeRobot.h"

class ControlComponent
{
public:
    ControlComponent();
    ~ControlComponent();

    Eigen::Matrix<float,3,1> CalTau(
        int group,Eigen::Matrix<float,3,1> angle,Eigen::Matrix<float,3,1> w,Eigen::Matrix<float,3,3>Kp ,Eigen::Matrix<float,3,3>Kd,Eigen::Matrix<float,3,1> target_pos, Eigen::Matrix<float,3,1> target_speed,FrameType frame);
    Eigen::Matrix<float,12,1> CalTaus(
        Eigen::Matrix<float,12,1> angle,Eigen::Matrix<float,12,1> w,Eigen::Matrix<float,3,3>Kp ,Eigen::Matrix<float,3,3>Kd,Eigen::Matrix<float,12,1> target_pos, Eigen::Matrix<float,12,1> target_speed,FrameType frame);
    Eigen::Matrix<double,3,3> calcJaco(int legid , Eigen::Matrix<float,3,1> q);
    Eigen::Matrix<double,3,1> calcTau(int legid,Eigen::Matrix<float,3,1> q, Eigen::Matrix<double,3,1> force);
    Eigen::Matrix<double,12,1> getTau(const Eigen::Matrix<float,12,1> &q, const Eigen::Matrix<double,3,4> feetForce);

    void runWaveGen();
    void setAllStance();
    void setAllSwing();
    void setStartWave();
    void Estimator_Init();

    LowCmd   *  _ctrl_cmd;
    UserCmd  *  user_cmd = UserCmd::GetInstance(); // 获取单一实例
    Estimator * _estimator;
    BalanceCtrl* balCtrl;
    QuadrupedRobot *robotModel;

    double dt;
    Eigen::Matrix<int,4,1> * _contact;
    Eigen::Matrix<double, 4, 1>* _phase;
    WaveGenerator *waveGen;
private:
    WaveStatus _waveStatus = WaveStatus::SWING_ALL;
};

ControlComponent::ControlComponent(){
    _ctrl_cmd = new LowCmd();
    _contact = new Eigen::Matrix<int,4,1>();
    _phase = new Eigen::Matrix<double,4,1>();
    *_contact = Eigen::Matrix<int,4,1>(0,0,0,0);
    *_phase = Eigen::Matrix<double,4,1>(0.5,0.5,0.5,0.5);
    robotModel = new QuadrupedRobot();
}

Eigen::Matrix<float,3,1> ControlComponent::CalTau(
    int group,Eigen::Matrix<float,3,1> angle,Eigen::Matrix<float,3,1> w,Eigen::Matrix<float,3,3>Kp ,Eigen::Matrix<float,3,3>Kd,Eigen::Matrix<float,3,1> target_pos, Eigen::Matrix<float,3,1> target_speed,FrameType frame)
{
    Eigen::Matrix<float,3,1> f; 
    Eigen::Matrix<float,3,1> tua;
    Eigen::Matrix<float,3,1> pos_xyz; // 存储xyz
    Eigen::Matrix<float,3,1> pos_s;   // 存储xyz的速度
    Eigen::Matrix<float,3,3> Jac;
    float l1 = _labad_;
    float l2 = -_lhip_;
    float l3 = -_lknee_;
    
    if(group == 0 || group == 2)
        l1 = -_labad_;
    // 计算雅可比 position
    float s1 = std::sin(angle(0));
    float s2 = std::sin(angle(1));
    float s3 = std::sin(angle(2));

    float c1 = std::cos(angle(0));
    float c2 = std::cos(angle(1));
    float c3 = std::cos(angle(2));

    float c23 = c2 * c3 - s2 * s3;
    float s23 = s2 * c3 + c2 * s3;
    Jac(0, 0) = 0;
    Jac(1, 0) = -l3 * c1 * c23 - l2 * c1 * c2 - l1 * s1;
    Jac(2, 0) = -l3 * s1 * c23 - l2 * c2 * s1 + l1 * c1;
    Jac(0, 1) = l3 * c23 + l2 * c2;
    Jac(1, 1) = l3 * s1 * s23 + l2 * s1 * s2;
    Jac(2, 1) = -l3 * c1 * s23 - l2 * c1 * s2;
    Jac(0, 2) = l3 * c23;
    Jac(1, 2) = l3 * s1 * s23;
    Jac(2, 2) = -l3 * c1 * s23;
    pos_s = Jac * w;    // 三维速度
    if(frame == FrameType::HIP){
        pos_xyz = GetPos_H(group,angle(0),angle(1),angle(2));
    }
    else if(frame == FrameType::BODY){
        pos_xyz = GetPos_B(group,angle(0),angle(1),angle(2));
    }
    f= Kp*(target_pos-pos_xyz)+Kd*(target_speed-pos_s);

    tua = Jac.transpose()*f;

    return tua;
}

Eigen::Matrix<float,12,1> ControlComponent::CalTaus(
    Eigen::Matrix<float,12,1> angle,Eigen::Matrix<float,12,1> w,Eigen::Matrix<float,3,3>Kp ,Eigen::Matrix<float,3,3>Kd,Eigen::Matrix<float,12,1> target_pos, Eigen::Matrix<float,12,1> target_speed,FrameType frame)
{
    Eigen::Matrix<float,12,1> tua;
    for(int i(0);i<4;++i){
        tua.segment(3*i,3) = CalTau(i,angle.segment(3*i,3),w.segment(3*i,3),Kp,Kd,target_pos.segment(3*i,3),target_speed.segment(3*i,3),frame);
    }
    return tua;
}

// 计算一条腿的Jaco
Eigen::Matrix<double,3,3> ControlComponent::calcJaco(int legid , Eigen::Matrix<float,3,1> q){
    Eigen::Matrix<double,3,3> Jac;
    float l1 = _labad_;
    float l2 = -_lhip_;
    float l3 = -_lknee_;
    
    if(legid == 0 || legid == 2)
        l1 = -_labad_;
    // 计算雅可比 

    float s1 = std::sin(q(0));
    float s2 = std::sin(q(1));
    float s3 = std::sin(q(2));

    float c1 = std::cos(q(0));
    float c2 = std::cos(q(1));
    float c3 = std::cos(q(2));

    float c23 = c2 * c3 - s2 * s3;
    float s23 = s2 * c3 + c2 * s3;
    Jac(0, 0) = 0;
    Jac(1, 0) = -l3 * c1 * c23 - l2 * c1 * c2 - l1 * s1;
    Jac(2, 0) = -l3 * s1 * c23 - l2 * c2 * s1 + l1 * c1;
    Jac(0, 1) = l3 * c23 + l2 * c2;
    Jac(1, 1) = l3 * s1 * s23 + l2 * s1 * s2;
    Jac(2, 1) = -l3 * c1 * s23 - l2 * c1 * s2;
    Jac(0, 2) = l3 * c23;
    Jac(1, 2) = l3 * s1 * s23;
    Jac(2, 2) = -l3 * c1 * s23;
    // std::cout<< "Jac:\n"<<Jac<<"---\n"<<std::endl;
    return Jac;
}

//计算一条腿的Tau
Eigen::Matrix<double,3,1> ControlComponent::calcTau(int legid ,Eigen::Matrix<float,3,1> q, Eigen::Matrix<double,3,1> force){
    return calcJaco(legid,q).transpose() * force;
}

//获取所有腿的Tau
Eigen::Matrix<double,12,1> ControlComponent::getTau(const Eigen::Matrix<float,12,1> &q, const Eigen::Matrix<double,3,4> feetForce){
    Eigen::Matrix<double,12,1> tau;
    for(int i(0); i < 4; ++i){
        tau.segment(3*i, 3) = calcTau(i,q.segment(3*i, 3), feetForce.col(i));
    }
    return tau;
}

void ControlComponent::runWaveGen(){
    waveGen->calcContactPhase(*_phase, *_contact, _waveStatus);
}

void ControlComponent::setAllStance(){
    // (*_contact)(0) = 1;
    // (*_contact)(1) = 1;
    // (*_contact)(2) = 1;
    // (*_contact)(3) = 1;
    _waveStatus = WaveStatus::STANCE_ALL;
}

void ControlComponent::setAllSwing(){
    // (*_contact)(0) = 0;
    // (*_contact)(1) = 0;
    // (*_contact)(2) = 0;
    // (*_contact)(3) = 0;
    _waveStatus = WaveStatus::SWING_ALL;
}

void ControlComponent::setStartWave(){
    _waveStatus = WaveStatus::WAVE_ALL;
}

void ControlComponent::Estimator_Init(){
    _estimator = new Estimator(&_ctrl_cmd->_state, _contact,_phase,dt);
    balCtrl = new BalanceCtrl();
}

ControlComponent::~ControlComponent(){
    delete _ctrl_cmd;
    delete _estimator;
    delete _contact;
    delete _phase;
    delete balCtrl;
    delete robotModel;
}
#endif