#include "FSM/ControlComponent.h"

ControlComponent::ControlComponent(){
    _contact = new Eigen::Matrix<int,4,1>();
    _phase = new Eigen::Matrix<double,4,1>();
    *_contact = Eigen::Matrix<int,4,1>(0,0,0,0);
    *_phase = Eigen::Matrix<double,4,1>(0.5,0.5,0.5,0.5);
    robotModel = new QuadrupedRobot();
    _ioros = std::make_shared<IORos>();

    executor.add_node(_ioros);
    spin_thread = std::thread([this]() {
        executor.spin(); // 子线程在这里阻塞
    });
    std::cout << "IOROS初始化 OK" << std::endl;
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
    _estimator = new Estimator(&_ioros->_state, _contact,_phase,dt);
    balCtrl = new BalanceCtrl();
}

ControlComponent::~ControlComponent(){
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    if (spin_thread.joinable()){
        spin_thread.join();
    }
    delete _estimator;
    delete _contact;
    delete _phase;
    delete balCtrl;
    delete robotModel;
}
