#ifndef CONTROLCOMPONENT_H
#define CONTROLCOMPONENT_H

#include "UserCmd.h"
#include "Estimator.h"
#include "BalanceCtrl.h"
#include "Gait/WaveGenerator.h"
#include "Robot.h"
#include "IOPort/IORos.h"
#include <thread>

class ControlComponent
{
public:
    ControlComponent();
    ~ControlComponent();

    void runWaveGen();
    void setAllStance();
    void setAllSwing();
    void setStartWave();
    void Estimator_Init();
    
    UserCmd  *  user_cmd = UserCmd::GetInstance(); // 获取单一实例
    Estimator * _estimator;
    BalanceCtrl* balCtrl;
    QuadrupedRobot *robotModel;

    double dt;
    Eigen::Matrix<int,4,1> * _contact;
    Eigen::Matrix<double, 4, 1>* _phase;
    WaveGenerator *waveGen;

    std::shared_ptr<IORos>  _ioros;
    
private:
    WaveStatus _waveStatus = WaveStatus::SWING_ALL;

    std::thread             spin_thread;
    rclcpp::executors::MultiThreadedExecutor executor;
};

#endif