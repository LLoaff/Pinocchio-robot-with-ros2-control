#ifndef IOROS_H
#define IOROS_H
// /pos_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory

#include "LowState.h"
#include "mathTypes.h"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "mathtool.h"

class IORos:public rclcpp::Node
{
public:
    IORos();
    ~IORos();

    void upDate();
    void stateCallBack(const control_msgs::msg::JointTrajectoryControllerState& rec);

    Eigen::Matrix<float,3,4> getQ();
    Eigen::Matrix<float,12,1> getQ12();
    Eigen::Matrix<float,12,1> getW12();

    LowState _state;
private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr _joint_puber_;
    rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr _joint_pub;
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory> _joint_cmd;
    std::shared_ptr<control_msgs::msg::JointTrajectoryControllerState> _joint_state;

    MotorCmd _motor_cmd[12];
};

IORos::IORos():Node("IORos"){
    _joint_puber_=this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/pos_controller/joint_trajectory",1);
    _joint_cmd->points.resize(12);
    _joint_cmd->joint_names={"fr_hip_joint","fr_knee_joint","fr_thigh_joint"
                            "fl_hip_joint","fl_knee_joint","fl_thigh_joint"
                            "br_hip_joint","br_knee_joint","br_thigh_joint"
                            "bl_hip_joint","bl_knee_joint","bl_thigh_joint"};

    for(int i(0);i<12;++i){
        _motor_cmd[i].id = i;
        _motor_cmd[i].motorType = MotorType::GO_M8010_6;
        _motor_cmd[i].mode =queryMotorMode(MotorType::GO_M8010_6,MotorMode::FOC);
        _motor_cmd[i].kp   = 0;
        _motor_cmd[i].kd   = 0;
        _motor_cmd[i].dq   = 0;
        _motor_cmd[i].tau  = 0;
    }

    _motor_cmd[0].q =_state.Angle_Initialization_Variable.Right_Front_Hip_Angle_Init_Value;
    _motor_cmd[1].q =_state.Angle_Initialization_Variable.Right_Front_Thigh_Angle_Init_Value;
    _motor_cmd[2].q =_state.Angle_Initialization_Variable.Right_Front_Calf_Angle_Init_Value;

    _motor_cmd[3].q =_state.Angle_Initialization_Variable.Left_Front_Hip_Angle_Init_Value;
    _motor_cmd[4].q =_state.Angle_Initialization_Variable.Left_Front_Thigh_Angle_Init_Value;
    _motor_cmd[5].q =_state.Angle_Initialization_Variable.Left_Front_Calf_Angle_Init_Value;
    
    _motor_cmd[6].q =_state.Angle_Initialization_Variable.Right_Back_Hip_Angle_Init_Value;
    _motor_cmd[7].q =_state.Angle_Initialization_Variable.Right_Back_Thigh_Angle_Init_Value;
    _motor_cmd[8].q =_state.Angle_Initialization_Variable.Right_Back_Calf_Angle_Init_Value;

    _motor_cmd[9].q =_state.Angle_Initialization_Variable.Left_Back_Hip_Angle_Init_Value;
    _motor_cmd[10].q =_state.Angle_Initialization_Variable.Left_Back_Thigh_Angle_Init_Value;
    _motor_cmd[11].q =_state.Angle_Initialization_Variable.Left_Back_Calf_Angle_Init_Value;

    _joint_pub = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>("/pos_controller/controller_state",1,
        std::bind(IORos::stateCallBack,this,std::placeholders::_1));
}

void IORos::upDate(){
    for(int i(0);i<12;++i){
        _joint_cmd->points[0].positions[i] = _motor_cmd[i].q;
        _joint_cmd->points[0].velocities[i] = _motor_cmd[i].dq;
    }
    _joint_cmd->header.stamp = this->get_clock()->now();
    _joint_puber_->publish(*_joint_cmd);
}

Eigen::Matrix<float,3,4> IORos::getQ(){
    Eigen::Matrix<float,3,4> qLegs;
    for(int i(0); i < 4; ++i){
        qLegs.col(i)(0) = _joint_state->feedback.positions[3*i + 0];
        qLegs.col(i)(1) = _joint_state->feedback.positions[3*i + 1];
        qLegs.col(i)(2) = _joint_state->feedback.positions[3*i + 2];
    }
  return qLegs;
}

Eigen::Matrix<float,12,1> IORos::getQ12(){
    Eigen::Matrix<float,12,1> qLegs;
    for(int i(0); i < 4; ++i){
        qLegs(3*i  ) = _joint_state->feedback.positions[3*i  ];
        qLegs(3*i+1) = _joint_state->feedback.positions[3*i+1];
        qLegs(3*i+2) = _joint_state->feedback.positions[3*i+2];
    }
  return qLegs;
}

Eigen::Matrix<float,12,1> IORos::getW12(){
    Eigen::Matrix<float,12,1> w;
    for(int i(0); i < 4; ++i){
        w(3*i  ) = _joint_state->feedback.velocities[3*i  ];
        w(3*i+1) = _joint_state->feedback.velocities[3*i+1];
        w(3*i+2) = _joint_state->feedback.velocities[3*i+2];
    }
  return w;
}

IORos::~IORos(){
    rclcpp::shutdown();
}
#endif


