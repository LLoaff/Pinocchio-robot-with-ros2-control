#ifndef IOROS_H
#define IOROS_H
// /pos_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory

#include "LowState.h"
#include "mathTypes.h"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "mathtool.h"
#include <csignal>
#include <unistd.h>

void RosShutDown(int sig){
	rclcpp::shutdown();
}
class IORos:public rclcpp::Node
{
public:
    IORos();
    ~IORos();

    void upDate();
    void init();
    void stateCallBack(const control_msgs::msg::JointTrajectoryControllerState& rec);

    Eigen::Matrix<float,3,4> getQ();
    Eigen::Matrix<float,12,1> getQ12();
    Eigen::Matrix<float,12,1> getW12();

    void SetQ(Eigen::Matrix<float,12,1> q);
    void SetQ(int id,float q);
    void SetQ(int leg_id , Eigen::Matrix<float,3,1> q);
    void SetDq(int leg_id,Eigen::Matrix<float,3,1> dq);
    void SetDq(Eigen::Matrix<float,12,1> dq);
    void SetP(int leg_id,Eigen::Matrix<float,3,1> p);
    void SetP(Eigen::Matrix<float,12,1> p);
    void SetZeroP();
    void SetD(int leg_id,Eigen::Matrix<float,3,1> d);
    void SetD(Eigen::Matrix<float,12,1> d);
    void SetZeroD();
    void SetTau(Eigen::Matrix<float,12,1> tau, Eigen::Matrix<double,2,1> torqueLimit);
    void SetTau(int leg_id,Eigen::Matrix<float,3,1> tau);
    void SetZeroTau(int legID);
    void SetZeroTau();
    void SetZeroDq(int legID);
    void SetZeroDq();
    void SetFree();

    LowState _state;

    Eigen::Matrix<float,12,1>                              _init_q;

private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr _joint_puber_;
    rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr _joint_sub;
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory> _joint_cmd;

    // std::shared_ptr<control_msgs::msg::JointTrajectoryControllerState> _joint_state;

    // MotorCmd _motor_cmd[12];
};

IORos::IORos():Node("IORos"){
    _joint_puber_=this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/pos_controller/joint_trajectory",1);

    _joint_cmd = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    _joint_cmd->points.resize(1);
    _joint_cmd->points[0].positions.resize(12);
    _joint_cmd->points[0].velocities.resize(12);

    _joint_cmd->joint_names={"fr_hip_joint","fr_knee_joint","fr_thigh_joint",
                             "fl_hip_joint","fl_knee_joint","fl_thigh_joint",
                             "br_hip_joint","br_knee_joint","br_thigh_joint",
                             "bl_hip_joint","bl_knee_joint","bl_thigh_joint"};

    // for(int i(0);i<12;++i){
    //     _motor_cmd[i].id = i;
    //     _motor_cmd[i].motorType = MotorType::GO_M8010_6;
    //     _motor_cmd[i].mode =queryMotorMode(MotorType::GO_M8010_6,MotorMode::FOC);
    //     _motor_cmd[i].kp   = 0;
    //     _motor_cmd[i].kd   = 0;
    //     _motor_cmd[i].dq   = 0;
    //     _motor_cmd[i].tau  = 0;
    // }

    _joint_cmd->points[0].positions[0] =_state.Angle_Initialization_Variable.Right_Front_Hip_Angle_Init_Value;
    _joint_cmd->points[0].positions[1] =_state.Angle_Initialization_Variable.Right_Front_Thigh_Angle_Init_Value;
    _joint_cmd->points[0].positions[2] =_state.Angle_Initialization_Variable.Right_Front_Calf_Angle_Init_Value;

    _joint_cmd->points[0].positions[3] =_state.Angle_Initialization_Variable.Left_Front_Hip_Angle_Init_Value;
    _joint_cmd->points[0].positions[4] =_state.Angle_Initialization_Variable.Left_Front_Thigh_Angle_Init_Value;
    _joint_cmd->points[0].positions[5] =_state.Angle_Initialization_Variable.Left_Front_Calf_Angle_Init_Value;
    
    _joint_cmd->points[0].positions[6] =_state.Angle_Initialization_Variable.Right_Back_Hip_Angle_Init_Value;
    _joint_cmd->points[0].positions[7] =_state.Angle_Initialization_Variable.Right_Back_Thigh_Angle_Init_Value;
    _joint_cmd->points[0].positions[8] =_state.Angle_Initialization_Variable.Right_Back_Calf_Angle_Init_Value;

    _joint_cmd->points[0].positions[9] =_state.Angle_Initialization_Variable.Left_Back_Hip_Angle_Init_Value;
    _joint_cmd->points[0].positions[10] =_state.Angle_Initialization_Variable.Left_Back_Thigh_Angle_Init_Value;
    _joint_cmd->points[0].positions[11] =_state.Angle_Initialization_Variable.Left_Back_Calf_Angle_Init_Value;
    for(int i(0);i<12;++i){
        _init_q(i) = _joint_cmd->points[0].positions[i];
    }
    
    
    signal(SIGINT, RosShutDown);

    init();
}
void IORos::init(){
    _joint_sub = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>("/pos_controller/controller_state",1,
        std::bind(&IORos::stateCallBack,this,std::placeholders::_1));
}

void IORos::upDate(){
    // for(int i(0);i<12;++i){
    //     _joint_cmd->points[0].positions[i] = _motor_cmd[i].q;
    //     _joint_cmd->points[0].velocities[i] = _motor_cmd[i].dq;
    // }
    _joint_cmd->header.stamp = rclcpp::Time(0);
    _joint_cmd->points[0].time_from_start = rclcpp::Duration(0, 0);
    _joint_puber_->publish(*_joint_cmd);
}

void IORos::stateCallBack(const control_msgs::msg::JointTrajectoryControllerState& rec){
    for(int i(0);i<12;++i){
        _state._motor_data[i].q = rec.feedback.positions[i];
        _state._motor_data[i].dq = rec.feedback.velocities[i];
    }
}

Eigen::Matrix<float,3,4> IORos::getQ(){
    Eigen::Matrix<float,3,4> qLegs;
    for(int i(0); i < 4; ++i){
        qLegs.col(i)(0) = _state._motor_data[3*i + 0].q;
        qLegs.col(i)(1) = _state._motor_data[3*i + 1].q;
        qLegs.col(i)(2) = _state._motor_data[3*i + 2].q;
    }
  return qLegs;
}

Eigen::Matrix<float,12,1> IORos::getQ12(){
    Eigen::Matrix<float,12,1> qLegs;
    for(int i(0); i < 4; ++i){
        qLegs(3*i  ) = _state._motor_data[3*i + 0].q;
        qLegs(3*i+1) = _state._motor_data[3*i + 1].q;
        qLegs(3*i+2) = _state._motor_data[3*i + 2].q;
    }
  return qLegs;
}

Eigen::Matrix<float,12,1> IORos::getW12(){
    Eigen::Matrix<float,12,1> w;
    for(int i(0); i < 4; ++i){
        w(3*i  ) = _state._motor_data[3*i + 0].dq;
        w(3*i+1) = _state._motor_data[3*i + 1].dq;
        w(3*i+2) = _state._motor_data[3*i + 2].dq;
    }
  return w;
}

void IORos::SetQ(Eigen::Matrix<float,12,1> q)
{
    for(int i(0); i<12; ++i){
        _joint_cmd->points[0].positions[i] = q(i);
    }
}

void IORos::SetQ(int id,float q)       // 单独控制某一电机角度
{
    _joint_cmd->points[0].positions[id] = q;
}

void IORos::SetQ(int leg_id , Eigen::Matrix<float,3,1> q)
{  
        _joint_cmd->points[0].positions[3*leg_id +0 ] = q(0);
        _joint_cmd->points[0].positions[3*leg_id +1 ] = q(1);
        _joint_cmd->points[0].positions[3*leg_id +2 ] = q(2); 
}

void IORos::SetDq(int leg_id,Eigen::Matrix<float,3,1> dq)
{
    _joint_cmd->points[0].velocities[3*leg_id +0 ] = dq(0);
    _joint_cmd->points[0].velocities[3*leg_id +1 ] = dq(1);
    _joint_cmd->points[0].velocities[3*leg_id +2 ] = dq(2); 
}

void IORos::SetDq(Eigen::Matrix<float,12,1> dq)  
{  
    for(int i=0;i<12;i++)
    {
        _joint_cmd->points[0].velocities[i] = dq(i);
    }
}

void IORos::SetP(int leg_id,Eigen::Matrix<float,3,1> p)
{
    // _cmd[3*leg_id +0 ].kp = p(0);
    // _cmd[3*leg_id +1 ].kp = p(1);
    // _cmd[3*leg_id +2 ].kp = p(2); 
}

void IORos::SetP(Eigen::Matrix<float,12,1> p)  
{  
    for(int i=0;i<12;i++)
    {
        // _cmd[i].kp = p(i);
    }
}

void IORos::SetZeroP(){
    for(int i(0); i<4; ++i){
        // _cmd[3*i+0].kp = 0;
        // _cmd[3*i+1].kp = 0;
        // _cmd[3*i+2].kp = 0;
    }
}

void IORos::SetD(int leg_id,Eigen::Matrix<float,3,1> d)
{
    // _cmd[3*leg_id +0 ].kd = d(0);
    // _cmd[3*leg_id +1 ].kd = d(1);
    // _cmd[3*leg_id +2 ].kd = d(2); 
}

void IORos::SetD(Eigen::Matrix<float,12,1> d)  
{  
    for(int i=0;i<12;i++)
    {
        // _cmd[i].kd = d(i);
    }
}

void IORos::SetZeroD(){
    for(int i(0); i<4; ++i){
        // _cmd[3*i+0].kd = 0;
        // _cmd[3*i+1].kd = 0;
        // _cmd[3*i+2].kd = 0;
    }
}

void IORos::SetTau(Eigen::Matrix<float,12,1> tau, Eigen::Matrix<double,2,1> torqueLimit = Eigen::Matrix<double,2,1>(-1.3, 1.3)){
    for(int i(0); i<12; ++i){
        if(std::isnan(tau(i))){
            printf("[ERROR] The setTau function meets Nan\n");
        }
        // _cmd[i].tau = saturation(tau(i), torqueLimit);
    }
}

void IORos::SetTau(int leg_id,Eigen::Matrix<float,3,1> tau){
    // _cmd[leg_id*3+0].tau = tau(0);
    // _cmd[leg_id*3+1].tau = tau(1);
    // _cmd[leg_id*3+2].tau = tau(2);
    
}

void IORos::SetZeroTau(int legID){
    // _cmd[legID*3+0].tau = 0;
    // _cmd[legID*3+1].tau = 0;
    // _cmd[legID*3+2].tau = 0;
}

void IORos::SetZeroTau(){
    for(uint8_t i=0;i<4;i++)
    {
        // _cmd[i*3+0].tau = 0;
        // _cmd[i*3+1].tau = 0;
        // _cmd[i*3+2].tau = 0;
    }
    
}

void IORos::SetZeroDq(int legID){
    _joint_cmd->points[0].velocities[legID*3+0] = 0;
    _joint_cmd->points[0].velocities[legID*3+1] = 0;
    _joint_cmd->points[0].velocities[legID*3+2] = 0;
}

void IORos::SetZeroDq(){
    for(int i(0); i<4; ++i){
        SetZeroDq(i);
    }
}

void IORos::SetFree()
{
    SetZeroDq();
    SetZeroTau();
}

IORos::~IORos(){
    rclcpp::shutdown();
}
#endif


