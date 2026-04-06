#include "IOROS.h"

IOROS::IOROS():Node("mujoco_node"){
    imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("imu_state",1);
}

void IOROS::pubImu(const sensor_msgs::msg::Imu& imu_state){
    imu_pub->publish(imu_state);
}

