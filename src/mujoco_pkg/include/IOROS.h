#ifndef IOROS_H
#define IOROS_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class IOROS : public rclcpp::Node
{
public:
    IOROS();
    void pubImu(const sensor_msgs::msg::Imu& imu_state);
private:
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;

};



#endif


