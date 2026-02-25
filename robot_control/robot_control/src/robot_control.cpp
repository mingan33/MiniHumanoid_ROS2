#include "robot_control.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Robot_Control>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


void Robot_Control::model_timer_callback(){
    
    joint_cmd_publish();
}

void Robot_Control::joint_cmd_publish(){
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->now();
    for (int i = 0; i < 3; i++) {
        msg.name.push_back("joint" + std::to_string(i+1));
        msg.position.push_back(0);
        msg.velocity.push_back(0);
        msg.effort.push_back(0);
    }
    left_leg_publisher_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "CAN0 Joint Cmd Publish");

}


void Robot_Control::subs_left_leg_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg) {
    RCLCPP_INFO(this->get_logger(), "CAN0 Joint0 pos: %f, vel: %f, tor: %f",  msg->position[0],msg->velocity[0],msg->effort[0]);
    RCLCPP_INFO(this->get_logger(), "CAN0 Joint1 pos: %f, vel: %f, tor: %f",  msg->position[1],msg->velocity[1],msg->effort[1]);
    RCLCPP_INFO(this->get_logger(), "CAN0 Joint2 pos: %f, vel: %f, tor: %f",  msg->position[2],msg->velocity[2],msg->effort[2]);
    RCLCPP_INFO(this->get_logger(), "CAN0 Joint3 pos: %f, vel: %f, tor: %f",  msg->position[3],msg->velocity[3],msg->effort[3]);
    RCLCPP_INFO(this->get_logger(), "CAN0 Joint4 pos: %f, vel: %f, tor: %f",  msg->position[4],msg->velocity[4],msg->effort[4]);
}

void Robot_Control::subs_right_leg_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg) {
    RCLCPP_INFO(this->get_logger(), "CAN1 Joint0 pos: %f, vel: %f, tor: %f",  msg->position[0],msg->velocity[0],msg->effort[0]);
    RCLCPP_INFO(this->get_logger(), "CAN1 Joint1 pos: %f, vel: %f, tor: %f",  msg->position[1],msg->velocity[1],msg->effort[1]);
    RCLCPP_INFO(this->get_logger(), "CAN1 Joint2 pos: %f, vel: %f, tor: %f",  msg->position[2],msg->velocity[2],msg->effort[2]);
}

void Robot_Control::subs_left_arm_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg) {
    RCLCPP_INFO(this->get_logger(), "CAN2 Joint0 pos: %f, vel: %f, tor: %f",  msg->position[0],msg->velocity[0],msg->effort[0]);
    RCLCPP_INFO(this->get_logger(), "CAN2 Joint1 pos: %f, vel: %f, tor: %f",  msg->position[1],msg->velocity[1],msg->effort[1]);
    RCLCPP_INFO(this->get_logger(), "CAN2 Joint2 pos: %f, vel: %f, tor: %f",  msg->position[2],msg->velocity[2],msg->effort[2]);
}

void Robot_Control::subs_right_arm_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg) {
    RCLCPP_INFO(this->get_logger(), "CAN3 Joint0 pos: %f, vel: %f, tor: %f",  msg->position[0],msg->velocity[0],msg->effort[0]);
    RCLCPP_INFO(this->get_logger(), "CAN3 Joint1 pos: %f, vel: %f, tor: %f",  msg->position[1],msg->velocity[1],msg->effort[1]);
    RCLCPP_INFO(this->get_logger(), "CAN3 Joint2 pos: %f, vel: %f, tor: %f",  msg->position[2],msg->velocity[2],msg->effort[2]);
}


void Robot_Control::subs_cmd_callback(const std::shared_ptr<geometry_msgs::msg::Twist> msg){
    RCLCPP_INFO(this->get_logger(), "Linear X: %f",  msg->linear.x);
}


void Robot_Control::subs_imu_callback(const std::shared_ptr<sensor_msgs::msg::Imu> msg)
{
    RCLCPP_INFO(this->get_logger(), "Angular velocity X: %f",  msg->angular_velocity.x);
}
