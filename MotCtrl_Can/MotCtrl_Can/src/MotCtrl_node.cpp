#include "MotCtrl_node.hpp"


void MotorsNode::publish_left_leg() {
    auto Joint_message = sensor_msgs::msg::JointState();
    Joint_message.header.stamp = this->now();
    Joint_message.name = {"joint1", "joint2", "joint3", "joint4", "joint5"};
    Joint_message.position = {left_leg_motors_DM[0]->get_motor_pos(),
                             left_leg_motors_DM[1]->get_motor_pos(),
                             left_leg_motors_DM[2]->get_motor_pos(),
                             left_leg_motors_ENC[0]->get_motor_pos(),
                             left_leg_motors_ENC[1]->get_motor_pos()
                             };
    Joint_message.velocity = {left_leg_motors_DM[0]->get_motor_spd(), 
                             left_leg_motors_DM[1]->get_motor_spd(),
                             left_leg_motors_DM[2]->get_motor_spd(),
                             left_leg_motors_ENC[0]->get_motor_spd(),
                             left_leg_motors_ENC[1]->get_motor_spd()
                             };
    Joint_message.effort = {left_leg_motors_DM[0]->get_motor_current(), 
                            left_leg_motors_DM[1]->get_motor_current(),
                            left_leg_motors_DM[2]->get_motor_current(),
                            left_leg_motors_ENC[0]->get_motor_current(),
                            left_leg_motors_ENC[1]->get_motor_current()
                            };
    left_leg_publisher_->publish(Joint_message);
}

void MotorsNode::publish_right_leg() {
    auto Joint_message = sensor_msgs::msg::JointState();
    Joint_message.header.stamp = this->now();
    Joint_message.name = {"joint1", "joint2", "joint3"};
    Joint_message.position = {right_leg_motors_DM[0]->get_motor_pos(),
                             right_leg_motors_DM[1]->get_motor_pos(),
                             right_leg_motors_DM[2]->get_motor_pos()};
    Joint_message.velocity = {right_leg_motors_DM[0]->get_motor_spd(), 
                             right_leg_motors_DM[1]->get_motor_spd(),
                             right_leg_motors_DM[2]->get_motor_spd()};
    Joint_message.effort = {right_leg_motors_DM[0]->get_motor_current(), 
                            right_leg_motors_DM[1]->get_motor_current(),
                            right_leg_motors_DM[2]->get_motor_current()};
    right_leg_publisher_->publish(Joint_message);
}

void MotorsNode::publish_left_arm() {    
    auto Joint_message = sensor_msgs::msg::JointState();
    Joint_message.header.stamp = this->now();
    Joint_message.name = {"joint1", "joint2", "joint3"};
    Joint_message.position = {left_arm_motors_DM[0]->get_motor_pos(),
                             left_arm_motors_DM[1]->get_motor_pos(),
                             left_arm_motors_DM[2]->get_motor_pos()};
    Joint_message.velocity = {left_arm_motors_DM[0]->get_motor_spd(), 
                             left_arm_motors_DM[1]->get_motor_spd(),
                             left_arm_motors_DM[2]->get_motor_spd()};
    Joint_message.effort = {left_arm_motors_DM[0]->get_motor_current(), 
                            left_arm_motors_DM[1]->get_motor_current(),
                            left_arm_motors_DM[2]->get_motor_current()};
    left_arm_publisher_->publish(Joint_message);
}

void MotorsNode::publish_right_arm() {
    auto Joint_message = sensor_msgs::msg::JointState();
    Joint_message.header.stamp = this->now();
    Joint_message.name = {"joint1", "joint2", "joint3"};
    Joint_message.position = {right_arm_motors_DM[0]->get_motor_pos(),
                             right_arm_motors_DM[1]->get_motor_pos(),
                             right_arm_motors_DM[2]->get_motor_pos()};
    Joint_message.velocity = {right_arm_motors_DM[0]->get_motor_spd(), 
                             right_arm_motors_DM[1]->get_motor_spd(),
                             right_arm_motors_DM[2]->get_motor_spd()};
    Joint_message.effort = {right_arm_motors_DM[0]->get_motor_current(), 
                            right_arm_motors_DM[1]->get_motor_current(),
                            right_arm_motors_DM[2]->get_motor_current()};
    right_arm_publisher_->publish(Joint_message);
}


void MotorsNode::subs_left_leg_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg) {
    publish_left_leg();
    for (int i = can0_startID_; i <= can0_endID_; i++) {
        left_leg_motors_DM[i - can0_startID_]->MotorMitModeCmd(
            msg->position[i - can0_startID_],
            msg->velocity[i - can0_startID_], Kp_MIT, Kd_MIT,
            msg->effort[i - can0_startID_]);
    }
    //ENCOS TEST
    left_leg_motors_ENC[0]->MotorMitCtrlCmd(0,0,Kp_MIT, Kd_MIT,0);
    left_leg_motors_ENC[1]->MotorMitCtrlCmd(0,0,Kp_MIT, Kd_MIT,0);
}

void MotorsNode::subs_right_leg_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg) {
    publish_right_leg();
    for (int i = can1_startID_; i <= can1_endID_; i++) {
        right_leg_motors_DM[i - can1_startID_]->MotorMitModeCmd(
            msg->position[i - can1_startID_],
            msg->velocity[i - can1_startID_], Kp_MIT, Kd_MIT,
            msg->effort[i - can1_startID_]);
    }
}

void MotorsNode::subs_left_arm_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg) {
    publish_left_arm();
    for (int i = can2_startID_; i <= can2_endID_; i++) {
        left_arm_motors_DM[i - can2_startID_]->MotorMitModeCmd(
            msg->position[i - can2_startID_],
            msg->velocity[i - can2_startID_], Kp_MIT, Kd_MIT,
            msg->effort[i - can2_startID_]);
    }
}

void MotorsNode::subs_right_arm_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg) {
    publish_right_arm();
    for (int i = can3_startID_; i <= can3_endID_; i++) {
        right_arm_motors_DM[i - can3_startID_]->MotorMitModeCmd(
            msg->position[i - can3_startID_],
            msg->velocity[i - can3_startID_], Kp_MIT, Kd_MIT,
            msg->effort[i - can3_startID_]);
    }
}


void MotorsNode::init_motors() {
    for (int i = can0_startID_; i <= can0_endID_; i++) {
        left_leg_motors_DM[i - can0_startID_]->MotorInit();
        Timer::ThreadSleepForUs(200);
    }
    for (int i = can1_startID_; i <= can1_endID_; i++) {
        right_leg_motors_DM[i - can1_startID_]->MotorInit();
        Timer::ThreadSleepForUs(200);
    }
    for (int i = can2_startID_; i <= can2_endID_; i++) {
        left_arm_motors_DM[i - can2_startID_]->MotorInit();
        Timer::ThreadSleepForUs(200);
    }
    for (int i = can3_startID_; i <= can3_endID_; i++) {
        right_arm_motors_DM[i - can3_startID_]->MotorInit();
        Timer::ThreadSleepForUs(200);
    }
    Timer::ThreadSleepFor(1000);
    publish_left_leg();
    publish_right_leg();
    publish_left_arm();
    publish_right_arm();
    is_init_.store(true);
}

void MotorsNode::deinit_motors() {
    for (int i = can0_startID_; i <= can0_endID_; i++) {
        left_leg_motors_DM[i - can0_startID_]->MotorDeInit();
        Timer::ThreadSleepForUs(200);
    }
    for (int i = can1_startID_; i <= can1_endID_; i++) {
        right_leg_motors_DM[i - can1_startID_]->MotorDeInit();
        Timer::ThreadSleepForUs(200);
    }
    for (int i = can2_startID_; i <= can2_endID_; i++) {
        left_arm_motors_DM[i - can2_startID_]->MotorDeInit();
        Timer::ThreadSleepForUs(200);
    }
    for (int i = can3_startID_; i <= can3_endID_; i++) {
        right_arm_motors_DM[i - can3_startID_]->MotorDeInit();
        Timer::ThreadSleepForUs(200);
    }
    is_init_.store(false);
}

void MotorsNode::set_zeros() {
    if(!is_init_.load()){
        RCLCPP_WARN(this->get_logger(), "Motors are not initialized, cannot set zeros.");
        return;
    }
    {
        for (int i = can0_startID_; i <= can0_endID_; i++) {
            left_leg_motors_DM[i - can0_startID_]->MotorSetZero();
            Timer::ThreadSleepForUs(200);
        }
        for (int i = can1_startID_; i <= can1_endID_; i++) {
            right_leg_motors_DM[i - can1_startID_]->MotorSetZero();
            Timer::ThreadSleepForUs(200);
        }
        for (int i = can2_startID_; i <= can2_endID_; i++) {
            left_arm_motors_DM[i - can2_startID_]->MotorSetZero();
            Timer::ThreadSleepForUs(200);
        }
        for (int i = can3_startID_; i <= can3_endID_; i++) {
            right_arm_motors_DM[i - can3_startID_]->MotorSetZero();
            Timer::ThreadSleepForUs(200);
        }
    }
}

void MotorsNode::read_motors(){
    for (int i = can0_startID_; i <= can0_endID_; i++) {
        left_leg_motors_DM[i - can0_startID_]->refresh_motor_status();
        Timer::ThreadSleepForUs(200);
    }
    Timer::ThreadSleepForUs(1000);
    publish_left_leg();

    for (int i = can1_startID_; i <= can1_endID_; i++) {
        right_leg_motors_DM[i - can1_startID_]->refresh_motor_status();
        Timer::ThreadSleepForUs(200);
    }
    Timer::ThreadSleepForUs(1000);
    publish_right_leg();

    for (int i = can2_startID_; i <= can2_endID_; i++) {
        left_arm_motors_DM[i - can2_startID_]->refresh_motor_status();
        Timer::ThreadSleepForUs(200);
    }
    Timer::ThreadSleepForUs(1000);
    publish_left_arm();

    for (int i = can3_startID_; i <= can3_endID_; i++) {
        right_arm_motors_DM[i - can3_startID_]->refresh_motor_status();
        Timer::ThreadSleepForUs(200);
    }
    Timer::ThreadSleepForUs(1000);
    publish_right_arm();

}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorsNode>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}