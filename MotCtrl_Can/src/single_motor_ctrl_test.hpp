#pragma once
#include "rclcpp/rclcpp.hpp"
#include "dm_motor_driver.hpp"
#include "encos_motor_driver.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

enum class MotorType { DM, ENCOS };

struct SingleMotorConfig {
    std::string     can_interface;
    uint16_t        motor_id;
    uint16_t        master_id;
    MotorType       motor_type;
    DM_Motor_Model  dm_model;    // 仅 motor_type == DM 时有效

    // ── 方向与零位 ──────────────────────────────
    // direction   : +1.0 表示电机正转与模型正方向一致，-1.0 表示相反
    // zero_offset : 单位 rad，模型零位时电机的实际读数
    //   变换公式：
    //     发指令：motor_pos = direction * joint_pos + zero_offset
    //     读状态：joint_pos = direction * (motor_pos - zero_offset)
    float direction;
    float zero_offset;

    // ── MIT 模式控制参数 ────────────────────────
    float kp;
    float kd;
};

class SingleMotorCtrlTest : public rclcpp::Node {
public:
    explicit SingleMotorCtrlTest(const SingleMotorConfig& config);
    ~SingleMotorCtrlTest();

private:
    // 坐标变换（关节空间 <-> 电机空间）
    float joint_to_motor_pos(float joint_pos) const;
    float joint_to_motor_vel(float joint_vel) const;
    float motor_to_joint_pos(float motor_pos) const;
    float motor_to_joint_vel(float motor_vel) const;

    // ROS2 回调
    void cmd_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg);
    void timer_publish_state();

    SingleMotorConfig config_;

    std::shared_ptr<DmMotorDriver>    dm_motor_;
    std::shared_ptr<EncosMotorDriver> encos_motor_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr    state_pub_;
    rclcpp::TimerBase::SharedPtr                                  state_timer_;
};
