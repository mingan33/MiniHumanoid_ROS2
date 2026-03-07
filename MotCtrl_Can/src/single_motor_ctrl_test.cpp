#include "single_motor_ctrl_test.hpp"


static SingleMotorConfig make_test_config() {
    SingleMotorConfig cfg;

    cfg.can_interface = "can0";
    cfg.motor_id      = 0x03;
    cfg.master_id     = 0x13;       // 通常是 motor_id + 0x10
    cfg.motor_type    = MotorType::DM;
    cfg.dm_model      = DM4310_24V; // DM 电机时有效

    // 方向：+1.0 正转与模型一致，-1.0 反转
    cfg.direction   = -1.0f;

    // 零位偏置：模型零位时电机的实际读数（单位 rad）
    // 调试方法：将电机手动移到模型零位，读出 /single_motor_state 中的 position，
    //           将该值填入此处，重新启动节点即可
    cfg.zero_offset = 5.0f;

    // MIT 模式控制参数
    cfg.kp = 1.0f;
    cfg.kd = 0.1f;

    return cfg;
}


SingleMotorCtrlTest::SingleMotorCtrlTest(const SingleMotorConfig& config)
    : Node("single_motor_ctrl_test"), config_(config)
{
    if (config_.motor_type == MotorType::DM) {
        dm_motor_ = std::make_shared<DmMotorDriver>(
            config_.motor_id, config_.can_interface,
            config_.master_id, config_.dm_model);
        uint8_t err = dm_motor_->MotorInit();
        RCLCPP_INFO(get_logger(), "DM motor OK  can=%s  id=%d",
                    config_.can_interface.c_str(), config_.motor_id);
    } else {
        encos_motor_ = std::make_shared<EncosMotorDriver>(
            config_.motor_id, config_.can_interface, config_.master_id);
        RCLCPP_INFO(get_logger(), "ENCOS motor OK  can=%s  id=%d",
                    config_.can_interface.c_str(), config_.motor_id);
    }

    RCLCPP_INFO(get_logger(),
                "direction=%.1f  zero_offset=%.4f rad  kp=%.1f  kd=%.2f",
                config_.direction, config_.zero_offset,
                config_.kp, config_.kd);

    auto ctrl_qos   = rclcpp::QoS(1).reliable().durability_volatile();
    auto sensor_qos = rclcpp::QoS(1).best_effort().durability_volatile();

    // 发布状态话题
    // 消息格式（JointState，仅第 0 个元素）：
    //   position[0] : 当前关节角（rad，模型坐标系）
    //   velocity[0] : 当前关节速度（rad/s，模型坐标系）
    //   effort[0]   : 当前电流/力矩（模型坐标系符号）
    state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        "/single_motor_state", sensor_qos);

    // 10 Hz 定时发布状态
    state_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&SingleMotorCtrlTest::timer_publish_state, this));
}

SingleMotorCtrlTest::~SingleMotorCtrlTest() {
    if (config_.motor_type == MotorType::DM && dm_motor_) {
        dm_motor_->MotorDeInit();
        RCLCPP_INFO(get_logger(), "DM motor disabled.");
    }
}


// ── 坐标变换 ─────────────────────────────────────────

// 关节空间 -> 电机空间（发指令时使用）
float SingleMotorCtrlTest::joint_to_motor_pos(float joint_pos) const {
    return config_.direction * joint_pos + config_.zero_offset;
}

float SingleMotorCtrlTest::joint_to_motor_vel(float joint_vel) const {
    return config_.direction * joint_vel;
}

// 电机空间 -> 关节空间（读状态时使用）
float SingleMotorCtrlTest::motor_to_joint_pos(float motor_pos) const {
    return config_.direction * (motor_pos - config_.zero_offset);
}

float SingleMotorCtrlTest::motor_to_joint_vel(float motor_vel) const {
    return config_.direction * motor_vel;
}


// ── ROS2 回调 ────────────────────────────────────────

void SingleMotorCtrlTest::cmd_callback(
    const std::shared_ptr<sensor_msgs::msg::JointState> msg)
{
    if (msg->position.empty()) {
        RCLCPP_WARN(get_logger(), "Received empty command, ignored.");
        return;
    }

    const float joint_pos = msg->position[0];
    const float joint_vel = msg->velocity.empty() ? 0.f : msg->velocity[0];
    const float joint_eff = msg->effort.empty()   ? 0.f : msg->effort[0];

    const float motor_pos = joint_to_motor_pos(joint_pos);
    const float motor_vel = joint_to_motor_vel(joint_vel);
    const float motor_eff = config_.direction * joint_eff;  // 力矩方向同样翻转

    if (config_.motor_type == MotorType::DM) {
        dm_motor_->MotorMitModeCmd(
            motor_pos, motor_vel, config_.kp, config_.kd, motor_eff);
    } else {
        encos_motor_->MotorMitCtrlCmd(
            motor_pos, motor_vel, config_.kp, config_.kd, motor_eff);
    }

    RCLCPP_INFO(get_logger(),
                 "cmd  joint(pos=%.3f vel=%.3f eff=%.3f)"
                 " -> motor(pos=%.3f vel=%.3f eff=%.3f)",
                 joint_pos, joint_vel, joint_eff,
                 motor_pos, motor_vel, motor_eff);
}

void SingleMotorCtrlTest::timer_publish_state() {

    float raw_pos, raw_vel, raw_effort;

    if (config_.motor_type == MotorType::DM) {
        dm_motor_->MotorMitModeCmd(0, 0, config_.kp, config_.kd, 0);
    } else {
        encos_motor_->MotorMitCtrlCmd(0, 0, config_.kp, config_.kd, 0);
    }


    if (config_.motor_type == MotorType::DM) {
        raw_pos    = dm_motor_->get_motor_pos();
        raw_vel    = dm_motor_->get_motor_spd();
        raw_effort = dm_motor_->get_motor_current();
    } else {
        raw_pos    = encos_motor_->get_motor_pos();
        raw_vel    = encos_motor_->get_motor_spd();
        raw_effort = encos_motor_->get_motor_current();
    }

    auto state         = sensor_msgs::msg::JointState();
    state.header.stamp = now();
    state.name         = {"joint"};
    state.position     = {motor_to_joint_pos(raw_pos)};
    state.velocity     = {motor_to_joint_vel(raw_vel)};
    state.effort       = {config_.direction * raw_effort};

    state_pub_->publish(state);

    RCLCPP_INFO(get_logger(),"state  motor_pos=%.3f -> joint_pos=%.3f", raw_pos, state.position[0]);
    RCLCPP_INFO(get_logger(),"state  motor_vel=%.3f -> joint_vel=%.3f", raw_vel, state.velocity[0]);
    RCLCPP_INFO(get_logger(),"state  motor_tor=%.3f -> joint_tor=%.3f", raw_effort, state.effort[0]);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SingleMotorCtrlTest>(make_test_config()));
    rclcpp::shutdown();
    return 0;
}
