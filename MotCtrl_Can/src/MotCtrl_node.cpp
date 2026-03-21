#include "MotCtrl_node.hpp"
#include <algorithm>
#include <cctype>
#include <sstream>

// 功能：校验输入 JointState 维度是否满足当前链路自由度要求。
bool MotorsNode::validate_joint_command(
    const std::shared_ptr<sensor_msgs::msg::JointState>& msg,
    const std::string& group_name,
    size_t expected_dof) {
    if (msg->position.size() < expected_dof ||
        msg->velocity.size() < expected_dof ||
        msg->effort.size() < expected_dof) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "[%s] invalid JointState sizes, expected >=%zu but got pos=%zu vel=%zu eff=%zu",
            group_name.c_str(), expected_dof, msg->position.size(), msg->velocity.size(), msg->effort.size());
        return false;
    }
    return true;
}

// 功能：将 int64 方向数组参数转换为内部 int 方向数组。
std::vector<int> MotorsNode::to_int_dirs(const std::vector<int64_t>& dirs64) const {
    std::vector<int> values;
    values.reserve(dirs64.size());
    for (const auto v : dirs64) {
        if (v > 1 || v < -1) {
            RCLCPP_WARN(
                this->get_logger(),
                "Direction value %lld is unusual, expected -1 or 1.",
                static_cast<long long>(v));
        }
        values.push_back(static_cast<int>(v));
    }
    if (values.empty()) {
        values.push_back(1);
    }
    return values;
}

// 功能：标准化链路电机类型参数，仅接受 dm 或 encos。
std::string MotorsNode::normalize_motor_type(const std::string& type, const char* can_name) const {
    std::string lower = type;
    std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) { return std::tolower(c); });
    if (lower == "dm" || lower == "encos") {
        return lower;
    }
    RCLCPP_WARN(
        this->get_logger(),
        "[%s] unsupported motor type '%s', fallback to 'dm'.",
        can_name, type.c_str());
    return "dm";
}

// 功能：解析每链路电机类型列表（csv）；为空时全链路回退为 dm。
// 约束：非空配置时，长度必须与 expected_count 严格一致。
std::vector<std::string> MotorsNode::parse_motor_types(
    const std::string& csv, size_t expected_count, const char* can_name) const {
    std::vector<std::string> result;
    const std::string normalized_fallback = "dm";
    if (csv.empty()) {
        result.assign(expected_count, normalized_fallback);
        return result;
    }

    std::stringstream ss(csv);
    std::string token;
    while (std::getline(ss, token, ',')) {
        token.erase(std::remove_if(token.begin(), token.end(), [](unsigned char c) { return std::isspace(c); }), token.end());
        if (!token.empty()) {
            result.push_back(normalize_motor_type(token, can_name));
        }
    }

    if (result.size() != expected_count) {
        RCLCPP_ERROR(
            this->get_logger(),
            "[%s] invalid motor type list size: expected %zu, got %zu. "
            "Use empty string for all-dm fallback or provide exact csv list.",
            can_name, expected_count, result.size());
        result.clear();
    }

    return result;
}

// 功能：将电机类型列表拼接成日志可读字符串。
std::string MotorsNode::join_motor_types(const std::vector<std::string>& types) const {
    std::stringstream ss;
    for (size_t i = 0; i < types.size(); ++i) {
        if (i > 0) {
            ss << ",";
        }
        ss << types[i];
    }
    return ss.str();
}

// 功能：按链路每个电机类型（DM/ENCOS）创建驱动对象。
void MotorsNode::init_motor_group(
    MotorGroup& group,
    int start_id,
    int end_id,
    const std::string& can_interface,
    const std::vector<std::string>& motor_types) {
    for (int id = start_id; id <= end_id; ++id) {
        const size_t idx = static_cast<size_t>(id - start_id);
        if (motor_types[idx] == "encos") {
            group[idx] = MotorWrapper::create_encos(static_cast<uint16_t>(id), can_interface, static_cast<uint16_t>(id + 0x10));
        } else {
            group[idx] = MotorWrapper::create_dm(static_cast<uint16_t>(id), can_interface, static_cast<uint16_t>(id + 0x10), DM4310_24V);
        }
    }
}

// 功能：将方向数组长度与电机数量对齐（不足补 +1，超出截断）。
void MotorsNode::normalize_dirs(std::vector<int>& dirs, size_t expected_dof, const std::string& group_name) {
    if (dirs.size() < expected_dof) {
        RCLCPP_WARN(
            this->get_logger(),
            "[%s] direction size (%zu) < dof (%zu), filling missing entries with +1.",
            group_name.c_str(), dirs.size(), expected_dof);
        dirs.resize(expected_dof, 1);
    } else if (dirs.size() > expected_dof) {
        RCLCPP_WARN(
            this->get_logger(),
            "[%s] direction size (%zu) > dof (%zu), truncating extras.",
            group_name.c_str(), dirs.size(), expected_dof);
        dirs.resize(expected_dof);
    }
}

// 功能：接收并缓存某条链路的控制目标，供固定频率控制循环读取。
bool MotorsNode::cache_joint_command(
    const std::shared_ptr<sensor_msgs::msg::JointState>& msg,
    const std::string& group_name,
    const std::vector<int>& dirs,
    JointCommand& cmd_cache,
    const char* recovered_log) {
    if (!is_init_.load()) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Ignore %s command because motors are not initialized.", group_name.c_str());
        return false;
    }
    if (!validate_joint_command(msg, group_name, dirs.size())) {
        return false;
    }

    // 订阅线程只负责缓存目标，真正下发由控制定时器完成
    JointCommand cmd;
    cmd.pos.resize(dirs.size());
    cmd.vel.resize(dirs.size());
    cmd.effort.resize(dirs.size());
    for (size_t i = 0; i < dirs.size(); ++i) {
        cmd.pos[i] = static_cast<float>(msg->position[i] * dirs[i]);
        cmd.vel[i] = static_cast<float>(msg->velocity[i] * dirs[i]);
        cmd.effort[i] = static_cast<float>(msg->effort[i] * dirs[i]);
    }
    cmd.valid = true;

    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        cmd_cache = cmd;
    }
    last_cmd_ns_.store(this->now().nanoseconds());
    if (cmd_timed_out_.exchange(false)) {
        RCLCPP_INFO(this->get_logger(), "%s", recovered_log);
    }
    return true;
}

// 功能：按链路维度发布关节状态话题。
void MotorsNode::publish_group(
    const MotorGroup& motors, const std::vector<int>& dirs, const JointPublisher& publisher) {
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->now();
    msg.name.reserve(motors.size());
    msg.position.reserve(motors.size());
    msg.velocity.reserve(motors.size());
    msg.effort.reserve(motors.size());
    // 发布维度以“电机数量与方向数量的较小值”为准，避免越界
    const size_t dof = std::min(motors.size(), dirs.size());
    for (size_t i = 0; i < dof; ++i) {
        msg.name.push_back("joint" + std::to_string(i + 1));
        msg.position.push_back(motors[i]->get_motor_pos() * dirs[i]);
        msg.velocity.push_back(motors[i]->get_motor_spd() * dirs[i]);
        msg.effort.push_back(motors[i]->get_motor_current() * dirs[i]);
    }
    publisher->publish(msg);
}

// 功能：初始化一条 CAN 链路上的全部电机，并汇总初始化结果。
bool MotorsNode::init_group(const MotorGroup& motors, int start_id, const char* can_name) {
    bool ok = true;
    for (size_t i = 0; i < motors.size(); ++i) {
        const uint8_t ret = motors[i]->MotorInit();
        if (ret != DMError::DM_UP) {
            ok = false;
            RCLCPP_ERROR(
                this->get_logger(), "Init failed on %s motor id=%d, err=0x%02x",
                can_name, static_cast<int>(start_id + i), ret);
        }
        Timer::ThreadSleepForUs(200);
    }
    return ok;
}

// 功能：失能一条 CAN 链路上的全部电机。
void MotorsNode::deinit_group(const MotorGroup& motors) {
    for (const auto& motor : motors) {
        motor->MotorDeInit();
        Timer::ThreadSleepForUs(200);
    }
}

// 功能：对一条 CAN 链路上的全部电机执行设零。
void MotorsNode::set_zero_group(const MotorGroup& motors) {
    for (const auto& motor : motors) {
        motor->MotorSetZero();
        Timer::ThreadSleepForUs(200);
    }
}

// 功能：主动刷新一条 CAN 链路上的电机状态缓存。
void MotorsNode::refresh_group(const MotorGroup& motors) {
    for (const auto& motor : motors) {
        motor->refresh_motor_status();
        Timer::ThreadSleepForUs(200);
    }
}

// 功能：固定频率读取缓存目标并下发到各链路电机。
void MotorsNode::apply_joint_command() {
    JointCommand left_leg_cmd;
    JointCommand right_leg_cmd;
    JointCommand left_arm_cmd;
    JointCommand right_arm_cmd;
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        left_leg_cmd = left_leg_cmd_;
        right_leg_cmd = right_leg_cmd_;
        left_arm_cmd = left_arm_cmd_;
        right_arm_cmd = right_arm_cmd_;
    }

    // 控制回路统一从缓存读取，减少订阅回调抖动对时序的影响
    auto apply_group = [this](const MotorGroup& motors, const JointCommand& cmd) {
        if (!cmd.valid) {
            return;
        }
        const size_t dof = std::min({motors.size(), cmd.pos.size(), cmd.vel.size(), cmd.effort.size()});
        for (size_t i = 0; i < dof; ++i) {
            motors[i]->MotorMitModeCmd(cmd.pos[i], cmd.vel[i], Kp_MIT, Kd_MIT, cmd.effort[i]);
        }
    };
    apply_group(left_leg_motors_DM, left_leg_cmd);
    apply_group(right_leg_motors_DM, right_leg_cmd);
    apply_group(left_arm_motors_DM, left_arm_cmd);
    apply_group(right_arm_motors_DM, right_arm_cmd);
}

// 功能：进入安全输出（保持当前位置，速度/力矩置零）。
void MotorsNode::send_safe_command() {
    auto send_group_safe = [this](const MotorGroup& motors) {
        for (const auto& motor : motors) {
            motor->MotorMitModeCmd(motor->get_motor_pos(), 0.0f, Kp_MIT, Kd_MIT, 0.0f);
        }
    };
    send_group_safe(left_leg_motors_DM);
    send_group_safe(right_leg_motors_DM);
    send_group_safe(left_arm_motors_DM);
    send_group_safe(right_arm_motors_DM);
}

// 功能：控制主循环定时器回调，负责周期下发与状态发布。
void MotorsNode::control_timer_callback() {
    if (!is_init_.load()) {
        return;
    }
    // 超时后由 watchdog 输出安全命令，控制回路暂停常规下发
    if (cmd_timed_out_.load()) {
        return;
    }

    apply_joint_command();
    publish_left_leg();
    publish_right_leg();
    publish_left_arm();
    publish_right_arm();
}

// 功能：看门狗定时器回调，检测命令超时并切换到安全输出。
void MotorsNode::watchdog_timer_callback() {
    if (!is_init_.load()) {
        return;
    }

    const int64_t now_ns = this->now().nanoseconds();
    const int64_t last_ns = last_cmd_ns_.load();
    const int64_t elapsed_ms = (now_ns - last_ns) / 1000000;

    // 长时间无新命令时进入安全模式（保持当前位置，速度/力矩清零）
    if (elapsed_ms > cmd_timeout_ms_) {
        if (!cmd_timed_out_.exchange(true)) {
            RCLCPP_WARN(
                this->get_logger(),
                "Command timeout detected (%lld ms > %d ms), sending safe MIT command.",
                static_cast<long long>(elapsed_ms), cmd_timeout_ms_);
        }
        send_safe_command();
    }
}

// 功能：发布左腿关节状态。
void MotorsNode::publish_left_leg() {
    publish_group(
        left_leg_motors_DM,
        left_leg_joint_dirs_,
        left_leg_publisher_);
}

// 功能：发布右腿关节状态。
void MotorsNode::publish_right_leg() {
    publish_group(
        right_leg_motors_DM,
        right_leg_joint_dirs_,
        right_leg_publisher_);
}

// 功能：发布左臂关节状态。
void MotorsNode::publish_left_arm() {    
    publish_group(
        left_arm_motors_DM,
        left_arm_joint_dirs_,
        left_arm_publisher_);
}

// 功能：发布右臂关节状态。
void MotorsNode::publish_right_arm() {
    publish_group(
        right_arm_motors_DM,
        right_arm_joint_dirs_,
        right_arm_publisher_);
}

// 功能：左腿命令订阅回调（仅缓存，不直接下发）。
void MotorsNode::subs_left_leg_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg) {
    cache_joint_command(
        msg, "left_leg",
        left_leg_joint_dirs_,
        left_leg_cmd_, "Left leg command stream recovered.");
}

// 功能：右腿命令订阅回调（仅缓存，不直接下发）。
void MotorsNode::subs_right_leg_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg) {
    cache_joint_command(
        msg, "right_leg",
        right_leg_joint_dirs_,
        right_leg_cmd_, "Right leg command stream recovered.");
}

// 功能：左臂命令订阅回调（仅缓存，不直接下发）。
void MotorsNode::subs_left_arm_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg) {
    cache_joint_command(
        msg, "left_arm",
        left_arm_joint_dirs_,
        left_arm_cmd_, "Left arm command stream recovered.");
}

// 功能：右臂命令订阅回调（仅缓存，不直接下发）。
void MotorsNode::subs_right_arm_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg) {
    cache_joint_command(
        msg, "right_arm",
        right_arm_joint_dirs_,
        right_arm_cmd_, "Right arm command stream recovered.");
}


// 功能：初始化全部链路电机并发布首帧状态。
void MotorsNode::init_motors() {
    bool all_init_ok = true;
    all_init_ok &= init_group(left_leg_motors_DM, can0_startID_, "can0");
    all_init_ok &= init_group(right_leg_motors_DM, can1_startID_, "can1");
    all_init_ok &= init_group(left_arm_motors_DM, can2_startID_, "can2");
    all_init_ok &= init_group(right_arm_motors_DM, can3_startID_, "can3");
    if (!all_init_ok) {
        is_init_.store(false);
        RCLCPP_ERROR(
            this->get_logger(),
            "Motor initialization failed. Node stays in non-initialized state and ignores commands.");
        return;
    }
    Timer::ThreadSleepFor(1000);
    publish_left_leg();
    publish_right_leg();
    publish_left_arm();
    publish_right_arm();
    last_cmd_ns_.store(this->now().nanoseconds());
    is_init_.store(true);
}

// 功能：失能全部链路电机。
void MotorsNode::deinit_motors() {
    deinit_group(left_leg_motors_DM);
    deinit_group(right_leg_motors_DM);
    deinit_group(left_arm_motors_DM);
    deinit_group(right_arm_motors_DM);
    is_init_.store(false);
}

// 功能：对全部链路执行设零操作。
void MotorsNode::set_zeros() {
    if(!is_init_.load()){
        RCLCPP_WARN(this->get_logger(), "Motors are not initialized, cannot set zeros.");
        return;
    }
    set_zero_group(left_leg_motors_DM);
    set_zero_group(right_leg_motors_DM);
    set_zero_group(left_arm_motors_DM);
    set_zero_group(right_arm_motors_DM);
}

// 功能：主动轮询全部链路状态并发布。
void MotorsNode::read_motors(){
    refresh_group(left_leg_motors_DM);
    Timer::ThreadSleepForUs(1000);
    publish_left_leg();

    refresh_group(right_leg_motors_DM);
    Timer::ThreadSleepForUs(1000);
    publish_right_leg();

    refresh_group(left_arm_motors_DM);
    Timer::ThreadSleepForUs(1000);
    publish_left_arm();

    refresh_group(right_arm_motors_DM);
    Timer::ThreadSleepForUs(1000);
    publish_right_arm();

}


// 功能：节点入口，初始化并进入 ROS2 事件循环。
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorsNode>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}