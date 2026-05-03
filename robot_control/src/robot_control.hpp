#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>


class Robot_Control: public rclcpp::Node 
{
    public:
        Robot_Control();

        void model_timer_callback();
        void watchdog_timer_callback();

        void joint_cmd_publish();
        void publish_group_command(
            const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr& publisher,
            int dof,
            float pos,
            float vel,
            float effort);
        bool validate_joint_state(
            const std::shared_ptr<sensor_msgs::msg::JointState>& msg,
            const char* group_name,
            size_t expected_dof);

        //发布订阅函数
        void subs_left_leg_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg);
        void subs_right_leg_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg);
        void subs_left_arm_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg);
        void subs_right_arm_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg);
        void subs_imu_callback(const std::shared_ptr<sensor_msgs::msg::Imu> msg);
        void subs_cmd_callback(const std::shared_ptr<geometry_msgs::msg::Twist> msg);


    private:   
        // 参数化控制配置
        int control_period_ms_{20};
        int watchdog_period_ms_{20};
        int cmd_timeout_ms_{300};
        int feedback_log_period_ms_{1000};
        bool verbose_log_{false};
        int left_leg_dof_{3};
        int right_leg_dof_{3};
        int left_arm_dof_{3};
        int right_arm_dof_{3};
        float nominal_joint_pos_{0.0f};
        float cmd_vel_to_joint_vel_gain_{1.0f};
        float yaw_to_joint_vel_gain_{0.5f};

        // 订阅控制指令
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscription_;

        // 订阅IMU
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;

        // 订阅四肢关节信息
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_leg_subscription_, right_leg_subscription_, left_arm_subscription_, right_arm_subscription_;

        // 发布四肢关节指令
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_leg_publisher_, right_leg_publisher_, left_arm_publisher_, right_arm_publisher_;

        // 控制定时器与命令超时看门狗
        rclcpp::TimerBase::SharedPtr Model_timer;
        rclcpp::TimerBase::SharedPtr watchdog_timer_;

        std::mutex cmd_mutex_;
        float cmd_linear_x_{0.0f};
        float cmd_angular_z_{0.0f};
        std::atomic<int64_t> last_cmd_ns_{0};
        std::atomic<bool> cmd_timed_out_{true};
};
