#pragma once

#include <atomic>
#include <string>
#include "can_msgs/msg/frame.hpp"
#include "SocketCAN.hpp"
#include "utils.hpp"
#include "timer.hpp"
#include <iostream>
#include <math.h>

class EncosMotorDriver
{
   public:
    EncosMotorDriver(uint16_t motor_id, std::string can_interface, uint16_t master_id);
    ~EncosMotorDriver();

    // 力位混合控制
    void MotorMitCtrlCmd(float pos,float spd,float kp,float kd,float tor);
    void MotorSetting(uint8_t cmd);

    uint8_t get_motor_id() { return motor_id_; }
    uint8_t get_error_id() { return error_id_; }
    float get_motor_pos() { return motor_pos_; }
    float get_motor_spd() { return motor_spd_; }
    float get_motor_current() { return motor_current_; }
    float get_motor_temperature() { return motor_temperature_; }

   private:
    void CanRxMsgCallback(const can_frame& rx_frame);  

    const float KP_MIN = 0.0f;
    const float KP_MAX = 500.0f;
    const float KD_MIN = 0.0f;
    const float KD_MAX = 5.0f;
    const float POS_MIN = -12.5f;
    const float POS_MAX = 12.5f;
    const float SPD_MIN = -18.0f;
    const float SPD_MAX = 18.0f;
    const float T_MIN = -30.0f;
    const float T_MAX = 30.0f;
    const float I_MIN = -30.0f;
    const float I_MAX = 30.0f;

    std::string can_interface_;
    std::shared_ptr<SocketCAN> can_;
    std::shared_ptr<spdlog::logger> logger_;
    
    std::atomic<int> response_count_{0};
    std::atomic<uint8_t> mos_temperature_{0};
    std::atomic<uint8_t> motor_temperature_{0};
    std::atomic<uint8_t> error_id_{0};
    std::atomic<float> motor_pos_{0.f};
    std::atomic<float> motor_spd_{0.f};
    std::atomic<float> motor_current_{0.f};

    uint16_t motor_id_;
    uint16_t master_id_;
};
