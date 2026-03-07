#pragma once

#include <atomic>
#include <string>
#include "can_msgs/msg/frame.hpp"
#include "SocketCAN.hpp"
#include "utils.hpp"
#include "timer.hpp"
#include <iostream>

enum DMError {
    DM_DOWN = 0x00,
    DM_UP = 0x01,
    OVER_VOLT = 0x08,
    UNDER_VOLT = 0x09,
    OVER_CURRENT = 0x0A,
    MOS_OVER_TEMP = 0x0B,
    COIL_OVER_TEMP = 0x0C,
    LOST_CONN = 0x0D,
    OVER_LOAD = 0x0E,
};

enum DM_Motor_Model { DM4310_24V, DM6006L_48V, Num_Of_Motor };

enum DM_REG {
    UV_Value = 0,
    KT_Value = 1,
    OT_Value = 2,
    OC_Value = 3,
    ACC = 4,
    DEC = 5,
    MAX_SPD = 6,
    MST_ID = 7,
    ESC_ID = 8,
    TIMEOUT = 9,
    CTRL_MODE = 10,
    Damp = 11,
    Inertia = 12,
    hw_ver = 13,
    sw_ver = 14,
    SN = 15,
    NPP = 16,
    Rs = 17,
    LS = 18,
    Flux = 19,
    Gr = 20,
    PMAX = 21,
    VMAX = 22,
    TMAX = 23,
    I_BW = 24,
    KP_ASR = 25,
    KI_ASR = 26,
    KP_APR = 27,
    KI_APR = 28,
    OV_Value = 29,
    GREF = 30,
    Deta = 31,
    V_BW = 32,
    IQ_c1 = 33,
    VL_c1 = 34,
    can_br = 35,
    sub_ver = 36,
    u_off = 50,
    v_off = 51,
    k1 = 52,
    k2 = 53,
    m_off = 54,
    dir = 55,
    p_m = 80,
    xout = 81,
};

typedef struct {
    float PosMax;
    float SpdMax;
    float TauMax;
} Limit_param;

class DmMotorDriver
{
   public:
    DmMotorDriver(uint16_t motor_id, std::string can_interface, uint16_t master_id,
                  DM_Motor_Model motor_model);
    ~DmMotorDriver();

    void MotorEna();
    void MotorDisEna();
    uint8_t MotorInit();
    void MotorDeInit();
    bool MotorSetZero();
    bool MotorWriteFlash();

    void MotorGetParam(uint8_t param_cmd);
    void MotorPosModeCmd(float pos, float spd, bool ignore_limit);
    void MotorSpdModeCmd(float spd);
    void MotorMitModeCmd(float f_p, float f_v, float f_kp, float f_kd, float f_t);
    void set_motor_control_mode(uint8_t motor_control_mode);
    int get_response_count() const {return response_count_;}
    void refresh_motor_status();

    uint8_t get_motor_id() { return motor_id_; }
    uint8_t get_motor_control_mode() { return motor_control_mode_; }
    uint8_t get_error_id() { return error_id_; }
    float get_motor_pos() { return motor_pos_; }
    float get_motor_spd() { return motor_spd_; }
    float get_motor_current() { return motor_current_; }
    float get_motor_temperature() { return motor_temperature_; }

   private:
    std::string can_interface_;
    bool param_cmd_flag_[30] = {false};
    const float KpMin = 0.0f;
    const float KpMax = 500.0f;
    const float KdMin = 0.0f;
    const float KdMax = 5.0f;
    DM_Motor_Model motor_model_;
    Limit_param limit_param_;

    void DmMotorSetZero();
    void DmMotorClearError();
    void DmWriteRegister(uint8_t rid, float value);
    void DmWriteRegister(uint8_t rid, int32_t value);
    void DmSaveRegister(uint8_t rid);
    void CanRxMsgCallback(const can_frame& rx_frame);

    std::shared_ptr<SocketCAN> can_;
    std::shared_ptr<spdlog::logger> logger_;
    
    std::atomic<int> response_count_{0};
    std::atomic<uint8_t> mos_temperature_{0};
    std::atomic<float> motor_temperature_{0.f};
    std::atomic<uint8_t> error_id_{0};
    std::atomic<float> motor_pos_{0.f};
    std::atomic<float> motor_spd_{0.f};
    std::atomic<float> motor_current_{0.f};

    int normal_sleep_time = 5;
    int setup_sleep_time = 500;

    uint16_t motor_id_;
    uint16_t master_id_;

    uint8_t motor_control_mode_;  // 0:none 1:pos 2:spd 3:mit
    enum MotorControlMode_e {
        NONE = 0,
        MIT = 1,
        POS = 2,
        SPD = 3,
    };

};

using union32_t = union Union32 {
    float f;
    int32_t i;
    uint32_t u;
    uint8_t buf[4];
};
