#pragma once

#include <cmath>
#include <cstdint>
#include <numbers>
#include <algorithm>

#include <atomic>
#include <iostream> 

#include "../utility/cross_os.hpp"

namespace librmcs::device {

class VescMotor {
public:
        //   enum class  Type : uint8_t { U10 };
    struct Config {
        Config() 
        :pole_pairs(7)
        ,kt_rough(0.0562)
        ,reversed(false)
        ,multi_turn_angle_enabled(false)
        {}
        
        Config& set_pole_pairs(int value) { return pole_pairs = value, *this;}
        Config& set_kt_rough(double value) { return kt_rough = value, *this;}
        Config& set_reversed() { return reversed = true, *this;}
        Config& enable_multi_turn_angle() { return multi_turn_angle_enabled = true, *this;}

        int pole_pairs;                             // 极对数
        double kt_rough;                            // 转矩常数
        bool reversed;                              // 是否反转
        bool multi_turn_angle_enabled;              // 多圈模式
    };

    VescMotor() = default;
    explicit VescMotor(const Config& config) 
        : VescMotor() {
            configure(config);
    }

    VescMotor(const VescMotor&) = delete;
    VescMotor& operator=(const VescMotor&) = delete;

    void configure(const Config& config) {
        pole_pairs_ = config.pole_pairs;
        kt_ = config.kt_rough;
        sign_ = config.reversed ? -1.0 : 1.0;
        multi_turn_angle_enabled_ = config.multi_turn_angle_enabled;

        last_pid_pos_ = std::numeric_limits<double>::quiet_NaN();
        encoder_zero_offset_ = 0.0;
        angle_ = 0.0;
        velocity_ = 0.0;
        current_ = 0.0;
        temperature_ = 0.0;

        // can_data_.store(0, std::memory_order_relaxed);
        can_data_status_4_.store(0, std::memory_order_relaxed);
        can_data_status_1_.store(0, std::memory_order_relaxed);
    }

    void store_status(uint32_t can_id, uint64_t can_data) {
        uint8_t cmd = (can_id >> 8) & 0xFF;

        switch (cmd) {
        case 9:
            can_data_status_1_.store(can_data, std::memory_order_relaxed);
            break;
        case 16:
            can_data_status_4_.store(can_data, std::memory_order_relaxed);
            break;
        default:
            break;
        }
    }

    void update_status() {
        // 读取位置+电流+温度
        uint64_t data_s4 = can_data_status_4_.load(std::memory_order_relaxed);
        if (data_s4 == 0) return;

        temperature_ = be16(data_s4, 16) * 0.1;             // B2-B3: 温度 (°C)
        current_ = be16(data_s4, 32) * 0.1;                 // B4-B5: 电流 (A)
        double pid_pos = be16(data_s4, 48) * 0.02;          // B6-B7: 位置 (度)

        pid_pos -= encoder_zero_offset_;

        uint64_t data_s1 = can_data_status_1_.load(std::memory_order_relaxed);
        if (data_s1 != 0) 
        {
            int32_t erpm = be32(data_s1, 0);

            // 机械转速 = 电转速 / 极对数
            double mechanicl_rpm = static_cast<double>(erpm) / pole_pairs_;
            //std::cout << "mechanicl_rpm: " << mechanicl_rpm << std::endl;
             // rad/s = (RPM / 60) × 2π
             double raw_velocity = sign_ * mechanicl_rpm * (2.0 * std::numbers::pi / 60.0);
             double lb = 0.1;
             velocity_ = velocity_ * lb + raw_velocity * (1 - lb);

        }

        if(!multi_turn_angle_enabled_)//单圈模式
        {
            angle_ = sign_ * pid_pos * DEG_TO_RAD;
            // 确保角度在0-2π之间
            if(angle_ < 0)
            {
                angle_ += 2.0 * std::numbers::pi;
            }
        }
        else 
        {
            // 多圈模式：累积角度变化量
            if(!std::isnan(last_pid_pos_)){
                double delta = pid_pos - last_pid_pos_;

                if(delta >180.0)
                    delta -= 360.0;
                else if (delta <- 180.0)
                    delta += 360.0;    

                angle_ += sign_ * delta * DEG_TO_RAD;
            }

            last_pid_pos_ = pid_pos;
        }
    }

// current = torque / Kt 电流 = 扭矩 / 转矩常数
    double torque_to_current(double torque, double max_current = 35.0)  const{
           if (std::isnan(torque)) 
                return 0.0;
           return sign_ * std::clamp(torque / kt_, -max_current, max_current);
    }
// 速度→rpm转换   60 / 2π
    double velocity_to_rpm(double velocity, double max_velocity = 2000) const{
         if (std::isnan(velocity))
            return 0.0;
         return sign_ * std::clamp(velocity * 60.0 / (2.0 * std::numbers::pi), -max_velocity, max_velocity);
    }

    int calibrate_zero_point() {
        // 读取当前编码器位置
        uint64_t data_s4 = can_data_status_4_.load(std::memory_order_relaxed);
        if (data_s4 != 0) {
          
            double raw_encoder_pos = be16(data_s4, 48) * 0.02;
            
            encoder_zero_offset_ = raw_encoder_pos;
            
            // 如果是多圈模式，重置跟踪点
            if (multi_turn_angle_enabled_) {
                last_pid_pos_ = 0.0;  // 重置为零点后的位置
            }
        }
        
        // 清零角度
        double old_angle_deg = angle_ * 180.0 / std::numbers::pi;
        angle_ = 0.0;
        
        return static_cast<int>(old_angle_deg);
    }

    double angle() const { return angle_; }
    double velocity() const { return velocity_; }
    double temperature() const { return temperature_; }
    double max_torque() const { return 30.0 * kt_; }
    double current() const { return current_; }
    double kt() const { return kt_; }
    
private:
    // 解析32位大端序整数
    static int32_t be32(uint64_t data, int shift){
        uint32_t raw = static_cast<uint32_t>(data >> shift);
        return static_cast<int32_t>(__builtin_bswap32(raw)); //__builtin_bswap32要求
    }

    // 解析16位大端序整数
    static double be16(uint64_t data, int shift){
        return __builtin_bswap16(static_cast<int16_t>(data >> shift));
    }

    // 度→弧度转换系数：π/180
    static constexpr double DEG_TO_RAD = std::numbers::pi / 180.0;

    int pole_pairs_ = 21;
    double kt_ = 0.0562;
    double sign_ = 0;                                // 方向系数（1或-1）
    bool multi_turn_angle_enabled_ = false;
    double encoder_zero_offset_ = 0.0;

    std::atomic<uint64_t> can_data_ = 0;             // CAN数据缓冲区
    std::atomic<uint64_t> can_data_status_4_ = 0;    // STATUS_4数据缓
    std::atomic<uint64_t> can_data_status_1_ = 0;    // STATUS_1数据缓

    double last_pid_pos_ = std::numeric_limits<double>::quiet_NaN();

    double angle_ = 0.0;              // 当前角度 (rad)
    double velocity_ = 0.0;           // 当前速度 (rad/s)
    double current_ = 0.0;            // 当前电流 (A)
    double temperature_ = 0.0;        // 当前温度 (°C)
};

}