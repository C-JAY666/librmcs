#pragma once


#include <algorithm>
#include <cmath>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include <mutex>
#include <opencv4/opencv2/core/hal/interface.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include "../utility/cross_os.hpp"

namespace librmcs::device {

    struct ELRSData
    {
        std::array<uint16_t, 16> channels;
    };

    struct KFSData
    {
        std::array<uint16_t, 12> channels;
    };

class Elrs {
public:
    explicit Elrs() = default;

    ELRSData get_data() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return current_data_;
    }

    KFSData get_kfs_data() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return kfs_data_;
    }

    void update_status()
    {
        auto channel_to_double = [](int32_t value) 
        {
            double v = static_cast<double>(value) - 992.0;
            constexpr double dead_zone = 0.0;

            if (std::abs(v) < dead_zone)
                return 0.0;

            double result = (v > 0) ? (v - dead_zone) / 819.0 : (v + dead_zone) / 820.0;

            return std::clamp(result, -1.0, 1.0);
        };

        auto switch_to_double = [](int32_t value)
        {
            if (value > 992)
                return Switch::UP; 
            else if (value < 992)
                return Switch::DOWN;
            else
                return Switch::MIDDLE;             
        };

        auto key_to_double = [](int32_t value)
        {
            return (value > 992) ? 1 : 0;
        };

        joystick_right_.y =  channel_to_double(static_cast<uint16_t>(current_data_.channels[0]));
        joystick_right_.x =  channel_to_double(static_cast<uint16_t>(current_data_.channels[1]));
        joystick_left_.y  =  channel_to_double(static_cast<uint16_t>(current_data_.channels[2]));
        joystick_left_.x  =  channel_to_double(static_cast<uint16_t>(current_data_.channels[3]));

        switch_right_ =  switch_to_double(static_cast<uint16_t>(current_data_.channels[4]));
        switch_left_  =  switch_to_double(static_cast<uint16_t>(current_data_.channels[5]));

        key.k1 = key_to_double(static_cast<uint16_t>(current_data_.channels[6]));
        key.k2 = key_to_double(static_cast<uint16_t>(current_data_.channels[7]));
        key.k3 = key_to_double(static_cast<uint16_t>(current_data_.channels[8]));
        key.k4 = key_to_double(static_cast<uint16_t>(current_data_.channels[9]));
        key.k5 = key_to_double(static_cast<uint16_t>(current_data_.channels[10]));
        key.k6 = key_to_double(static_cast<uint16_t>(current_data_.channels[11]));
        key.k7 = key_to_double(static_cast<uint16_t>(current_data_.channels[12]));
        key.k8 = key_to_double(static_cast<uint16_t>(current_data_.channels[13]));
        key.k9 = key_to_double(static_cast<uint16_t>(current_data_.channels[14]));
        key.k10= key_to_double(static_cast<uint16_t>(current_data_.channels[15]));
    }

    void store_status(const std::byte* uart_data, size_t uart_data_length)
    {
    static uint8_t rxBuffer[static_cast<size_t>(CRSF_ID::CRSF_MAX_FRAME_SIZE)];
    static uint8_t rxPos = 0;
    static uint8_t expectedLength = 0;

    uint8_t byteReceived = std::to_integer<uint8_t>(*(uart_data));

            if (rxPos == 0) { // 等待同步字节
            if (byteReceived == static_cast<uint8_t>(CRSF_ID::CRSF_SYNC_BYTE)) {
                rxBuffer[rxPos++] = byteReceived;
            }
        } else if (rxPos == 1) { // 接收长度字节
            if (byteReceived >= (static_cast<uint8_t>(CRSF_ID::CRSF_MIN_FRAME_SIZE) - 2) && byteReceived <= (static_cast<uint8_t>(CRSF_ID::CRSF_MAX_FRAME_SIZE) - 2) ) { // 长度字节范围校验 (Type+Payload+CRC)
                rxBuffer[rxPos++] = byteReceived;
                expectedLength = byteReceived + 2; // 完整帧长度 = Length_Field + SyncByte + LengthByte
            } else { // 无效的长度字节，重置
                rxPos = 0;
            }
        } else { // 接收剩余数据 (Type, Payload, CRC)
            rxBuffer[rxPos++] = byteReceived;
            if (rxPos == expectedLength) { // 接收到完整帧
                // CRC校验范围: 从Type字节(rxBuffer[2])到Payload结束
                // CRC本身位于帧尾 rxBuffer[expectedLength-1]
                // 校验的数据长度 = expectedLength - 3 (排除Sync, Length, CRC本身)
                // 或者等于 Length_Field - 1 (排除CRC本身)
                uint8_t calculatedCRC = crc8(&rxBuffer[2], expectedLength - 3);
                if (calculatedCRC == rxBuffer[expectedLength - 1]) {
                    // CRC校验成功，根据类型处理数据包
                    uint8_t frameType = rxBuffer[2];
                    const uint8_t *payload = &rxBuffer[3]; // 负载数据从第3个字节开始
                    // uint8_t payloadLength = expectedLength - 4; // Sync, Length, Type, CRC

                    if (frameType == static_cast<uint8_t>(CRSF_ID::CRSF_FRAMETYPE_RC_CHANNELS_PACKED)) {
                        processCRSFRcChannelsPacket(payload);
                    } else {
                        if (frameType == static_cast<uint8_t>(CRSF_ID::CRSF_FRAMETYPE_KFS_CHANNELS_PACKED))
                        {
                            processCRSFKfsChannelsPacket(payload + 2);
                        }
                        // 在此处理其他类型的CRSF帧
                        // Serial.print("Received CRSF Frame Type: 0x");
                        // Serial.println(frameType, HEX);
                    }
                } else {
                    // Serial.println("CRC Error!");
                }
                rxPos = 0; // 重置缓冲区，准备接收下一帧
            } else if (rxPos >= static_cast<uint8_t>(CRSF_ID::CRSF_MAX_FRAME_SIZE)) { // 缓冲区溢出，通常不应发生
                RCLCPP_ERROR(rclcpp::get_logger("ELRS"), "ELRS RX Buffer Overflow");
                rxPos = 0;
            }
        }
    }
      struct Vector {
        constexpr static inline Vector zero() { return {0, 0}; }
        double x, y;
    };

    enum class Switch : uint8_t { UNKNOWN = 0, UP = 1, DOWN = 2, MIDDLE = 3 }; 

    PACKED_STRUCT(Key {
        constexpr static inline Key zero() {
            constexpr uint16_t zero = 0;
            return std::bit_cast<Key>(zero);
        }

        bool k1     : 1;
        bool k2     : 1;
        bool k3     : 1;
        bool k4     : 1;
        bool k5     : 1;
        bool k6     : 1;
        bool k7     : 1;
        bool k8     : 1;
        bool k9     : 1;
        bool k10    : 1;
        uint16_t empty : 6;

    });
    static_assert(sizeof(Key) == 2);

    Vector joystick_right() const { return joystick_right_; }
    Vector joystick_left() const { return joystick_left_; }

    Switch switch_right() const { return switch_right_; }
    Switch switch_left() const { return switch_left_; }

    Key key_status() const { return key; }

    
private:

    enum class CRSF_ID :uint32_t{
    CRSF_SYNC_BYTE  = 0xC8,
    CRSF_MAX_CHANNEL = 16,
    KFS_MAX_CHANNEL = 12,
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED  = 0x16,
    CRSF_FRAMETYPE_KFS_CHANNELS_PACKED = 0x5F,
    CRSF_MIN_FRAME_SIZE = 4,
    CRSF_MAX_FRAME_SIZE  = 64
    };

    void processCRSFRcChannelsPacket(const uint8_t *payload) {
    ELRSData local_data;
    uint64_t bitBuffer = 0;
    uint8_t bitsInBuffer = 0;
    int byteIndex = 0;

    for (int i = 0; i < static_cast<int>(CRSF_ID::CRSF_MAX_CHANNEL); i++) {
        while (bitsInBuffer < 11) {
            // 从payload中读取字节
            bitBuffer |= ((uint64_t)payload[byteIndex++] << bitsInBuffer);
            bitsInBuffer += 8;
        }
        local_data.channels[i] = bitBuffer & 0x07FF; // 取低11位作为通道值
        bitBuffer >>= 11;
        bitsInBuffer -= 11;
    }

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        current_data_ = local_data; // 内存拷贝，极快
    }

    // 打印解析出的各个通道原始值和转换后的PWM值
    // Serial.println("CRSF Channels Received:");
    // for (int i = 0; i < CRSF_MAX_CHANNEL; i++) {
    //     Serial.print("CH");
    //     Serial.print(i + 1);
    //     Serial.print(": ");
    //     Serial.print(rcChannels[i]); // 原始CRSF值 (172-1811)
    //     Serial.print(" (PWM: ");
    //     Serial.print(convertCrsfToPwm(rcChannels[i]));
    //     Serial.println("us)");
    // }
    }

    void processCRSFKfsChannelsPacket(const uint8_t *payload) {
    KFSData local_data;
    uint64_t bitBuffer = 0;
    uint8_t bitsInBuffer = 0;
    int byteIndex = 0;

    for (int i = 0; i < static_cast<int>(CRSF_ID::KFS_MAX_CHANNEL); i++) {
        local_data.channels[i] = payload[i];
    }

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        kfs_data_ = local_data; // 内存拷贝，极快
    }

    // 打印解析出的各个通道原始值和转换后的PWM值
    // Serial.println("CRSF Channels Received:");
    // for (int i = 0; i < CRSF_MAX_CHANNEL; i++) {
    //     Serial.print("CH");
    //     Serial.print(i + 1);
    //     Serial.print(": ");
    //     Serial.print(rcChannels[i]); // 原始CRSF值 (172-1811)
    //     Serial.print(" (PWM: ");
    //     Serial.print(convertCrsfToPwm(rcChannels[i]));
    //     Serial.println("us)");
    // }
    }




    // PACKED_STRUCT(ELRSData {
    //     uint64_t joystick_channel0 : 11;
    //     uint64_t joystick_channel1 : 11;
    //     uint64_t joystick_channel2 : 11;
    //     uint64_t joystick_channel3 : 11;

    //     uint64_t switch_right : 11;
    //     uint64_t switch_left : 11;

    //     uint64_t key_1 : 11;
    //     uint64_t key_2 : 11;
    //     uint64_t key_3 : 11;
    //     uint64_t key_4 : 11;
    //     uint64_t key_5 : 11;
    //     uint64_t key_6 : 11;
    //     uint64_t key_7 : 11;
    //     uint64_t key_8 : 11;
    //     uint64_t key_9 : 11;
    //     uint64_t key_10 : 11;
    //     // std::array<uint16_t, 16> channels;
    // });
    // static_assert(sizeof(ELRSData) == 22);

    // std::atomic<uint64_t> elrs_part1_{0};    //前44位
    // std::atomic<uint32_t> elrs_part2_{0};    //前22位
    // std::atomic<uint64_t> elrs_part3_{0};    //前55位
    // std::atomic<uint64_t> elrs_part4_{0};    //前55位

    std::mutex data_mutex_; // 新增一把锁
    ELRSData current_data_; // 受保护的数据
    KFSData kfs_data_; // 受保护的数据



    const unsigned char crc8tab[256] = {
  0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54,
  0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
  0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06,
  0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
  0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0,
  0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
  0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2,
  0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
  0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9,
  0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
  0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B,
  0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
  0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D,
  0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
  0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F,
  0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
  0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB,
  0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
  0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9,
  0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
  0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F,
  0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
  0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D,
  0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
  0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26,
  0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
  0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74,
  0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
  0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82,
  0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
  0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0,
  0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
    };

    uint8_t crc8(volatile uint8_t *data, uint32_t len)
    {
    uint8_t crc = 0;
    while (len--)
    {
        crc = crc8tab[crc ^ *data++];
    }
    return crc;
    }

    Vector joystick_right_ = Vector::zero();
    Vector joystick_left_ = Vector::zero();

    Switch switch_right_ = Switch::UNKNOWN;
    Switch switch_left_ = Switch::UNKNOWN;

    Key key = Key::zero();  
};
}