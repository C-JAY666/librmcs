#pragma once


#include <HardwareSerial.h>

// 定义CRSF协议相关常量 (与发送部分共用)
#define CRSF_SYNC_BYTE 0xC8
#define CRSF_MAX_CHANNEL 16
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16
#define CRSF_MIN_FRAME_SIZE 4 // Sync + Length + Type + CRC (最短帧，如ping)
#define CRSF_MAX_FRAME_SIZE 64 // CRSF帧最大长度限制 (包括扩展帧)

// 假设使用Serial2作为CRSF通信串口
HardwareSerial &crsfSerial = Serial2;

// 用于存储解码后的16个通道原始CRSF值 (172-1811)
uint16_t rcChannels[CRSF_MAX_CHANNEL];

// 转换CRSF原始通道值 (172-1811) 到 PWM值 (例如 988-2012us)
uint16_t convertCrsfToPwm(uint16_t crsf_val) {
    // 线性映射: (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    // CRSF: 172 (min) -> 992 (mid) -> 1811 (max)
    // PWM:  988 (min) -> 1500 (mid) -> 2012 (max)
    if (crsf_val <= 172) return 988;
    if (crsf_val >= 1811) return 2012;
    return (uint16_t)(988.0f + (crsf_val - 172.0f) * (2012.0f - 988.0f) / (1811.0f - 172.0f));
}


// CRC校验函数 (与发送部分共用)
uint8_t calcCRC(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0xD5;
            else crc = crc << 1;
        }
    }
    return crc;
}

// 处理接收到的CRSF RC Channels Packed 数据包
void processCRSFRcChannelsPacket(const uint8_t *payload) {
    uint64_t bitBuffer = 0;
    uint8_t bitsInBuffer = 0;
    int byteIndex = 0;

    for (int i = 0; i < CRSF_MAX_CHANNEL; i++) {
        while (bitsInBuffer < 11) {
            // 从payload中读取字节
            bitBuffer |= ((uint64_t)payload[byteIndex++] << bitsInBuffer);
            bitsInBuffer += 8;
        }
        rcChannels[i] = bitBuffer & 0x07FF; // 取低11位作为通道值
        bitBuffer >>= 11;
        bitsInBuffer -= 11;
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

// 从串口接收并解析CRSF数据包
void receiveCRSF() {
    static uint8_t rxBuffer[CRSF_MAX_FRAME_SIZE];
    static uint8_t rxPos = 0;
    static uint8_t expectedLength = 0;

    while (crsfSerial.available()) {
        uint8_t byteReceived = crsfSerial.read();

        if (rxPos == 0) { // 等待同步字节
            if (byteReceived == CRSF_SYNC_BYTE) {
                rxBuffer[rxPos++] = byteReceived;
            }
        } else if (rxPos == 1) { // 接收长度字节
            if (byteReceived >= (CRSF_MIN_FRAME_SIZE - 2) && byteReceived <= (CRSF_MAX_FRAME_SIZE - 2) ) { // 长度字节范围校验 (Type+Payload+CRC)
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
                uint8_t calculatedCRC = calcCRC(&rxBuffer[2], expectedLength - 3);
                if (calculatedCRC == rxBuffer[expectedLength - 1]) {
                    // CRC校验成功，根据类型处理数据包  
                    uint8_t frameType = rxBuffer[2];
                    const uint8_t *payload = &rxBuffer[3]; // 负载数据从第3个字节开始
                    // uint8_t payloadLength = expectedLength - 4; // Sync, Length, Type, CRC

                    if (frameType == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
                        processCRSFRcChannelsPacket(payload);
                    } else {
                        // 在此处理其他类型的CRSF帧
                        // Serial.print("Received CRSF Frame Type: 0x");
                        // Serial.println(frameType, HEX);
                    }
                } else {
                    // Serial.println("CRC Error!");
                }
                rxPos = 0; // 重置缓冲区，准备接收下一帧
            } else if (rxPos >= CRSF_MAX_FRAME_SIZE) { // 缓冲区溢出，通常不应发生
                rxPos = 0;
            }
        }
    }
}

void setup() {
    // 初始化调试串口
    Serial.begin(115200); // 用于打印调试信息
    while(!Serial); // 等待串口连接 (某些板子需要)
    Serial.println("CRSF Parser Initializing...");

    // 初始化CRSF串口
    initCRSFSerial();
    Serial.println("CRSF Serial Initialized at 420000bps.");
}

void loop() {
    receiveCRSF(); // 循环接收并解析CRSF数据包

    // 可以在这里使用解析到的 rcChannels[] 数据
    // 例如，每秒打印一次通道1的值
    // static unsigned long lastPrintTime = 0;
    // if (millis() - lastPrintTime > 1000) {
    //     lastPrintTime = millis();
    //     Serial.print("CH1 (Raw): ");
    //     Serial.print(rcChannels[0]);
    //     Serial.print(" (PWM): ");
    //     Serial.println(convertCrsfToPwm(rcChannels[0]));
    // }
}
