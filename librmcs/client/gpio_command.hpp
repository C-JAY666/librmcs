#pragma once


#include <cstdint>



namespace librmcs::client {

class GpioCommand { 
public:

    enum class Command_Type : uint8_t{
        SET_SINGLE_GPIO = 0x01,
        SET_MULTIPLE_GPIO = 0x02,
        READ_GPIO = 0x03,
        TOGGLE_GPIO = 0x04,
        PWM_CONTROL = 0x05
    };

    enum class GPIO_PORT : uint8_t{
        Port_A = 0x00,
        Port_B = 0x01,
        Port_C = 0x02,
        Port_D = 0x03,
        Port_E = 0x04
    };

    enum class GPIO_PIN : uint8_t{
        Pin_0 = 0, Pin_1 = 1, Pin_2= 2, Pin_3 = 3,
        Pin_4 = 4, Pin_5 = 5, Pin_6 = 6, Pin_7 = 7,
        Pin_8 = 8, Pin_9 = 9, Pin_10 = 10, Pin_11 = 11,
        Pin_12 = 12, Pin_13 = 13, Pin_14 = 14, Pin_15 = 15
    };
    
    enum class GPIO_STATE : uint8_t{
        LOW = 0x00,
        HIGH = 0x01,
        TOGGLE = 0x02
    };

    struct Gpio_Command {
        Command_Type cmd_type;
        GPIO_PORT port;
        GPIO_PIN  pin;
        GPIO_STATE state;
        uint16_t pwm_value;
        uint8_t reserved[2];
    };

    uint64_t set_gpio( GPIO_PORT port, GPIO_PIN pin, GPIO_STATE state) {
         return set_can( Command_Type::SET_SINGLE_GPIO, port, pin, state, 0);    
    };
     
    uint64_t set_multiple( GPIO_PORT port, GPIO_PIN pin, uint16_t value) {
        return set_can( Command_Type::SET_MULTIPLE_GPIO, port, pin, GPIO_STATE::LOW, value);
    };

    uint64_t set_toggle( GPIO_PORT port, GPIO_PIN pin) {
        return set_can( Command_Type::TOGGLE_GPIO, port, pin, GPIO_STATE::TOGGLE, 0);
    };

    uint64_t set_pwm( GPIO_PORT port, GPIO_PIN pin, uint16_t value) {
        return set_can( Command_Type::PWM_CONTROL, port, pin, GPIO_STATE::LOW, value);
    };

    uint64_t set_4pwm( uint16_t pwm1, uint16_t pwm2, uint16_t pwm3, uint16_t pwm4) {
        return pack_4_pwm(pwm1,  pwm2,  pwm3,  pwm4);
    };

    
private:
    uint64_t set_can( Command_Type cmd, GPIO_PORT port,GPIO_PIN pin, GPIO_STATE state, uint16_t value ){
        uint64_t can_data = 
            static_cast<uint64_t>(cmd)  |
            (static_cast<uint64_t>(port) << 8) |
            (static_cast<uint64_t>(pin) << 16) |
            (static_cast<uint64_t>(state) << 24) |
            (static_cast<uint64_t>(value & 0xFF) << 32) |
            (static_cast<uint64_t>((value >> 8) & 0xFF) << 40) ;
        
        return can_data;
    };
    // 在 GpioCommand 类中添加
    uint64_t pack_4_pwm(uint16_t pwm1, uint16_t pwm2, uint16_t pwm3, uint16_t pwm4) {
        uint64_t data = 0;
        data |= static_cast<uint64_t>(pwm1);         // Byte 0-1
        data |= static_cast<uint64_t>(pwm2) << 16;   // Byte 2-3
        data |= static_cast<uint64_t>(pwm3) << 32;   // Byte 4-5
        data |= static_cast<uint64_t>(pwm4) << 48;   // Byte 6-7
        return data;
    }

};
}