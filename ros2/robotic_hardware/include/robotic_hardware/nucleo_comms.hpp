#ifndef ROBOTIC_HARDWARE_NUCLEO_COMMS_H
#define ROBOTIC_HARDWARE_NUCLEO_COMMS_H

#include <serial/serial.h>
#include <cstring>

namespace robotic_hardware
{
    union floatUnion
    {
        float f;
        uint8_t u[sizeof(float)];
    }

    class NucleoComms
    {
    public:
        NucleoComms();
        NucleoComms(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
            : serial_conn_(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms)) {}

        void setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);
        void floatToBytes(uint8_t *msg, int start, float data);
        void bytesToFloat(float *msg, uint8_t data[4]);

        bool connected() const { return serial_conn_.isOpen(); }

        bool sendMsg(const float msg_to_send[], uint8_t code, int len);
        *float NucleoComms::readMsg(uint8_t code, int len);

    private:
        serial::Serial serial_conn_;
        uint8_t buffer_[30];
        bool ack_;
        floatUnion converter_;
    }
}
#endif // ROBOTIC_HARDWARE_NUCLEO_COMMS_H