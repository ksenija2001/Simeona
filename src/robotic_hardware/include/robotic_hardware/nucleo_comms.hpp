#ifndef ROBOTIC_HARDWARE_NUCLEO_COMMS_H
#define ROBOTIC_HARDWARE_NUCLEO_COMMS_H

#include <libserial/SerialPort.h>
#include <cstring>

union floatUnion
{
    float f;
    uint8_t u[sizeof(float)];
};

class NucleoComms
{
public:
    NucleoComms();
    NucleoComms(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);

    void setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);
    void floatToBytes(LibSerial::DataBuffer msg, int start, float data);
    void bytesToFloat(float *msg, LibSerial::DataBuffer data, int start);

    bool connected() const { return serial_conn_.IsOpen(); }

    bool sendMsg(const float msg_to_send[], uint8_t code, int len);
    void readMsg(float received[], uint8_t code, int len);

private:
    LibSerial::SerialPort serial_conn_;
    LibSerial::DataBuffer buffer_;
    bool ack_;
    floatUnion converter_;
    int timeout_ms;
};

#endif // ROBOTIC_HARDWARE_NUCLEO_COMMS_H