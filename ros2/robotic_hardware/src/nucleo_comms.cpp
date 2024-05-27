#include "include/robotic_hardware/nucleo_comms.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <sstream>
#include <cstdlib>
#include <mutex>
#include <unistd.h>

#include "include/robotic_hardware/definitions.hpp"

namespace robotic_hardware
{
    void NucleoComms::setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
    {
        serial_conn_.setPort(serial_device);
        serial_conn_.setBaudrate(baud_rate);
        serial::Timeout tt = serial::Timeout::simpleTimeout(timeout_ms);
        serial_conn_.setTimeout(tt); // This should be inline except setTimeout takes a reference and so needs a variable
        serial_conn_.open();
    }

    void NucleoComms::readMsg(float receive[], uint8_t code, int len)
    {
        if (serial_conn_.read(buffer_, len) == len && buffer_[1] == code)
        {
            for (int i = 0; i < (len - 4) / 4; i++)
            {
                bytesToFloat(&receive[i], buffer_, 4 * i + 3);
            }
        }
        else
        {
            RCLCPP_ERROR_STREAM(logger_, "Message was not received in time");
        }
    }

    bool NucleoComms::sendMsg(const float msg_to_send[], uint8_t code, int len)
    {
        int length = len * 4 + 4;
        uint8_t msg[length];
        msg[0] = START;
        msg[1] = code;
        msg[2] = length - 1;

        for (int i = 0; i < len; i++)
        {
            floatToBytes(&msg, 4 * i + 3, msg_to_send[i]);
        }

        msg[length - 1] = STOP;

        serial_con_.flushOutput();
        serial_conn_.write(msg, length);

        if (serial_conn_.read(buffer_, ACK_LEN) != ACK_LEN)
        {
            RCLCPP_ERROR_STREAM(logger_, "ACK was not received in time");
            return false;
        }

        RCLCPP_INFO_STREAM(logger_, "Sent: " << msg_to_send);

        return true;
    }

    void NucleoComms::floatToBytes(uint8_t *msg, int start, float data)
    {
        converter_.f = data;
        msg[start] = converter_.u[0];
        msg[start + 1] = converter_.u[1];
        msg[start + 2] = converter_.u[2];
        msg[start + 3] = converter_.u[3];
    }

    void NucleoComms::bytesToFloat(float *msg, uint8_t data[], int start)
    {
        converter_.u[0] = data[start];
        converter_.u[1] = data[start + 1];
        converter_.u[2] = data[start + 2];
        converter_.u[3] = data[start + 3];

        msg = convert.f;
    }

}