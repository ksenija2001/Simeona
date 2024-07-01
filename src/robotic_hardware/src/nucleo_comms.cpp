#include "robotic_hardware/nucleo_comms.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <sstream>
#include <cstdlib>
#include <mutex>
#include <unistd.h>

#include "robotic_hardware/definitions.hpp"

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
    // Just handle some common baud rates
    switch (baud_rate)
    {
    case 1200:
        return LibSerial::BaudRate::BAUD_1200;
    case 1800:
        return LibSerial::BaudRate::BAUD_1800;
    case 2400:
        return LibSerial::BaudRate::BAUD_2400;
    case 4800:
        return LibSerial::BaudRate::BAUD_4800;
    case 9600:
        return LibSerial::BaudRate::BAUD_9600;
    case 19200:
        return LibSerial::BaudRate::BAUD_19200;
    case 38400:
        return LibSerial::BaudRate::BAUD_38400;
    case 57600:
        return LibSerial::BaudRate::BAUD_57600;
    case 115200:
        return LibSerial::BaudRate::BAUD_115200;
    case 230400:
        return LibSerial::BaudRate::BAUD_230400;
    default:
        std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
        return LibSerial::BaudRate::BAUD_57600;
    }
}

void NucleoComms::setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{
    timeout_ms = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
}

void NucleoComms::readMsg(float receive[], uint8_t code, int len)
{
    // TODO napraviti sve sa readbyte kao sto je i u python codu
    serial_conn_.Read(buffer_, len, timeout_ms);
    if (buffer_[1] == code && buffer_[2] == len - 1)
    {
        for (int i = 0; i < (len - 4) / 4; i++)
        {
            bytesToFloat(&receive[i], buffer_, 4 * i + 3);
        }
    }
    else
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Message was not received in time");
    }
}

bool NucleoComms::sendMsg(const float msg_to_send[], uint8_t code, int len)
{
    int length = len * 4 + 4;
    LibSerial::DataBuffer msg;
    msg[0] = START;
    msg[1] = code;
    msg[2] = length - 1;

    for (int i = 0; i < len; i++)
    {
        floatToBytes(msg, 4 * i + 3, msg_to_send[i]);
    }

    msg[length - 1] = STOP;

    serial_conn_.FlushOutputBuffer();
    serial_conn_.Write(msg);

    serial_conn_.Read(buffer_, ACK_LEN, timeout_ms);
    if (buffer_[2] != ACK_LEN - 1)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "ACK was not received in time");
        return false;
    }

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Sent: " << msg_to_send);

    return true;
}

void NucleoComms::floatToBytes(LibSerial::DataBuffer msg, int start, float data)
{
    converter_.f = data;
    msg[start] = converter_.u[0];
    msg[start + 1] = converter_.u[1];
    msg[start + 2] = converter_.u[2];
    msg[start + 3] = converter_.u[3];
}

void NucleoComms::bytesToFloat(float *msg, LibSerial::DataBuffer data, int start)
{
    converter_.u[0] = data[start];
    converter_.u[1] = data[start + 1];
    converter_.u[2] = data[start + 2];
    converter_.u[3] = data[start + 3];

    *msg = converter_.f;
}
