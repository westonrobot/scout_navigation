#include <iostream>

#include <serial/serial.h>

#include "scout_io/serialport.h"

namespace
{
serial::Serial ser;
}

namespace scout_serial
{
int Open_Serial(std::string port_name, int baud_rate)
{
    try
    {
        // 设置串口属性，并打开串口 Configure and open serial port
        ser.setPort(port_name);
        ser.setBaudrate(baud_rate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        std::cerr << "Unable to open port " << std::endl;
        return -1;
    }

    if (ser.isOpen())
    {
        std::cerr << "Serial Port initialized" << std::endl;
        return 1;
    }
    else
    {
        return -1;
    }
}

unsigned int Write(unsigned char *data, unsigned short len)
{
    unsigned int Len = ser.write(data, len);
    return Len;
}

unsigned int Read(unsigned char *data, unsigned short len)
{
    unsigned int Len = ser.read(data, len);
    return Len;
}

unsigned int GetDataCount(void)
{
    return ser.available();
}
} // namespace scout_serial
