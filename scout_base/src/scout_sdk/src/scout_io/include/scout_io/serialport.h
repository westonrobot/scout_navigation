#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <string>

namespace scout_serial
{
int Open_Serial(std::string port_name, int baud_rate);
unsigned int Write(unsigned char *data, unsigned short len);
unsigned int Read(unsigned char *data, unsigned short len);
unsigned int GetDataCount(void);
} // namespace scout_serial

#endif /* SERIALPORT_H */
