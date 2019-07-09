/* 
 * scout_serial.h
 * 
 * Created on: May 05, 2019 11:35
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SCOUT_SERIAL_H
#define SCOUT_SERIAL_H

#include "scout_io/scout_io.h"

namespace scout
{
struct Frame_t
{
    unsigned short Header;
    unsigned char Len;
    unsigned char Typedef;
    unsigned char Count;
    unsigned char Time_Out;
    short Linear;
    short Angular;
    unsigned short CheckSum;
};

struct Cmd_t
{
    unsigned short Linear;
    unsigned short Angular;
    bool IsUpdata;
};

typedef enum
{
    eHead = 0,
    eLen = 1,
    eTypedef = 3,
    eChecksum = 2
} state_t;

class ScoutSerial : public ScoutIO
{
public:
    Cmd_t Get_dataOfTransport();
    void Set_dataOfTransport(Cmd_t *CMD);
    void Read_DataOfChassis_Loop(void);
    void Send_SpeedToChassis(short Angular, short Linear, unsigned char Count);
    void Send_Speed(short Angular, short Linear, unsigned char Count);
    unsigned short Checksum(unsigned char *data, unsigned short len);
    void Find_NextHead();
};

Cmd_t Get_dataOfTransport();
void Set_dataOfTransport(Cmd_t *CMD);
void Read_DataOfChassis_Loop(void);
void Send_SpeedToChassis(short Angular, short Linear, unsigned char Count);
void Send_Speed(short Angular, short Linear, unsigned char Count);
unsigned short Checksum(unsigned char *data, unsigned short len);
void Find_NextHead();
void Send_SpeedToChassis(short Angular, short Linear, unsigned char Count);
} // namespace scout

#endif /* SCOUT_SERIAL_H */
