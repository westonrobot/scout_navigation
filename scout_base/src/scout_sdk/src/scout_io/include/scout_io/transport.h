#ifndef TRANSPORT_H
#define TRANSPORT_H

namespace scout_transport
{
typedef struct
{
    unsigned short Header;
    unsigned char Len;
    unsigned char Typedef;
    unsigned char Count;
    unsigned char Time_Out;
    short Linear;
    short Angular;
    unsigned short CheckSum;
} Frame_t;

typedef struct
{
    unsigned short Linear;
    unsigned short Angular;
    bool IsUpdata;
} Cmd_t;

typedef enum
{
    eHead = 0,
    eLen = 1,
    eTypedef = 3,
    eChecksum = 2
} state_t;

Cmd_t Get_dataOfTransport();
void Set_dataOfTransport(Cmd_t *CMD);
void Read_DataOfChassis_Loop(void);
void Send_SpeedToChassis(short Angular, short Linear, unsigned char Count);
void Send_Speed(short Angular, short Linear, unsigned char Count);
unsigned short Checksum(unsigned char *data, unsigned short len);
void Find_NextHead();
void Send_SpeedToChassis(short Angular, short Linear, unsigned char Count);
} // namespace scout_transport

#endif /* TRANSPORT_H */
