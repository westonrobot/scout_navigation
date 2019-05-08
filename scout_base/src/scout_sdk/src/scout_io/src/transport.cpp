#include <iostream>
#include <thread>
#include <mutex>
#include <cstring>

#include "scout_io/serialport.h"
#include "scout_io/transport.h"

#define serial_debug 1
#define receive_speed_callback_debug 0
#define receive_odom_debug 1

namespace
{
#define ChassisToRos_Odom 0x56
#define RosToChassis_Cmd 0x12

bool Is_send_speed_callback;

scout_transport::Frame_t Get_Odom_Frame;
scout_transport::Cmd_t cmd;

std::mutex ack_write;
std::mutex cmd_write;
} // namespace

namespace scout_transport
{
unsigned short Checksum(unsigned char *data, unsigned short len)
{
    unsigned short i = 0;
    unsigned short sum = 0;
    for (i = 0; i < len; i++)
    {
        sum += data[i];
    }
    return sum;
}

void Send_Speed(short Angular, short Linear, unsigned char Count)
{
    Frame_t Cmd_Frame;
    memset(&Cmd_Frame, 0, sizeof(Cmd_Frame));
    Cmd_Frame.Header = 0x55aa;
    Cmd_Frame.Len = sizeof(Cmd_Frame) - 2;
    Cmd_Frame.Typedef = RosToChassis_Cmd;
    Cmd_Frame.Count = Count;
    Cmd_Frame.Angular = Angular;
    Cmd_Frame.Linear = Linear;
    Cmd_Frame.CheckSum = Checksum((unsigned char *)&Cmd_Frame, Cmd_Frame.Len);
    scout_serial::Write((unsigned char *)&Cmd_Frame, sizeof(Cmd_Frame));
}

void Read_DataOfChassis(void)
{
    static int state = 0;
    static int len_index = 0;
    static unsigned char len = 0;
    static unsigned char buff[30] = {0};

    switch (state)
    {
    case 0:
    {
        scout_serial::Read(&buff[0], 1);
        if (buff[0] == 0xaa)
        {
            len_index = 0;
            state = 1;
        }
        else
        {
            printf("Head error 1\r\n");
            state = 0;
        }
        break;
    }
    case 1:
    {
        scout_serial::Read(&buff[1], 1);
        if (buff[1] == 0x55)
        {
            len_index = 0;
            state = 2;
        }
        else
        {
            printf("Head error 2\r\n");
            state = 0;
        }
        break;
    }
    case 2:
    {
        scout_serial::Read(&buff[2], 1);
        len = buff[2];
        if (len == 10)
        {
            state = 3;
        }
        else
        {
            state = 0;
            printf("Len error 1 : %d \r\n", buff[2]);
        }
        break;
    }
    case 3:
    {
        scout_serial::Read(&buff[3], len - 1);
        unsigned short CheckSum = buff[len] | buff[len + 1] << 8;
        if (CheckSum == Checksum(&buff[0], len))
        {
            state = 4;
        }
        else
        {
            state = 0;
            printf("Check sum error \r\n");
        }
        break;
    }
    case 4:
    {
        unsigned char Typedef = buff[3];
        if (Typedef == ChassisToRos_Odom)
        {
            memcpy(&Get_Odom_Frame, &buff[0], sizeof(Get_Odom_Frame));

            cmd.IsUpdata = true;
            cmd.Angular = Get_Odom_Frame.Angular;
            cmd.Linear = Get_Odom_Frame.Linear;

            static int Successful_Connection = 0;

            if (Successful_Connection == 0)
            {
                Successful_Connection = 1;
                printf("The connection was successful and the vehicle was ready!");
            }

#if receive_odom_debug
            printf("Angular = %d  Linear = %d count = %d Time_Out = %d \r\n",
                   Get_Odom_Frame.Angular,
                   Get_Odom_Frame.Linear,
                   Get_Odom_Frame.Count,
                   Get_Odom_Frame.Time_Out);
#endif
        }
        state = 0;
        break;
    }
    default:
    {
        state = 0;
        break;
    }
    }
}

Cmd_t Get_dataOfTransport() { return cmd; }

void Set_dataOfTransport(Cmd_t *CMD) { cmd = *CMD; }

void Read_DataOfChassis_Loop(void)
{
    unsigned int len = scout_serial::GetDataCount();
    if (len >= sizeof(Frame_t))
    {
        Read_DataOfChassis();
    }
}

void Send_SpeedToChassis(short Angular, short Linear, unsigned char Count)
{
    Is_send_speed_callback = false;
    static int time = 0;
    time = 0;
    do
    {
        Send_Speed(Angular, Linear, Count);
        // boost::this_thread::sleep(boost::posix_time::milliseconds(60));
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
        time++;
        if (time > 20)
        {
            // ROS_INFO_STREAM("lost scout! Please check whether the power supply or the serial line of the vehicle is loose.\r\n");
            std::cout << "Lost Scout! Please check whether the power supply or the serial cable of the vehicle is loose." << std::endl;
            break;
        }
    } while (Is_send_speed_callback == false);
}
} // namespace scout_transport
