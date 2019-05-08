#include "scout/scout_robot.h"

#include <string>
#include <iostream>

#include "scout_io/serialport.h"
#include "scout_io/transport.h"

namespace scout
{
void ScoutRobot::ConnectSerialPort(const std::string &port_name, int32_t baud_rate)
{
    serial_connected_ = (scout_serial::Open_Serial(port_name, baud_rate) > 0) ? true : false;
}

bool ScoutRobot::QueryRobotState(RobotState *data)
{
    scout_transport::Cmd_t cmd;
    scout_transport::Read_DataOfChassis_Loop();
    cmd = scout_transport::Get_dataOfTransport();

    if (cmd.IsUpdata == true)
    {
        cmd.IsUpdata = false;
        data->linear = cmd.Linear;
        data->angular = cmd.Angular;
        scout_transport::Set_dataOfTransport(&cmd);

        return true;
    }

    return false;
}

void ScoutRobot::SendCommand(const RobotCmd& cmd)
{
    double cent_speed = cmd.linear;
    double cmd_twist_rotation = cmd.angular;

    cent_speed = cent_speed * 10000;
    cmd_twist_rotation = cmd_twist_rotation * 10000;
    if (cent_speed > 20000)
        cent_speed = 20000;
    if (cent_speed < -20000)
        cent_speed = -20000;
    if (cmd_twist_rotation > 20000)
        cmd_twist_rotation = 20000;
    if (cmd_twist_rotation < -20000)
        cmd_twist_rotation = -20000;

    scout_transport::Send_Speed(static_cast<short>(cmd_twist_rotation), static_cast<short>(cent_speed), cmd.count);
    std::cout << "send -> linear: " << cent_speed << "; angular: " << cmd_twist_rotation << std::endl;
}
} // namespace scout
