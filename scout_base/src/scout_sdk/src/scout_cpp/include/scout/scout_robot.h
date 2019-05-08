#ifndef SCOUT_ROBOT_H
#define SCOUT_ROBOT_H

#include <string>
#include <cstdint>

namespace scout
{
struct RobotState
{
    RobotState() : linear(0), angular(0) {}
    RobotState(short _linear, short _angular) : linear(_linear), angular(_angular) {}

    short linear;
    short angular;
};

struct RobotCmd
{
    RobotCmd() : linear(0), angular(0) {}
    RobotCmd(double _linear, double _angular, uint32_t cnt)
        : linear(_linear), angular(_angular), count(0) {}

    double linear;
    double angular;
    uint32_t count;
};

class ScoutRobot
{
public:
    void ConnectSerialPort(const std::string &port_name = "/dev/ttyUSB0", int32_t baud_rate = 115200);
    bool IsConnectionActive() const { return serial_connected_; }

    void SendCommand(const RobotCmd& cmd);
    bool QueryRobotState(RobotState *data);

private:
    bool serial_connected_ = false;
};
} // namespace scout

#endif /* SCOUT_ROBOT_H */
