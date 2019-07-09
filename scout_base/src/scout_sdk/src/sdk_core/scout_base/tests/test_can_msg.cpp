#include <iostream>

#include "scout/scout_can_protocol.h"

void print_msg(uint8_t data[8])
{
    for (int i = 0; i < 8; ++i)
        std::cout << std::hex << static_cast<int>(data[i]) << " ";
    std::cout << std::dec << std::endl;
}

int main()
{
    MotionControlMessage msg;
    msg.data.cmd.control_mode = CMD_CAN_MODE;
    msg.data.cmd.fault_clear_flag = NO_FAULT;
    msg.data.cmd.linear_velocity_cmd = 10;
    msg.data.cmd.angular_velocity_cmd = 0;
    msg.data.cmd.reserved0 = 0;
    msg.data.cmd.reserved1 = 0;
    msg.data.cmd.count = 0;
    msg.data.cmd.checksum = Agilex_CANMsgChecksum(msg.id, msg.data.raw, msg.dlc);
    print_msg(msg.data.raw);

    LightControlMessage msg2;
    msg2.data.cmd.light_ctrl_enable = DISABLE_LIGHT_CTRL;
    msg2.data.cmd.front_light_mode = CONST_ON;
    msg2.data.cmd.front_light_custom = 0;
    msg2.data.cmd.rear_light_mode = CONST_ON;
    msg2.data.cmd.rear_light_custom = 0;
    msg2.data.cmd.reserved0 = 0;
    msg2.data.cmd.count = 0;
    msg2.data.cmd.checksum = Agilex_CANMsgChecksum(msg2.id, msg2.data.raw, msg2.dlc);
    print_msg(msg2.data.raw);

    return 0;
}