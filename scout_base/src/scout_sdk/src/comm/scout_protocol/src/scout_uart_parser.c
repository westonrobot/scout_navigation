/* 
 * scout_uart_parser.c
 * 
 * Created on: Aug 14, 2019 12:02
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "scout_protocol/scout_uart_parser.h"

// #define PRINT_CPP_DEBUG_INFO
// #define PRINT_JLINK_DEBUG_INFO
// #define USE_XOR_CHECKSUM

#ifdef PRINT_CPP_DEBUG_INFO
#undef PRINT_JLINK_DEBUG_INFO
#endif

#ifdef PRINT_CPP_DEBUG_INFO
#define < iostream >
#elif (defined(PRINT_JLINK_DEBUG_INFO))
#include "segger/jlink_rtt.h"
#endif

typedef enum
{
    WAIT_FOR_SOF1 = 0,
    WAIT_FOR_SOF2,
    WAIT_FOR_FRAME_LEN,
    WAIT_FOR_FRAME_TYPE,
    WAIT_FOR_FRAME_ID,
    WAIT_FOR_PAYLOAD,
    WAIT_FOR_FRAME_COUNT,
    WAIT_FOR_CHECKSUM
} ScoutSerialDecodeState;

#define PAYLOAD_BUFFER_SIZE (SCOUT_FRAME_SIZE * 2)

#define FRAME_SOF_LEN ((uint8_t)2)
#define FRAME_FIXED_FIELD_LEN ((uint8_t)4)

#define FRAME_SOF1 ((uint8_t)0x5a)
#define FRAME_SOF2 ((uint8_t)0xa5)

#define FRAME_TYPE_CONTROL ((uint8_t)0x55)
#define FRAME_TYPE_STATUS ((uint8_t)0xaa)

#define FRAME_NONE_ID ((uint8_t)0x00)

typedef union {
    ScoutStatusMessage status_msg;
    ScoutControlMessage control_msg;
} ScoutDecodedMessage;

// frame buffer
static uint8_t frame_id = 0;
static uint8_t frame_type = 0;
static uint8_t frame_len = 0;
static uint8_t frame_cnt = 0;
static uint8_t frame_checksum = 0;
static uint8_t internal_checksum = 0;
static uint8_t payload_buffer[PAYLOAD_BUFFER_SIZE];
static size_t payload_data_pos = 0;

// statisctics
static uint32_t frame_parsed = 0;
static uint32_t frame_with_wrong_checksum = 0;

static bool ParseChar(uint8_t c, ScoutDecodedMessage *msg);
static uint8_t CalcBufferedFrameChecksum();
static bool ConstructStatusMessage(ScoutStatusMessage *msg);
static bool ConstructControlMessage(ScoutControlMessage *msg);

void EncodeScoutStatusMsgToUART(const ScoutStatusMessage *msg, uint8_t *buf, uint8_t *len)
{
    // SOF
    buf[0] = FRAME_SOF1;
    buf[1] = FRAME_SOF2;

    // frame len, type, ID
    buf[2] = 0x0a;
    buf[3] = FRAME_TYPE_STATUS;

    switch (msg->msg_type)
    {
    // in the current implementation, both MsgType and can_frame include 8 * uint8_t
    case ScoutMotionStatusMsg:
    {
        buf[4] = UART_FRAME_MOTION_STATUS_ID;
        buf[5] = msg->motion_status_msg.data.status.linear_velocity.high_byte;
        buf[6] = msg->motion_status_msg.data.status.linear_velocity.low_byte;
        buf[7] = msg->motion_status_msg.data.status.angular_velocity.high_byte;
        buf[8] = msg->motion_status_msg.data.status.angular_velocity.low_byte;
        buf[9] = 0;
        buf[10] = 0;
        buf[11] = msg->motion_status_msg.data.status.count;
        break;
    }
    case ScoutLightStatusMsg:
    {
        buf[4] = UART_FRAME_LIGHT_STATUS_ID;
        buf[5] = msg->light_status_msg.data.status.light_ctrl_enable;
        buf[6] = msg->light_status_msg.data.status.front_light_mode;
        buf[7] = msg->light_status_msg.data.status.front_light_custom;
        buf[8] = msg->light_status_msg.data.status.rear_light_mode;
        buf[9] = msg->light_status_msg.data.status.rear_light_custom;
        buf[10] = 0;
        buf[11] = msg->light_status_msg.data.status.count;
        break;
    }
    case ScoutSystemStatusMsg:
    {
        buf[4] = UART_FRAME_SYSTEM_STATUS_ID;
        buf[5] = msg->system_status_msg.data.status.base_state;
        buf[6] = msg->system_status_msg.data.status.control_mode;
        buf[7] = msg->system_status_msg.data.status.battery_voltage.high_byte;
        buf[8] = msg->system_status_msg.data.status.battery_voltage.low_byte;
        buf[9] = msg->system_status_msg.data.status.fault_code.high_byte;
        buf[10] = msg->system_status_msg.data.status.fault_code.low_byte;
        buf[11] = msg->system_status_msg.data.status.count;
        break;
    }
    case ScoutMotorDriverStatusMsg:
    {
        if (msg->motor_driver_status_msg.motor_id == SCOUT_MOTOR1_ID)
            buf[4] = UART_FRAME_MOTOR1_DRIVER_STATUS_ID;
        else if (msg->motor_driver_status_msg.motor_id == SCOUT_MOTOR2_ID)
            buf[4] = UART_FRAME_MOTOR2_DRIVER_STATUS_ID;
        else if (msg->motor_driver_status_msg.motor_id == SCOUT_MOTOR3_ID)
            buf[4] = UART_FRAME_MOTOR3_DRIVER_STATUS_ID;
        else if (msg->motor_driver_status_msg.motor_id == SCOUT_MOTOR4_ID)
            buf[4] = UART_FRAME_MOTOR4_DRIVER_STATUS_ID;
        buf[5] = msg->motor_driver_status_msg.data.status.current.high_byte;
        buf[6] = msg->motor_driver_status_msg.data.status.current.low_byte;
        buf[7] = msg->motor_driver_status_msg.data.status.rpm.high_byte;
        buf[8] = msg->motor_driver_status_msg.data.status.rpm.low_byte;
        buf[9] = msg->motor_driver_status_msg.data.status.temperature;
        buf[10] = 0;
        buf[11] = msg->motor_driver_status_msg.data.status.count;
        break;
    }
    default:
        break;
    }

    buf[12] = CalcScoutUARTChecksum(buf, buf[2] + FRAME_SOF_LEN);

    // length: SOF + Frame + Checksum
    *len = buf[2] + FRAME_SOF_LEN + 1;
}

void EncodeScoutControlMsgToUART(const ScoutControlMessage *msg, uint8_t *buf, uint8_t *len)
{
    switch (msg->msg_type)
    {
    case ScoutMotionControlMsg:
    {
        EncodeMotionControlMsgToUART(&(msg->motion_control_msg), buf, len);
        break;
    }
    case ScoutLightControlMsg:
    {
        EncodeLightControlMsgToUART(&(msg->light_control_msg), buf, len);
        break;
    }
    default:
        break;
    }
}

void EncodeMotionControlMsgToUART(const MotionControlMessage *msg, uint8_t *buf, uint8_t *len)
{
    // SOF
    buf[0] = FRAME_SOF1;
    buf[1] = FRAME_SOF2;

    // frame len, type, ID
    buf[2] = 0x0a;
    buf[3] = FRAME_TYPE_CONTROL;
    buf[4] = UART_FRAME_MOTION_CONTROL_ID;

    // frame payload
    buf[5] = msg->data.cmd.control_mode;
    buf[6] = msg->data.cmd.fault_clear_flag;
    buf[7] = msg->data.cmd.linear_velocity_cmd;
    buf[8] = msg->data.cmd.angular_velocity_cmd;
    buf[9] = 0x00;
    buf[10] = 0x00;

    // frame count, checksum
    buf[11] = msg->data.cmd.count;
    buf[12] = CalcScoutUARTChecksum(buf, buf[2] + FRAME_SOF_LEN);

    // length: SOF + Frame + Checksum
    *len = buf[2] + FRAME_SOF_LEN + 1;
}

void EncodeLightControlMsgToUART(const LightControlMessage *msg, uint8_t *buf, uint8_t *len)
{
    // SOF
    buf[0] = FRAME_SOF1;
    buf[1] = FRAME_SOF2;

    // frame len, type, ID
    buf[2] = 0x0a;
    buf[3] = FRAME_TYPE_CONTROL;
    buf[4] = UART_FRAME_LIGHT_CONTROL_ID;

    // frame payload
    buf[5] = msg->data.cmd.light_ctrl_enable;
    buf[6] = msg->data.cmd.front_light_mode;
    buf[7] = msg->data.cmd.front_light_custom;
    buf[8] = msg->data.cmd.rear_light_mode;
    buf[9] = msg->data.cmd.rear_light_custom;
    buf[10] = 0x00;

    // frame count, checksum
    buf[11] = msg->data.cmd.count;
    buf[12] = CalcScoutUARTChecksum(buf, buf[2] + FRAME_SOF_LEN);

    // length: SOF + Frame + Checksum
    *len = buf[2] + FRAME_SOF_LEN + 1;
}

bool DecodeScoutStatusMsgFromUART(uint8_t c, ScoutStatusMessage *msg)
{
    static ScoutDecodedMessage decoded_msg;

    bool result = ParseChar(c, &decoded_msg);
    if (result)
        *msg = decoded_msg.status_msg;
    return result;
}

bool DecodeScoutControlMsgFromUART(uint8_t c, ScoutControlMessage *msg)
{
    static ScoutDecodedMessage decoded_msg;

    bool result = ParseChar(c, &decoded_msg);
    if (result)
        *msg = decoded_msg.control_msg;
    return result;
}

uint8_t CalcScoutUARTChecksum(uint8_t *buf, uint8_t len)
{
    uint8_t checksum = 0;

#ifdef USE_XOR_CHECKSUM
    for (int i = 0; i < len; ++i)
        checksum ^= buf[i];
#else
    for (int i = 0; i < len; ++i)
        checksum += buf[i];
#endif

    return checksum;
}

uint8_t CalcBufferedFrameChecksum()
{
    uint8_t checksum = 0x00;

#ifdef USE_XOR_CHECKSUM
    checksum ^= FRAME_SOF1;
    checksum ^= FRAME_SOF2;
    checksum ^= frame_len;
    checksum ^= frame_type;
    checksum ^= frame_id;
    for (size_t i = 0; i < payload_data_pos; ++i)
        checksum ^= payload_buffer[i];
    checksum ^= frame_cnt;
#else
    checksum += FRAME_SOF1;
    checksum += FRAME_SOF2;
    checksum += frame_len;
    checksum += frame_type;
    checksum += frame_id;
    for (size_t i = 0; i < payload_data_pos; ++i)
        checksum += payload_buffer[i];
    checksum += frame_cnt;
#endif

    return checksum;
}

bool ParseChar(uint8_t c, ScoutDecodedMessage *msg)
{
    static ScoutSerialDecodeState decode_state = WAIT_FOR_SOF1;

    bool new_frame_parsed = false;
    switch (decode_state)
    {
    case WAIT_FOR_SOF1:
    {
        if (c == FRAME_SOF1)
        {
            frame_id = FRAME_NONE_ID;
            frame_type = 0;
            frame_len = 0;
            frame_cnt = 0;
            frame_checksum = 0;
            internal_checksum = 0;
            payload_data_pos = 0;
            memset(payload_buffer, 0, PAYLOAD_BUFFER_SIZE);

            decode_state = WAIT_FOR_SOF2;
#ifdef PRINT_CPP_DEBUG_INFO
            std::cout << "found sof1" << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
            JLinkWriteString(0, "found sof1\n");
#endif
        }
        break;
    }
    case WAIT_FOR_SOF2:
    {
        if (c == FRAME_SOF2)
        {
            decode_state = WAIT_FOR_FRAME_LEN;
#ifdef PRINT_CPP_DEBUG_INFO
            std::cout << "found sof2" << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
            JLinkWriteString(0, "found sof2\n");
#endif
        }
        else
        {
            decode_state = WAIT_FOR_SOF1;
#ifdef PRINT_CPP_DEBUG_INFO
            std::cout << "failed to find sof2" << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
            JLinkWriteString(0, "failed to find sof2\n");
#endif
        }
        break;
    }
    case WAIT_FOR_FRAME_LEN:
    {
        frame_len = c;
        decode_state = WAIT_FOR_FRAME_TYPE;
#ifdef PRINT_CPP_DEBUG_INFO
        std::cout << "frame len: " << std::hex << static_cast<int>(frame_len) << std::dec << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
        JLinkRTTPrintf(0, "frame len: %d\n", frame_len);
#endif
        break;
    }
    case WAIT_FOR_FRAME_TYPE:
    {
        switch (c)
        {
        case FRAME_TYPE_CONTROL:
        {
            frame_type = FRAME_TYPE_CONTROL;
            decode_state = WAIT_FOR_FRAME_ID;
#ifdef PRINT_CPP_DEBUG_INFO
            std::cout << "control type frame received" << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
            JLinkWriteString(0, "control type frame received\n");
#endif
            break;
        }
        case FRAME_TYPE_STATUS:
        {
            frame_type = FRAME_TYPE_STATUS;
            decode_state = WAIT_FOR_FRAME_ID;
#ifdef PRINT_CPP_DEBUG_INFO
            std::cout << "status type frame received" << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
            JLinkWriteString(0, "status type frame received\n");
#endif
            break;
        }
        default:
        {
#ifdef PRINT_CPP_DEBUG_INFO
            std::cerr << "ERROR: Not expecting frame of a type other than FRAME_TYPE_STATUS" << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
            JLinkWriteString(0, "ERROR: Not expecting frame of a type other than FRAME_TYPE_STATUS\n");
#endif
            decode_state = WAIT_FOR_SOF1;
        }
        }
        break;
    }
    case WAIT_FOR_FRAME_ID:
    {
        switch (c)
        {
        case UART_FRAME_SYSTEM_STATUS_ID:
        case UART_FRAME_MOTION_STATUS_ID:
        case UART_FRAME_MOTOR1_DRIVER_STATUS_ID:
        case UART_FRAME_MOTOR2_DRIVER_STATUS_ID:
        case UART_FRAME_MOTOR3_DRIVER_STATUS_ID:
        case UART_FRAME_MOTOR4_DRIVER_STATUS_ID:
        case UART_FRAME_LIGHT_STATUS_ID:
        {
            frame_id = c;
            decode_state = WAIT_FOR_PAYLOAD;
#ifdef PRINT_CPP_DEBUG_INFO
            std::cout << "frame id: " << std::hex << static_cast<int>(frame_id) << std::dec << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
            JLinkRTTPrintf(0, "frame id: %d\n", frame_id);
#endif
            break;
        }
        default:
        {
#ifdef PRINT_CPP_DEBUG_INFO
            std::cerr << "ERROR: Unknown frame id" << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
            JLinkWriteString(0, "ERROR: Unknown frame id\n");
#endif
            decode_state = WAIT_FOR_SOF1;
        }
        }
        break;
    }
    case WAIT_FOR_PAYLOAD:
    {
        payload_buffer[payload_data_pos++] = c;
#ifdef PRINT_CPP_DEBUG_INFO
        std::cout << "1 byte added: " << std::hex << static_cast<int>(c) << std::dec << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
        JLinkRTTPrintf(0, "1 byte added: %d\n", c);
#endif
        if (payload_data_pos == (frame_len - FRAME_FIXED_FIELD_LEN))
            decode_state = WAIT_FOR_FRAME_COUNT;
        break;
    }
    case WAIT_FOR_FRAME_COUNT:
    {
        frame_cnt = c;
        decode_state = WAIT_FOR_CHECKSUM;
#ifdef PRINT_CPP_DEBUG_INFO
        std::cout << "frame count: " << std::hex << static_cast<int>(frame_cnt) << std::dec << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
        JLinkRTTPrintf(0, "frame count: %d\n", frame_cnt);
#endif
        break;
    }
    case WAIT_FOR_CHECKSUM:
    {
        frame_checksum = c;
        internal_checksum = CalcBufferedFrameChecksum();
        new_frame_parsed = true;
        decode_state = WAIT_FOR_SOF1;
#ifdef PRINT_CPP_DEBUG_INFO
        std::cout << "--- frame checksum: " << std::hex << static_cast<int>(frame_checksum) << std::dec << std::endl;
        std::cout << "--- internal frame checksum: " << std::hex << static_cast<int>(internal_checksum) << std::dec << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
        JLinkRTTPrintf(0, "--- frame checksum: : %d\n", frame_checksum);
        JLinkRTTPrintf(0, "--- internal frame checksum: : %d\n", internal_checksum);
#endif
        break;
    }
    default:
        break;
    }

    if (new_frame_parsed)
    {
        if (frame_checksum == internal_checksum)
        {
#ifdef PRINT_CPP_DEBUG_INFO
            std::cout << "checksum correct" << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
            JLinkWriteString(0, "checksum correct\n");
#endif
            if (frame_type == FRAME_TYPE_STATUS)
                ConstructStatusMessage(&(msg->status_msg));
            else if (frame_type == FRAME_TYPE_CONTROL)
                ConstructControlMessage(&(msg->control_msg));
            ++frame_parsed;
        }
        else
        {
            ++frame_with_wrong_checksum;
#ifdef PRINT_CPP_DEBUG_INFO
            std::cout << "checksum is NOT correct" << std::endl;
            std::cout << std::hex << static_cast<int>(frame_id) << " , " << static_cast<int>(frame_len) << " , " << static_cast<int>(frame_cnt) << " , " << static_cast<int>(frame_checksum) << " : " << std::dec << std::endl;
            std::cout << "payload: ";
            for (int i = 0; i < payload_data_pos; ++i)
                std::cout << std::hex << static_cast<int>(payload_buffer[i]) << std::dec << " ";
            std::cout << std::endl;
            std::cout << "--- frame checksum: " << std::hex << static_cast<int>(frame_checksum) << std::dec << std::endl;
            std::cout << "--- internal frame checksum: " << std::hex << static_cast<int>(internal_checksum) << std::dec << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
            JLinkWriteString(0, "checksum is NOT correct\n");
#endif
        }
    }

    return new_frame_parsed;
}

bool ConstructControlMessage(ScoutControlMessage *msg)
{
    if (msg == NULL)
        return false;

    switch (frame_id)
    {
    case UART_FRAME_MOTION_CONTROL_ID:
    {
        msg->msg_type = ScoutMotionControlMsg;
        msg->motion_control_msg.data.cmd.control_mode = payload_buffer[0];
        msg->motion_control_msg.data.cmd.fault_clear_flag = payload_buffer[1];
        msg->motion_control_msg.data.cmd.linear_velocity_cmd = payload_buffer[2];
        msg->motion_control_msg.data.cmd.angular_velocity_cmd = payload_buffer[3];
        msg->motion_control_msg.data.cmd.reserved0 = payload_buffer[4];
        msg->motion_control_msg.data.cmd.reserved1 = payload_buffer[5];
        msg->motion_control_msg.data.cmd.count = frame_cnt;
        msg->motion_control_msg.data.cmd.checksum = frame_checksum;
        break;
    }
    case UART_FRAME_LIGHT_CONTROL_ID:
    {
        msg->msg_type = ScoutLightControlMsg;
        msg->light_control_msg.data.cmd.light_ctrl_enable = payload_buffer[0];
        msg->light_control_msg.data.cmd.front_light_mode = payload_buffer[1];
        msg->light_control_msg.data.cmd.front_light_custom = payload_buffer[2];
        msg->light_control_msg.data.cmd.rear_light_mode = payload_buffer[3];
        msg->light_control_msg.data.cmd.rear_light_custom = payload_buffer[4];
        msg->light_control_msg.data.cmd.reserved0 = payload_buffer[5];
        msg->light_control_msg.data.cmd.count = frame_cnt;
        msg->light_control_msg.data.cmd.checksum = frame_checksum;
        break;
    }
    }
    return true;
}

bool ConstructStatusMessage(ScoutStatusMessage *msg)
{
    if (msg == NULL)
        return false;

    switch (frame_id)
    {
    case UART_FRAME_SYSTEM_STATUS_ID:
    {
        msg->msg_type = ScoutSystemStatusMsg;
        msg->system_status_msg.data.status.base_state = payload_buffer[0];
        msg->system_status_msg.data.status.control_mode = payload_buffer[1];
        msg->system_status_msg.data.status.battery_voltage.high_byte = payload_buffer[2];
        msg->system_status_msg.data.status.battery_voltage.low_byte = payload_buffer[3];
        msg->system_status_msg.data.status.fault_code.high_byte = payload_buffer[4];
        msg->system_status_msg.data.status.fault_code.low_byte = payload_buffer[5];
        msg->system_status_msg.data.status.count = frame_cnt;
        msg->system_status_msg.data.status.checksum = frame_checksum;
        break;
    }
    case UART_FRAME_MOTION_STATUS_ID:
    {
        msg->msg_type = ScoutMotionStatusMsg;
        msg->motion_status_msg.data.status.linear_velocity.high_byte = payload_buffer[0];
        msg->motion_status_msg.data.status.linear_velocity.low_byte = payload_buffer[1];
        msg->motion_status_msg.data.status.angular_velocity.high_byte = payload_buffer[2];
        msg->motion_status_msg.data.status.angular_velocity.low_byte = payload_buffer[3];
        msg->motion_status_msg.data.status.reserved0 = 0x00;
        msg->motion_status_msg.data.status.reserved0 = 0x00;
        msg->motion_status_msg.data.status.count = frame_cnt;
        msg->motion_status_msg.data.status.checksum = frame_checksum;
        break;
    }
    case UART_FRAME_MOTOR1_DRIVER_STATUS_ID:
    {
        msg->msg_type = ScoutMotorDriverStatusMsg;
        msg->motor_driver_status_msg.motor_id = SCOUT_MOTOR1_ID;
        msg->motor_driver_status_msg.data.status.current.high_byte = payload_buffer[0];
        msg->motor_driver_status_msg.data.status.current.low_byte = payload_buffer[1];
        msg->motor_driver_status_msg.data.status.rpm.high_byte = payload_buffer[2];
        msg->motor_driver_status_msg.data.status.rpm.low_byte = payload_buffer[3];
        msg->motor_driver_status_msg.data.status.temperature = payload_buffer[4];
        msg->motor_driver_status_msg.data.status.reserved0 = 0x00;
        msg->motor_driver_status_msg.data.status.count = frame_cnt;
        msg->motor_driver_status_msg.data.status.checksum = frame_checksum;
        break;
    }
    case UART_FRAME_MOTOR2_DRIVER_STATUS_ID:
    {
        msg->msg_type = ScoutMotorDriverStatusMsg;
        msg->motor_driver_status_msg.motor_id = SCOUT_MOTOR2_ID;
        msg->motor_driver_status_msg.data.status.current.high_byte = payload_buffer[0];
        msg->motor_driver_status_msg.data.status.current.low_byte = payload_buffer[1];
        msg->motor_driver_status_msg.data.status.rpm.high_byte = payload_buffer[2];
        msg->motor_driver_status_msg.data.status.rpm.low_byte = payload_buffer[3];
        msg->motor_driver_status_msg.data.status.temperature = payload_buffer[4];
        msg->motor_driver_status_msg.data.status.reserved0 = 0x00;
        msg->motor_driver_status_msg.data.status.count = frame_cnt;
        msg->motor_driver_status_msg.data.status.checksum = frame_checksum;
        break;
    }
    case UART_FRAME_MOTOR3_DRIVER_STATUS_ID:
    {
        msg->msg_type = ScoutMotorDriverStatusMsg;
        msg->motor_driver_status_msg.motor_id = SCOUT_MOTOR3_ID;
        msg->motor_driver_status_msg.data.status.current.high_byte = payload_buffer[0];
        msg->motor_driver_status_msg.data.status.current.low_byte = payload_buffer[1];
        msg->motor_driver_status_msg.data.status.rpm.high_byte = payload_buffer[2];
        msg->motor_driver_status_msg.data.status.rpm.low_byte = payload_buffer[3];
        msg->motor_driver_status_msg.data.status.temperature = payload_buffer[4];
        msg->motor_driver_status_msg.data.status.reserved0 = 0x00;
        msg->motor_driver_status_msg.data.status.count = frame_cnt;
        msg->motor_driver_status_msg.data.status.checksum = frame_checksum;
        break;
    }
    case UART_FRAME_MOTOR4_DRIVER_STATUS_ID:
    {
        msg->msg_type = ScoutMotorDriverStatusMsg;
        msg->motor_driver_status_msg.motor_id = SCOUT_MOTOR4_ID;
        msg->motor_driver_status_msg.data.status.current.high_byte = payload_buffer[0];
        msg->motor_driver_status_msg.data.status.current.low_byte = payload_buffer[1];
        msg->motor_driver_status_msg.data.status.rpm.high_byte = payload_buffer[2];
        msg->motor_driver_status_msg.data.status.rpm.low_byte = payload_buffer[3];
        msg->motor_driver_status_msg.data.status.temperature = payload_buffer[4];
        msg->motor_driver_status_msg.data.status.reserved0 = 0x00;
        msg->motor_driver_status_msg.data.status.count = frame_cnt;
        msg->motor_driver_status_msg.data.status.checksum = frame_checksum;
        break;
    }
    case UART_FRAME_LIGHT_STATUS_ID:
    {
        msg->msg_type = ScoutLightStatusMsg;
        msg->light_status_msg.data.status.light_ctrl_enable = payload_buffer[0];
        msg->light_status_msg.data.status.front_light_mode = payload_buffer[1];
        msg->light_status_msg.data.status.front_light_custom = payload_buffer[2];
        msg->light_status_msg.data.status.rear_light_mode = payload_buffer[3];
        msg->light_status_msg.data.status.rear_light_custom = payload_buffer[4];
        msg->light_status_msg.data.status.reserved0 = 0x00;
        msg->light_status_msg.data.status.count = frame_cnt;
        msg->light_status_msg.data.status.checksum = frame_checksum;
        break;
    }
    }
    return true;
}
