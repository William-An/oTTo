static const int RX_BUF_SIZE = 1024;

const uint32_t HEADER = 0x92384233;
// const uint32_t COMMAND_DATA_SIZE = 16;
// const uint32_t COMMAND_DATA_PACKET_SIZE = 32;
// const uint32_t FEEDBACK_DATA_SIZE = 28;
// const uint32_t FEEDBACK_DATA_PACKET_SIZE = 44;

// For UART (Not ESP-NOW)
// 16 bytes
typedef struct Command_Data {
    float leftAngularVelo;
    float rightAngularVelo;
    float angleRotatedLeftMotor;
    float angleRotatedRightMotor;
} Command_Data;

// 32 bytes
typedef struct Command_Data_Packet {
    uint32_t header;
    Command_Data commandData;
    uint64_t timestamp;
    uint32_t CRC;
} Command_Data_Packet;

// 28 bytes
typedef struct Feedback_Data {
    float leftAngularVelo;
    float rightAngularVelo;
    float angleRotatedLeftMotor;
    float angleRotatedRightMotor;
    float yaw;
    float pitch;
    float roll;
} Feedback_Data;

// 44 bytes
typedef struct Feedback_Data_Packet {
    uint32_t header;
    Feedback_Data feedBackData;
    uint64_t timestamp;
    uint32_t CRC;
} Feedback_Data_Packet;