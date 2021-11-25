// TODO Weili: Create a func to check for header match
const uint32_t HEADER = 0x92384233;
const uint8_t HEADER_BYTE1 = 0x33;
const uint8_t HEADER_BYTE2 = 0x42;
const uint8_t HEADER_BYTE3 = 0x38;
const uint8_t HEADER_BYTE4 = 0x92;

// For UART (Not ESP-NOW)
// 16 bytes
typedef struct Command_Data {
    float leftAngularVelo;
    float rightAngularVelo;
    float angleRotatedLeftMotor;
    float angleRotatedRightMotor;
} Command_Data;

// 32 bytes
typedef struct Command_Data_Packet_UART {
    uint32_t header;
    Command_Data commandData;
    uint64_t timestamp;
    uint32_t CRC;
} Command_Data_Packet_UART;

// 28 bytes
typedef struct Command_Data_Packet_ESP_NOW {
    Command_Data commandData;
    uint64_t timestamp;
    uint32_t CRC;
} Command_Data_Packet_ESP_NOW;

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
typedef struct Feedback_Data_Packet_UART {
    uint32_t header;
    Feedback_Data feedBackData;
    uint64_t timestamp;
    uint32_t CRC;
} Feedback_Data_Packet_UART;

typedef struct Feedback_Data_Packet_ESP_NOW {
    Feedback_Data feedBackData;
    uint64_t timestamp;
    uint32_t CRC;
} Feedback_Data_Packet_ESP_NOW;