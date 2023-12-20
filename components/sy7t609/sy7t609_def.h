#pragma once

namespace esphome {
namespace sy7t609 {
enum sy7t609_register_map {
    ADDR_COMMAND     = 0x0000,
    ADDR_FW_VER      = 0x0003,
    ADDR_CONTROL     = 0x0006,
    //Metering Address
    ADDR_VAVG        = 0x002D,
    ADDR_IAVG        = 0x0030,
    ADDR_VRMS        = 0x0033,
    ADDR_IRMS        = 0x0036,
    ADDR_POWER       = 0x0039,
    ADDR_VAR         = 0x003C,
    ADDR_FREQUENCY   = 0x0042,
    ADDR_AVG_POWER   = 0x0045,
    ADDR_PF          = 0x0048,
    ADDR_EPPCNT      = 0x0069, //Positive Active Energy Count
    ADDR_EPMCNT      = 0x006C, //Negative Active Energy Count
    ADDR_IPEAK       = 0x008A,
    ADDR_VPEAK       = 0x0093,
    //I/O Control Address
    ADDR_DIO_DIR     = 0x0099,
    ADDR_DIO_SET     = 0x009F,
    ADDR_DIO_RST     = 0x00A2,
    //Calibration Address
    ADDR_BUCKETL     = 0x00C0,
    ADDR_BUCKETH     = 0x00C3,
    ADDR_IGAIN       = 0x00D5,
    ADDR_VGAIN       = 0x00D8,
    ADDR_ISCALE      = 0x00ED,
    ADDR_VSCALE      = 0x00F0,
    ADDR_PSCALE      = 0x00F3,
    ADDR_ACCUM       = 0x0105,
    ADDR_IRMS_TARGET = 0x0117,
    ADDR_VRMS_TARGET = 0x011A,
    ADDR_ERROR = 0x0FFF
};

enum command_register_code {
    CMD_REG_CLEAR_ENGERGY_COUNTERS = 0xEC0000, //Clear All Energy Counters
    CMD_REG_SOFT_RESET             = 0xBD0000, //Invoke Soft-Reset
    CMD_REG_SAVE_TO_FLASH          = 0xACC200,
    CMD_REG_CLEAR_FLASH_STORAGE_0  = 0xACC000,
    CMD_REG_CLEAR_FLASH_STORAGE_1  = 0XACC100,
    CMD_REG_CALIBRATION_VOLTAGE    = 0xCA0020,
    CMD_REG_CALIBRATION_CURRENT    = 0xCA0010,
    CMD_REG_CALIBRATION_ALL        = 0xCA0030
};

#define COMMAND_REGISTER_CALIBRATION_MASK (0xFF0000)
#define CONTROL_REGISTER_MASK             (0x001815)

#define SSI_HEADER (0xAA)
#define SSI_DEFAULT_FRAME_SIZE   (3)
#define SSI_MAX_PAYLOAD_SIZE     (7)
#define SSI_READ_PAYLOAD_SIZE    (4)
#define SSI_REPLY_PAYLOAD_SIZE   (3)
#define SSI_WRITE_PAYLOAD_SIZE   (7)
#define SSI_UART_SEND_READ_PKG_SIZE ((SSI_DEFAULT_FRAME_SIZE + SSI_READ_PAYLOAD_SIZE))
#define SSI_UART_SEND_WRITE_PKG_SIZE ((SSI_DEFAULT_FRAME_SIZE + SSI_WRITE_PAYLOAD_SIZE))
#define SSI_UART_RECV_PKG_SIZE ((SSI_DEFAULT_FRAME_SIZE + SSI_REPLY_PAYLOAD_SIZE))

#define CMD_CLEAR_ADDRESS            (0xA0)
#define CMD_SELECT_REGISTER_ADDRESS  (0xA3)
#define CMD_READ_REGITSTER_3BYTES    (0xE3)
#define CMD_WRITE_RETISTER_3BYTES    (0xD3)

enum sy7t609_reply_code {
    REPLY_ACK_WITH_DATA           = 0xAA,
    REPLY_AUTO_REPORTING_HEADER   = 0xAE,
    REPLY_ACK_WITHOUT_DATA        = 0xAD,
    REPLY_NEGATIVE_ACK            = 0xB0,
    REPLY_COMMAND_NOT_IMPLEMENTED = 0xBC,
    REPLY_CHECKSUM_FAILED         = 0xBD,
    REPLY_BUFFER_OVERFLOW         = 0xBF
};

typedef enum process_state {
    PROCESS_STATE_READ_PF = 0,
    PROCESS_STATE_READ_VRMS,
    PROCESS_STATE_READ_IRMS,
    PROCESS_STATE_READ_POWER,
    PROCESS_STATE_READ_AVG_POWER,
    PROCESS_STATE_READ_EPPCNT,
    PROCESS_STATE_READ_EPMCNT,
	PROCESS_STATE_DELAY_1,
	PROCESS_STATE_DELAY_2,
    PROCESS_STATE_UPDATE_INFO
} process_state_t;

typedef struct ssi_command_packet_frame {
    uint8_t header;
    uint8_t byte_count;
    uint8_t payload[SSI_MAX_PAYLOAD_SIZE];
    uint8_t checksum;
} ssi_command_packet_frame_t;

typedef struct ssi_reply_packet_frame {
    uint8_t reply_code;
	uint8_t byte_count;
    uint8_t payload[SSI_REPLY_PAYLOAD_SIZE];
    uint8_t checksum;
} ssi_reply_packet_frame_t;

} //sy7t609
} //esphome