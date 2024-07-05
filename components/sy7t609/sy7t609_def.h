#pragma once

namespace esphome {
namespace sy7t609 {

/*寄存器默认值*/
#define ISCALE        (0x00F230)
#define VSCALE        (0x0A2D78)
#define PSCALE        (0x027703)
#define ACCUM         (0x001A2C)
#define IRMS_TARGET   (0x0003E8)
#define VRMS_TARGET   (0x035B60)
#define POWER_TARGET  (0x01D4C0)
#define CONTROL       (0x001817)
#define BUCKETH       (0x000247)
//#define BUCKETL       (0x7780A5)   /*原厂默认的数据*/
#define BUCKETL       (0xD5AEE6)   /*其他正常插排的数据*/
#define IGAIN         (0x43DF0D)    /*使用万用表校准的的数据*/
#define VGAIN         (0x208155)

#define COMMAND_REGISTER_CALIBRATION_MASK (0xFF0000)
#define CONTROL_REGISTER_MASK             (0x001815)

//[23-20]   [19-16]  [15-12]  [11-8]  [7-4]   [3-0]
//0000      0100     1000     1000     0001     0111
// #define CONTROL_REGISTER_BIT_SET       (0x048817)  
// #define CONTROL_REGISTER_BIT_SET       (0x1817)  // LXY1226
#define CONTROL_REGISTER_BIT_SET       (0x181F) 
#define SSI_HEADER (0xAA)
#define SSI_DEFAULT_FRAME_SIZE   (3)
#define SSI_MAX_PAYLOAD_SIZE     (7)
#define SSI_READ_PAYLOAD_SIZE    (4)
#define SSI_REPLY_PAYLOAD_SIZE   (3)
#define SSI_WRITE_PAYLOAD_SIZE   (7)
#define SSI_UART_READ_SEND_PKG_SIZE ((SSI_DEFAULT_FRAME_SIZE + SSI_READ_PAYLOAD_SIZE))
#define SSI_UART_WRITE_SEND_PKG_SIZE ((SSI_DEFAULT_FRAME_SIZE + SSI_WRITE_PAYLOAD_SIZE))
#define SSI_UART_READ_RECV_PKG_SIZE ((SSI_DEFAULT_FRAME_SIZE + SSI_REPLY_PAYLOAD_SIZE)-1)
#define SSI_UART_AUTO_REPORT_PKG_SIZE (0x21-1) // packet-size-header
#define SSI_UART_WRITE_RECV_PKG_SIZE (1)

#define CMD_CLEAR_ADDRESS            (0xA0)
#define CMD_SELECT_REGISTER_ADDRESS  (0xA3)
#define CMD_READ_REGITSTER_3BYTES    (0xE3)
#define CMD_WRITE_RETISTER_3BYTES    (0xD3)

enum sy7t609_register_map {
    ADDR_COMMAND       = 0x0000,
    ADDR_FW_VER        = 0x0003,
    ADDR_CONTROL       = 0x0006,
    //Metering Address
    ADDR_CTEMP         = 0x0027,
    ADDR_VAVG          = 0x002D,
    ADDR_IAVG          = 0x0030,
    ADDR_VRMS          = 0x0033, 
    ADDR_IRMS          = 0x0036,
    ADDR_POWER         = 0x0039,
    ADDR_VAR           = 0x003C,
    ADDR_FREQUENCY     = 0x0042,
    ADDR_AVG_POWER     = 0x0045,
    ADDR_PF            = 0x0048,
    ADDR_EPPCNT        = 0x0069, //Positive Active Energy Count
    ADDR_EPMCNT        = 0x006C, //Negative Active Energy Count
    ADDR_IPEAK         = 0x008A,
    ADDR_VPEAK         = 0x0093,
    //I/O Control Address
    ADDR_DIO_DIR       = 0x0099,
    ADDR_DIO_SET       = 0x009F,
    ADDR_DIO_RST       = 0x00A2,
    //Calibration Address
    ADDR_BUCKETL       = 0x00C0,
    ADDR_BUCKETH       = 0x00C3,
    ADDR_IGAIN         = 0x00D5,
    ADDR_VGAIN         = 0x00D8,
    ADDR_ISCALE        = 0x00ED,
    ADDR_VSCALE        = 0x00F0,
    ADDR_PSCALE        = 0x00F3,
    ADDR_ACCUM_CYC     = 0x0102,
    ADDR_ACCUM         = 0x0105,
    ADDR_IAVG_TARGET   = 0x0111,
    ADDR_VAVG_TARGET   = 0x0114,
    ADDR_IRMS_TARGET   = 0x0117,
    ADDR_VRMS_TARGET   = 0x011A,
    ADDR_POWER_TARGET  = 0x011D,
    ADDR_BAUD_RATE     = 0x0120,
    ADDR_ERROR         = 0x0FFF
};

enum command_register_code {
    CMD_NONE = 0x000000,
    CMD_REG_CLEAR_ENGERGY_COUNTERS = 0xEC0000, //Clear All Energy Counters
    CMD_REG_SOFT_RESET             = 0xBD0000, //Invoke Soft-Reset
    CMD_REG_SAVE_TO_FLASH          = 0xACC200,
    CMD_REG_CLEAR_FLASH_STORAGE_0  = 0xACC000,
    CMD_REG_CLEAR_FLASH_STORAGE_1  = 0XACC100,
    CMD_REG_CALIBRATION_VOLTAGE    = 0xCA0020,
    CMD_REG_CALIBRATION_CURRENT    = 0xCA0010,
    CMD_REG_CALIBRATION_ALL        = 0xCA0030
};

enum sy7t609_reply_code {
    REPLY_ACK_WITH_DATA           = 0xAA,
    REPLY_AUTO_REPORTING_HEADER   = 0xAE,
    REPLY_ACK_WITHOUT_DATA        = 0xAD,
    REPLY_NEGATIVE_ACK            = 0xB0,
    REPLY_COMMAND_NOT_IMPLEMENTED = 0xBC,
    REPLY_CHECKSUM_FAILED         = 0xBD,
    REPLY_BUFFER_OVERFLOW         = 0xBF
};
/*读取寄存器的数据结构*/
typedef struct ssi_command_packet_frame {
    uint8_t header;
    uint8_t byte_count;
    uint8_t payload[SSI_MAX_PAYLOAD_SIZE];
    uint8_t checksum;
} ssi_command_packet_frame_t;

/*写入寄存器的数据结构*/
typedef struct ssi_reply_packet_frame {
    uint8_t reply_code;
	uint8_t byte_count;
    uint8_t payload[SSI_REPLY_PAYLOAD_SIZE];
    uint8_t checksum;
} ssi_reply_packet_frame_t;

typedef enum process_state_ 
{
    PROCESS_DONE = 0,
    PROCESS_STATE_READ_REGISTER_MIN,
    PROCESS_STATE_READ_PF,
    PROCESS_STATE_READ_VRMS,
    PROCESS_STATE_READ_IRMS,
    PROCESS_STATE_READ_POWER,
    PROCESS_STATE_READ_REACTIVE_POWER,
    PROCESS_STATE_READ_EPPCNT,
    PROCESS_STATE_READ_FREQUENCY,
    PROCESS_STATE_READ_TEMPERATURE,
    PROCESS_STATE_READ_ADDR_IGAIN,
    PROCESS_STATE_READ_ADDR_VGAIN,
    PROCESS_STATE_READ_ADDR_ISCALE,
    PROCESS_STATE_READ_ADDR_VSCALE,
    PROCESS_STATE_READ_ADDR_PSCALE,
    PROCESS_STATE_READ_ADDR_ACCUM,
    PROCESS_STATE_READ_ADDR_IRMS_TARGET,
    PROCESS_STATE_READ_ADDR_VRMS_TARGET,
    PROCESS_STATE_READ_ADDR_POWER_TARGET,
    PROCESS_STATE_READ_ADDR_BAUD,
    PROCESS_STATE_READ_BUCKETL,
    PROCESS_STATE_READ_BUCKETH,
    PROCESS_STATE_READ_CONTROL_REGISTER,
    PROCESS_STATE_READ_REGISTER_MAX,
    PROCESS_STATE_WRITE_REGISTER_MIN,
    PROCESS_STATE_WRITE_ADDR_IGAIN,
    PROCESS_STATE_WRITE_ADDR_VGAIN,
    PROCESS_STATE_WRITE_ADDR_ISCALE,
    PROCESS_STATE_WRITE_ADDR_VSCALE,
    PROCESS_STATE_WRITE_ADDR_PSCALE,
    PROCESS_STATE_WRITE_ADDR_ACCUM,
    PROCESS_STATE_WRITE_ADDR_IRMS_TARGET,
    PROCESS_STATE_WRITE_ADDR_VRMS_TARGET,
    PROCESS_STATE_WRITE_ADDR_POWER_TARGET,
    PROCESS_STATE_WRITE_BUCKETL,
    PROCESS_STATE_WRITE_BUCKETH,
    PROCESS_STATE_WRITE_CONTROL_REGISTER,
    PROCESS_STATE_WRITE_CMD_REG_SAVE_TO_FLASH,
    PROCESS_STATE_WRITE_CMD_REG_CLEAR_ENGERGY_COUNTERS,
    PROCESS_STATE_WRITE_CALIBRATION_ALL_REGISTER,
    PROCESS_STATE_WRITE_CMD_REG_SOFT_RESET,
    PROCESS_STATE_WRITE_REGISTER_MAX,
    PROCESS_STATE_READ_CALIBRATION_ALL_REGISTER
} process_state;

} //sy7t609
} //esphome