#include "sy7t609_uart.h"
#include "esphome/core/log.h"
#include <cinttypes>

namespace esphome {
namespace sy7t609 {

static const char *const TAG = "sy7t609";

void SY7T609_UART::setup() {
  // Clear UART buffer
  while (this->available())
    this->read();
}

void SY7T609_UART::loop() {
  const uint32_t now = millis();
  if (now - this->last_read_ > 500 && this->available() < SSI_UART_SEND_READ_PKG_SIZE) {
    while (this->available())
      this->read();
    this->last_read_ = now;
  }

 
  while (this->available() >= SSI_UART_SEND_READ_PKG_SIZE) {
    auto resp = *this->read_array<SSI_UART_RECV_PKG_SIZE>();

    uint8_t sum = 0;
    for (int i = 0; i < SSI_UART_RECV_PKG_SIZE; i++) {
        sum += resp[i];
    }
    sum = ~sum + 1;
    if (sum != resp[6]) {
      ESP_LOGW(TAG, "SY7T609_UART invalid checksum! 0x%02X != 0x%02X", sum, resp[6]);
      continue;
    }

    switch (read_state_) {

      case PROCESS_STATE_READ_VRMS: { 
        float voltage = readVRMS(resp);
        if (this->voltage_sensor_ != nullptr)
          this->voltage_sensor_->publish_state(voltage);
        ESP_LOGD(TAG, "Got Voltage %.1f V", voltage);
        this->write_state_(PROCESS_STATE_READ_VRMS);
        break;
      }

      case PROCESS_STATE_READ_POWER: {  // Active Power Response
        uint16_t power = readPOWER(resp);
        if (this->power_sensor_ != nullptr)
          this->power_sensor_->publish_state(power);
        ESP_LOGD(TAG, "Got Power %u W", power);
        this->write_state_(PROCESS_DONE);
        break;
      }

      case PROCESS_STATE_READ_PF:
      case PROCESS_STATE_READ_IRMS:
      case PROCESS_STATE_READ_AVG_POWER:
      case PROCESS_STATE_READ_EPPCNT:
      case PROCESS_STATE_READ_EPMCNT:
      case PROCESS_STATE_DELAY_1:
      case PROCESS_STATE_DELAY_2:
      case PROCESS_STATE_UPDATE_INFO:
      default:
        break;
    }

    this->last_read_ = now;
  }
}
void SY7T609_UART::update() { this->write_state_(PROCESS_STATE_READ_VRMS); }
uint16_t SY7T609_UART::get_reg_addr(SY7T609_UART::process_state state)
{
    uint16_t addr = ADDR_ERROR;
    switch(state)
    {
        case PROCESS_STATE_READ_VRMS:
            addr = ADDR_VRMS;
            break;
        case PROCESS_STATE_READ_POWER:
            addr = ADDR_POWER;
            break;
        default:
            break;
    }

    return addr;
}
void SY7T609_UART::write_state_(SY7T609_UART::process_state_t state) {
  if (state == PROCESS_DONE) 
  {
    this->read_state_ = state;
    return;
  }
  uint16_t reg_addr = get_reg_addr(state);
  if(ADDR_ERROR == reg_addr)
  {
    ESP_LOGE(TAG, "SY7T609_UART invalid process_state[%d]!", state);
    return ;
  }
  std::array<uint8_t, 7> data{};
  //header
  data[0] = SSI_HEADER;
  //byte_count
  data[1] = SSI_UART_SEND_READ_PKG_SIZE;
  //payload[4]
  data[2] = CMD_SELECT_REGISTER_ADDRESS;
  data[3] = addr & 0xFF;;
  data[4] = (addr >> 8) & 0xFF;
  data[5] = CMD_READ_REGITSTER_3BYTES;
  //checksum
  data[6] = 0;
  uint8_t checksum = 0;
  for (int i = 0; i < 6; i++)
  {
    checksum += data[i];
  }
  checksum = ~checksum + 1;
  data[6] = checksum;

  this->write_array(data);
  this->read_state_ = state;
}
void SY7T609_UART::dump_config() {
  ESP_LOGCONFIG(TAG, "SY7T609_UART:");
  LOG_SENSOR("", "Voltage", this->voltage_sensor_);
  LOG_SENSOR("", "Current", this->current_sensor_);
  LOG_SENSOR("", "Power", this->power_sensor_);
}

  float readVRMS(std::array<uint8_t> *array)
  {
    return 0.0f;
  }
  float readPOWER(std::array<uint8_t> *array) 
  {
    return 0.0f;
  }
}  // namespace pzem004t
}  // namespace esphome
