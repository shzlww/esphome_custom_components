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
  if (now - this->last_read_ > 500 && this->available() != SSI_UART_RECV_PKG_SIZE) {
    while (this->available())
      this->read();
    this->last_read_ = now;
  }

 
  while (this->available() == SSI_UART_RECV_PKG_SIZE) {
    auto resp = *this->read_array<SSI_UART_RECV_PKG_SIZE>();
    
    if (resp[0] != REPLY_ACK_WITH_DATA) {
        //SSI Error
        continue;
    }
    uint8_t sum = 0;
    for (int i = 0; i < resp.size() - 1; i++) {
       ESP_LOGV(TAG, "SY7T609_UART process_state[%d],Recv<<<<--[%02x]!", read_state_,resp[i]);
        sum += resp[i];
    }
    ESP_LOGV(TAG, "SY7T609_UART process_state[%d],Recv<<<<--[%02x]!", read_state_,resp[resp.size() - 1]);
    sum = ~sum + 1;
    if (sum != resp[resp.size() - 1]) {
      ESP_LOGW(TAG, "SY7T609_UART invalid checksum! 0x%02X != 0x%02X", sum, resp[resp.size() - 1]);
      continue;
    }

    switch (read_state_) {

      case PROCESS_STATE_READ_PF: {
        float data = readPF(resp);
        if (this->power_factor_sensor_ != nullptr)
          this->power_factor_sensor_->publish_state(data);
        ESP_LOGD(TAG, "Got Frequency %.1f Hz", data);
        this->write_state_(PROCESS_STATE_READ_VRMS);
        break;
      }
      case PROCESS_STATE_READ_VRMS: { 
        float data = readVRMS(resp);
        if (this->voltage_sensor_ != nullptr)
          this->voltage_sensor_->publish_state(data);
        ESP_LOGD(TAG, "Got Voltage %.1f V", data);
        this->write_state_(PROCESS_STATE_READ_IRMS);
        break;
      }
      case PROCESS_STATE_READ_IRMS: { 
        float data = readIRMS(resp);
        if (this->current_sensor_ != nullptr)
          this->current_sensor_->publish_state(data);
        ESP_LOGD(TAG, "Got Current %.3f A", data);
        this->write_state_(PROCESS_STATE_READ_POWER);
        break;
      }
      case PROCESS_STATE_READ_POWER: {
        float data = readPower(resp);
        if (this->power_sensor_ != nullptr)
          this->power_sensor_->publish_state(data);
        ESP_LOGD(TAG, "Got Power %.1f W", data);
        this->write_state_(PROCESS_STATE_READ_EPPCNT);
        break;
      }
      case PROCESS_STATE_READ_EPPCNT: { 
        float data = readPositiveActiveEnergy(resp);
        if (this->energy_sensor_ != nullptr)
          this->energy_sensor_->publish_state(data);
        ESP_LOGD(TAG, "Got Energy %.1f ", data);
        this->write_state_(PROCESS_STATE_READ_FREQUENCY);
        break;
      }
      //case PROCESS_STATE_READ_EPMCNT: {
      //  float data = readNegativeActiveEnergy(resp);
      //  if (this->energy_negative_active_sensor_ != nullptr)
      //    this->energy_negative_active_sensor_->publish_state(data);
      //  ESP_LOGD(TAG, "Got Negative Active Energy %.1f ", data);
      //  this->write_state_(PROCESS_STATE_READ_FREQUENCY);
      //  break;
      //}
      case PROCESS_STATE_READ_FREQUENCY: {
        float data = readFrequency(resp);
        if (this->frequency_sensor_ != nullptr)
          this->frequency_sensor_->publish_state(data);
        ESP_LOGD(TAG, "Got Frequency %.1f ", data);
        this->write_state_(PROCESS_DONE);
        break;
      }
      default:
        break;
    }

    this->last_read_ = now;
  }
}
void SY7T609_UART::update() 
{
   this->write_state_(PROCESS_STATE_READ_PF); 
}
uint16_t SY7T609_UART::get_reg_addr(SY7T609_UART::process_state state)
{
    uint16_t addr = ADDR_ERROR;
    switch(state)
    {
        case PROCESS_STATE_READ_PF:
            addr = ADDR_PF;
            break;
        case PROCESS_STATE_READ_VRMS:
            addr = ADDR_VRMS;
            break;
        case PROCESS_STATE_READ_IRMS:
            addr = ADDR_IRMS;
            break;
        case PROCESS_STATE_READ_POWER:
            addr = ADDR_POWER;
            break;
        case PROCESS_STATE_READ_AVG_POWER:
            addr = ADDR_AVG_POWER;
            break;
        case PROCESS_STATE_READ_EPPCNT:
            addr = ADDR_EPPCNT;
            break;
        case PROCESS_STATE_READ_EPMCNT:
            addr = ADDR_EPMCNT;
            break;
        case PROCESS_STATE_READ_FREQUENCY:
            addr = ADDR_FREQUENCY;
            break;
        default:
            break;
    }

    return addr;
}
void SY7T609_UART::write_state_(SY7T609_UART::process_state state) {
  if (state == PROCESS_DONE) 
  {
    this->read_state_ = state;
    return;
  }
  uint16_t addr = get_reg_addr(state);
  if(ADDR_ERROR == addr)
  {
    ESP_LOGE(TAG, "SY7T609_UART invalid process_state[%d]!", state);
    return ;
  }
  std::array<uint8_t, SSI_UART_SEND_READ_PKG_SIZE> data{};
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
  for (int i = 0; i < SSI_UART_SEND_READ_PKG_SIZE - 1; i++)
  {
    checksum += data[i];
  }
  checksum = ~checksum + 1;
  data[6] = checksum;
  for(int i = 0; i < data.size();i++)
  {
    ESP_LOGV(TAG, "SY7T609_UART process_state[%d],Send-->>>>[%02x]!", state,data[i]);
  }

  this->write_array(data);
  this->read_state_ = state;
}
void SY7T609_UART::dump_config() {
  ESP_LOGCONFIG(TAG, "SY7T609_UART:");
  LOG_SENSOR("", "Voltage", this->voltage_sensor_);
  LOG_SENSOR("", "Current", this->current_sensor_);
  LOG_SENSOR("", "Power", this->power_sensor_);
}

/* resp package struct:
    index       type       desc
    array[0]----uint8_t----reply_code;
    array[1]----uint8_t----byte_count;
    array[2]----uint8_t----payload[0];
    array[3]----uint8_t----payload[1];
    array[4]----uint8_t----payload[2];
    array[5]----uint8_t----checksum;
*/
float SY7T609_UART::readVRMS(const std::array<uint8_t,SSI_UART_RECV_PKG_SIZE> &array)
{
  uint32_t data = 0;
  data = ((uint32_t)array[4]<<16) | ((uint32_t)array[3]<<8) | (uint32_t)array[2];
  
  return data/1000.0f;
}

float SY7T609_UART::readIRMS(const std::array<uint8_t,SSI_UART_RECV_PKG_SIZE> &array)
{
  uint32_t data = 0;
  data = ((uint32_t)array[4]<<16) | ((uint32_t)array[3]<<8) | (uint32_t)array[2];
  //data = (uint32_t)(data/128);
  
  return data/1000.0f;
}

float SY7T609_UART::readPower(const std::array<uint8_t,SSI_UART_RECV_PKG_SIZE> &array) 
{
  uint32_t data = 0;
  data = ((uint32_t)array[4]<<16) | ((uint32_t)array[3]<<8) | (uint32_t)array[2];
  if(data >= 0x800000) 
  {
		data = 0x01000000 - data;
	}
  
  return data/1000.0f;
}
float SY7T609_UART::readAvgPower(const std::array<uint8_t,SSI_UART_RECV_PKG_SIZE> &array) 
{
  uint32_t data = 0;
  data = ((uint32_t)array[4]<<16) | ((uint32_t)array[3]<<8) | (uint32_t)array[2];
  
  return data/1000.0f;
}
float SY7T609_UART::readPF(const std::array<uint8_t,SSI_UART_RECV_PKG_SIZE> &array)
{
  uint32_t value = 0;
  value = ((uint32_t)array[4]<<16) | ((uint32_t)array[3]<<8) | (uint32_t)array[2];
  if(value >= 0x800000)
  {
    value = 0x01000000 - value;
  }
  return value/1000.0f;
}
float SY7T609_UART::readPositiveActiveEnergy(const std::array<uint8_t,SSI_UART_RECV_PKG_SIZE> &array)
{
  uint32_t data = 0;
  data = ((uint32_t)array[4]<<16) | ((uint32_t)array[3]<<8) | (uint32_t)array[2];
  
  return data/1000.0f;;
}
float SY7T609_UART::readNegativeActiveEnergy(const std::array<uint8_t,SSI_UART_RECV_PKG_SIZE> &array)
{
  uint32_t data = 0;
  data = ((uint32_t)array[4]<<16) | ((uint32_t)array[3]<<8) | (uint32_t)array[2];
  
  return data/1000.0f;
}
float SY7T609_UART::readFrequency(const std::array<uint8_t,SSI_UART_RECV_PKG_SIZE> &array)
{
  uint32_t data = 0;
  data = ((uint32_t)array[4]<<16) | ((uint32_t)array[3]<<8) | (uint32_t)array[2];
  
  return data/1000.0f;
}

}  // namespace pzem004t
}  // namespace esphome
