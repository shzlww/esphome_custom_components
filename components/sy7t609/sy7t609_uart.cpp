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
  //register-write cmd recieved ,should block the register-read
  if(this->m_interrupt) 
  {
    if (this->available() != SSI_UART_WRITE_RECV_PKG_SIZE) 
    {
      while (this->available())
        this->read();
    }

    while (this->available() == SSI_UART_WRITE_RECV_PKG_SIZE) 
    {
      auto resp = *this->read_array<SSI_UART_WRITE_RECV_PKG_SIZE>();
      if (resp[0] != REPLY_ACK_WITHOUT_DATA) 
      {
        ESP_LOGE(TAG, "SY7T609_UART register-write failure! response-code <<<< [%02x]!", resp[0]);
      }
      else
      {
        ESP_LOGI(TAG, "SY7T609_UART register-write success! response-code <<<< [%02x]!", resp[0]);
      }
    }
    this->write_state_(PROCESS_DONE);
    this->m_interrupt = false; //unblock the register-read  
    return;
  }
  else
  {
    if (now - this->last_read_ > 500 && this->available() != SSI_UART_READ_RECV_PKG_SIZE) {
      while (this->available())
        this->read();
      this->last_read_ = now;
    }

    while (this->available() == SSI_UART_READ_RECV_PKG_SIZE) {
      auto resp = *this->read_array<SSI_UART_READ_RECV_PKG_SIZE>();
      
      if (resp[0] != REPLY_ACK_WITH_DATA) {
          //SSI Error
          continue;
      }
      uint8_t sum = 0;
      for (int i = 0; i < resp.size() - 1; i++) {
        ESP_LOGV(TAG, "SY7T609_UART process_state[%d],Recv<<<<--[%02x]!", m_process_state,resp[i]);
          sum += resp[i];
      }
      ESP_LOGV(TAG, "SY7T609_UART process_state[%d],Recv<<<<--[%02x]!", m_process_state,resp[resp.size() - 1]);
      sum = ~sum + 1;
      if (sum != resp[resp.size() - 1]) {
        ESP_LOGW(TAG, "SY7T609_UART invalid checksum! 0x%02X != 0x%02X", sum, resp[resp.size() - 1]);
        continue;
      }

      switch (m_process_state) {

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
          this->write_state_(PROCESS_STATE_READ_REACTIVE_POWER);
          break;
        }
        case PROCESS_STATE_READ_REACTIVE_POWER: {
          float data = readReactivePower(resp);
          if (this->power_reactive_sensor_ != nullptr)
            this->power_reactive_sensor_->publish_state(data);
          ESP_LOGD(TAG, "Got Reactive Power %.1f W", data);
          this->write_state_(PROCESS_STATE_READ_EPPCNT);
          break;
        }
        case PROCESS_STATE_READ_EPPCNT: { 
          float data = readEnergy(resp);
          if (this->energy_sensor_ != nullptr)
            this->energy_sensor_->publish_state(data);
          ESP_LOGD(TAG, "Got Positive Active Energy %.1f ", data);
          this->write_state_(PROCESS_STATE_READ_FREQUENCY);
          break;
        }
        case PROCESS_STATE_READ_FREQUENCY: {
          float data = readFrequency(resp);
          if (this->frequency_sensor_ != nullptr)
            this->frequency_sensor_->publish_state(data);
          ESP_LOGD(TAG, "Got Frequency %.1f ", data);
          this->write_state_(PROCESS_STATE_READ_TEMPERATURE);
          break;
        }
        case PROCESS_STATE_READ_TEMPERATURE: {
          float data = readFrequency(resp);
          if (this->temperature_sensor_ != nullptr)
            this->temperature_sensor_->publish_state(data);
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

}
void SY7T609_UART::update() 
{
  if(!m_interrupt)
  {
    this->write_state_(PROCESS_STATE_READ_PF); 
  }
}
uint16_t SY7T609_UART::get_reg_addr(process_state state)
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
        case PROCESS_STATE_READ_REACTIVE_POWER:
            addr = ADDR_VAR;
            break;
        case PROCESS_STATE_READ_EPPCNT:
            addr = ADDR_EPPCNT;
            break;
        case PROCESS_STATE_READ_FREQUENCY:
            addr = ADDR_FREQUENCY;
            break;
        case PROCESS_STATE_READ_TEMPERATURE:
            addr = ADDR_CTEMP;
            break;
        case PROCESS_STATE_WRITE_CLEAR_ENERGY_COUNT:
            addr = ADDR_COMMAND;
            break;
        default:
            break;
    }

    return addr;
}
uint32_t SY7T609_UART::get_reg_writecmd(process_state state)
{
  uint32_t cmd = CMD_NONE;
  switch(state)
  {
    case PROCESS_STATE_WRITE_CLEAR_ENERGY_COUNT:
        cmd = CMD_REG_CLEAR_ENGERGY_COUNTERS;
        break;
    default:
        break;
  }

  return cmd;
}
void SY7T609_UART::write_state_(process_state state) {
  if (state == PROCESS_DONE) 
  {
    this->m_process_state = state;
    return;
  }
  
  //write-register
  if(state >= PROCESS_STATE_WRITE_CLEAR_ENERGY_COUNT)  
  {
    uint16_t addr = get_reg_addr(state);
    if(addr == ADDR_ERROR)
    {
      ESP_LOGE(TAG, "SY7T609_UART process_state[%d],write register failure! ADDR_ERROR", state);
      return;
    }
    uint32_t value = get_reg_writecmd(state);
    if(value == CMD_NONE)
    {
      ESP_LOGE(TAG, "SY7T609_UART process_state[%d],write register failure! CMD_NONE", state);
      return;
    }
    std::array<uint8_t, SSI_UART_WRITE_SEND_PKG_SIZE> data{};
    //header
    data[0] = SSI_HEADER;
    //byte_count
    data[1] = SSI_UART_WRITE_SEND_PKG_SIZE;
    //payload[7]
    data[2] = CMD_SELECT_REGISTER_ADDRESS;
    data[3] = addr & 0xFF;
    data[4] = (addr >> 8) & 0xFF;
    data[5] = CMD_WRITE_RETISTER_3BYTES;
    data[6] = value & 0xFF;
    data[7] = (value >> 8) & 0xFF;
    data[8] = (value >> 16) & 0xFF;
    //checksum
    data[9] = 0;
    uint8_t checksum = 0;
    for (int i = 0; i < SSI_UART_WRITE_SEND_PKG_SIZE - 1; i++)
    {
      checksum += data[i];
    }
    checksum = ~checksum + 1;
    data[9] = checksum;
    ESP_LOGI(TAG, "SY7T609_UART process_state[%d],write register!", state);
    this->write_array(data);

    return;
  }
  
  //read-register
  if(state < PROCESS_STATE_WRITE_CLEAR_ENERGY_COUNT)  
  {
    uint16_t addr = get_reg_addr(state);
    if(ADDR_ERROR == addr)
    {
      ESP_LOGE(TAG, "SY7T609_UART invalid process_state[%d]!", state);
      return ;
    }
    std::array<uint8_t, SSI_UART_READ_SEND_PKG_SIZE> data{};
    //header
    data[0] = SSI_HEADER;
    //byte_count
    data[1] = SSI_UART_READ_SEND_PKG_SIZE;
    //payload[4]
    data[2] = CMD_SELECT_REGISTER_ADDRESS;
    data[3] = addr & 0xFF;;
    data[4] = (addr >> 8) & 0xFF;
    data[5] = CMD_READ_REGITSTER_3BYTES;
    //checksum
    data[6] = 0;
    uint8_t checksum = 0;
    for (int i = 0; i < SSI_UART_READ_SEND_PKG_SIZE - 1; i++)
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
    this->m_process_state = state;

    return;
  }

}
void SY7T609_UART::dump_config() {
  ESP_LOGCONFIG(TAG, "SY7T609_UART:");
  LOG_SENSOR("", "PowerFactor", this->power_factor_sensor_);
  LOG_SENSOR("", "Voltage", this->voltage_sensor_);
  LOG_SENSOR("", "Current", this->current_sensor_);
  LOG_SENSOR("", "Power", this->power_sensor_);
  LOG_SENSOR("", "Reactive Power", this->power_reactive_sensor_);
  LOG_SENSOR("", "Energy", this->energy_sensor_);
  LOG_SENSOR("", "Frequency", this->frequency_sensor_);
  LOG_SENSOR("", "Temperature", this->temperature_sensor_);
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
float SY7T609_UART::readVRMS(const std::array<uint8_t,SSI_UART_READ_RECV_PKG_SIZE> &array)
{
  uint32_t data = 0;
  data = ((uint32_t)array[4]<<16) | ((uint32_t)array[3]<<8) | (uint32_t)array[2];
  
  return data/1000.0f;
}

float SY7T609_UART::readIRMS(const std::array<uint8_t,SSI_UART_READ_RECV_PKG_SIZE> &array)
{
  uint32_t data = 0;
  data = ((uint32_t)array[4]<<16) | ((uint32_t)array[3]<<8) | (uint32_t)array[2];
  //data = (uint32_t)(data/128);
  
  return data/1000.0f;
}

float SY7T609_UART::readPower(const std::array<uint8_t,SSI_UART_READ_RECV_PKG_SIZE> &array) 
{
  uint32_t data = 0;
  data = ((uint32_t)array[4]<<16) | ((uint32_t)array[3]<<8) | (uint32_t)array[2];
  if(data >= 0x800000) 
  {
		data = 0x01000000 - data;
	}
  
  return data/1000.0f;
}

float SY7T609_UART::readReactivePower(const std::array<uint8_t,SSI_UART_READ_RECV_PKG_SIZE> &array) 
{
  uint32_t data = 0;
  data = ((uint32_t)array[4]<<16) | ((uint32_t)array[3]<<8) | (uint32_t)array[2];
  if(data >= 0x800000) 
  {
		data = 0x01000000 - data;
	}
  return data/1000.0f;
}

float SY7T609_UART::readPF(const std::array<uint8_t,SSI_UART_READ_RECV_PKG_SIZE> &array)
{
  uint32_t value = 0;
  value = ((uint32_t)array[4]<<16) | ((uint32_t)array[3]<<8) | (uint32_t)array[2];
  if(value >= 0x800000)
  {
    value = 0x01000000 - value;
  }
  return value/1000.0f;
}

float SY7T609_UART::readEnergy(const std::array<uint8_t,SSI_UART_READ_RECV_PKG_SIZE> &array)
{
  uint32_t data = 0;
  data = ((uint32_t)array[4]<<16) | ((uint32_t)array[3]<<8) | (uint32_t)array[2];
  
  return data/1000.0f;;
}

float SY7T609_UART::readFrequency(const std::array<uint8_t,SSI_UART_READ_RECV_PKG_SIZE> &array)
{
  uint32_t data = 0;
  data = ((uint32_t)array[4]<<16) | ((uint32_t)array[3]<<8) | (uint32_t)array[2];
  
  return data/1000.0f;
}

float SY7T609_UART::readTemperature(const std::array<uint8_t,SSI_UART_READ_RECV_PKG_SIZE> &array)
{
  uint32_t data = 0;
  data = ((uint32_t)array[4]<<16) | ((uint32_t)array[3]<<8) | (uint32_t)array[2];
  
  return data/1000.0f;
}

void SY7T609_UART::reset_energy_() 
{
  ESP_LOGI(TAG, "SY7T609_UART reset energy counter, begin...");

  uint16_t addr = ADDR_COMMAND;
  uint32_t value = CMD_REG_CLEAR_ENGERGY_COUNTERS;
  m_interrupt = true;
  //wait register-read finished
  while(m_process_state != PROCESS_DONE)
  {
     ;;
  }
  write_state_(PROCESS_STATE_WRITE_CLEAR_ENERGY_COUNT);
  ESP_LOGI(TAG, "SY7T609_UART reset energy counter,end.");
}
}  // namespace pzem004t
}  // namespace esphome
