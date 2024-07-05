#include "sy7t609_uart.h"
#include "esphome/core/log.h"
#include <cinttypes>

namespace esphome
{
  namespace sy7t609
  {

    static const char *const TAG = "sy7t609";

    void SY7T609_UART::setup()
    {
      this->uartSendWriteCmd(ADDR_COMMAND, CMD_REG_SOFT_RESET);
      this->uartSendWriteCmd(ADDR_ACCUM_CYC, 0);
      this->uartSendWriteCmd(ADDR_ACCUM, 6702);
      this->uartSendWriteCmd(ADDR_CONTROL, CONTROL_REGISTER_BIT_SET);
      // this->parent_->on_data([this](const uint8_t *data, size_t len)
      //                        { this->on_uart_data(data, len); });
      // Clear UART buffer
      while (this->available())
      {
        this->read();
        delay(0);
      }
    }

    inline float parseU24(const uint8_t *array)
    {
      return float(((uint32_t)array[2] << 16) | ((uint32_t)array[1] << 8) | (uint32_t)array[0]);
    }

    inline int32_t parseS24(const uint8_t *array)
    {
      int32_t data = ((uint32_t)array[2] << 16) | ((uint32_t)array[1] << 8) | (uint32_t)array[0];
      if (data >= 0x800000)
        return 0x01000000 - data;
      return float(data);
    }
#define parseReportTemperature(array) ((parseU24(&array[0 * 3+1])) / 1000.0f)
#define parseReportFrequency(array) ((parseU24(&array[1 * 3+1])) / 1000.0f)
#define parseReportVavg(array) ((parseS24(&array[2 * 3+1])) / 1000.0f)
#define parseReportIavg(array) ((parseS24(&array[3 * 3+1])) / 1000.0f)
// TODO avgpower at cmd 0x045 ?
// might be added with application note
#define parseReportVrms(array) ((parseU24(&array[4 * 3+1])) / 1000.0f)
#define parseReportIrms(array) ((parseU24(&array[5 * 3+1])) / 1000.0f)
#define parseReportPwr(array) ((parseS24(&array[6 * 3+1])) / 1000.0f)
#define parseReportVAR(array) ((parseS24(&array[7 * 3+1])) / 1000.0f)
#define parseReportVA(array) ((parseS24(&array[8 * 3+1])) / 1000.0f)
#define parseReportPF(array) ((parseS24(&array[9 * 3+1])) / 1000.0f)
// Read From Register
#define parseEnergy(array) ((parseS24(&array[0 * 3])) / 100000.0f)

    void SY7T609_UART::update()
    {
      this->uartSendReadCmd(ADDR_EPPCNT);
      while (this->available())
      {
        this->recvPacket();
        delay(0);
      }
    }

    void SY7T609_UART::recvPacket()
    {
      uint8_t header = this->read();
      // ESP_LOGI(TAG, "Debug: Header: 0x%02x", header);
      if (header == REPLY_ACK_WITH_DATA)
      {
        auto resp = *this->read_array<SSI_UART_READ_RECV_PKG_SIZE>();
        uint8_t sum = checksumWithHeader<SSI_UART_READ_RECV_PKG_SIZE>(header, resp);
        if (sum != resp[resp.max_size() - 1])
        {
          ESP_LOGE(TAG, "Debug: REPLY_ACK_WITH_DATA: invalid checksum! 0x%02X != 0x%02X",
                   sum,
                   resp[resp.max_size() - 1]);
                   return;
        }
          this->energy_sensor_->publish_state(parseEnergy(resp));
      }
      else if (header == REPLY_AUTO_REPORTING_HEADER)
      {
        auto resp = *this->read_array<SSI_UART_AUTO_REPORT_PKG_SIZE>();
        uint8_t sum = checksumWithHeader<SSI_UART_AUTO_REPORT_PKG_SIZE>(header, resp);
        if (sum != resp[resp.max_size() - 1])
        {
          ESP_LOGE(TAG, "Debug: REPLY_AUTO_REPORTING: invalid checksum! 0x%02X != 0x%02X",
                   sum,
                   resp[resp.max_size() - 1]);
        }
        this->temperature_sensor_->publish_state(parseReportTemperature(resp));
        this->frequency_sensor_->publish_state(parseReportFrequency(resp));
        this->voltage_sensor_->publish_state(parseReportVrms(resp));
        this->current_sensor_->publish_state(parseReportIavg(resp));
        this->power_sensor_->publish_state(parseReportPwr(resp));
        this->power_reactive_sensor_->publish_state(parseReportVAR(resp));
        this->power_factor_sensor_->publish_state(parseReportPF(resp));
      }
      else
      {
        ESP_LOGE(TAG, "Debug: invalid packet header 0x%02X", header);
      }
    }

    uint16_t SY7T609_UART::getRegisterAddrByState(process_state state)
    {
      uint16_t addr = ADDR_ERROR;
      switch (state)
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
      case PROCESS_STATE_READ_ADDR_IGAIN:
        addr = ADDR_IGAIN;
        break;
      case PROCESS_STATE_READ_ADDR_VGAIN:
        addr = ADDR_VGAIN;
        break;
      case PROCESS_STATE_READ_ADDR_ISCALE:
        addr = ADDR_ISCALE;
        break;
      case PROCESS_STATE_READ_ADDR_VSCALE:
        addr = ADDR_VSCALE;
        break;
      case PROCESS_STATE_READ_ADDR_PSCALE:
        addr = ADDR_PSCALE;
        break;
      case PROCESS_STATE_READ_ADDR_ACCUM:
        addr = ADDR_ACCUM;
        break;
      case PROCESS_STATE_READ_ADDR_IRMS_TARGET:
        addr = ADDR_IRMS_TARGET;
        break;
      case PROCESS_STATE_READ_ADDR_VRMS_TARGET:
        addr = ADDR_VRMS_TARGET;
        break;
      case PROCESS_STATE_READ_ADDR_POWER_TARGET:
        addr = ADDR_POWER_TARGET;
        break;
      case PROCESS_STATE_READ_ADDR_BAUD:
        addr = ADDR_BAUD_RATE;
        break;
      case PROCESS_STATE_READ_BUCKETL:
        addr = ADDR_BUCKETL;
        break;
      case PROCESS_STATE_READ_BUCKETH:
        addr = ADDR_BUCKETH;
        break;
      case PROCESS_STATE_READ_CONTROL_REGISTER:
        addr = ADDR_CONTROL;
        break;
      case PROCESS_STATE_READ_CALIBRATION_ALL_REGISTER:
        addr = ADDR_COMMAND;
        break;
      case PROCESS_STATE_WRITE_ADDR_IGAIN:
        addr = ADDR_IGAIN;
        break;
      case PROCESS_STATE_WRITE_ADDR_VGAIN:
        addr = ADDR_VGAIN;
        break;
      case PROCESS_STATE_WRITE_ADDR_ISCALE:
        addr = ADDR_ISCALE;
        break;
      case PROCESS_STATE_WRITE_ADDR_VSCALE:
        addr = ADDR_VSCALE;
        break;
      case PROCESS_STATE_WRITE_ADDR_PSCALE:
        addr = ADDR_PSCALE;
        break;
      case PROCESS_STATE_WRITE_ADDR_ACCUM:
        addr = ADDR_ACCUM;
        break;
      case PROCESS_STATE_WRITE_ADDR_IRMS_TARGET:
        addr = ADDR_IRMS_TARGET;
        break;
      case PROCESS_STATE_WRITE_ADDR_VRMS_TARGET:
        addr = ADDR_VRMS_TARGET;
        break;
      case PROCESS_STATE_WRITE_ADDR_POWER_TARGET:
        addr = ADDR_POWER_TARGET;
        break;
      case PROCESS_STATE_WRITE_BUCKETL:
        addr = ADDR_BUCKETL;
        break;
      case PROCESS_STATE_WRITE_BUCKETH:
        addr = ADDR_BUCKETH;
        break;
      case PROCESS_STATE_WRITE_CONTROL_REGISTER:
        addr = ADDR_CONTROL;
        break;
      case PROCESS_STATE_WRITE_CMD_REG_CLEAR_ENGERGY_COUNTERS:
        addr = ADDR_COMMAND;
        break;
      case PROCESS_STATE_WRITE_CMD_REG_SAVE_TO_FLASH:
        addr = ADDR_COMMAND;
        break;
      case PROCESS_STATE_WRITE_CALIBRATION_ALL_REGISTER:
        addr = ADDR_COMMAND;
        break;
      case PROCESS_STATE_WRITE_CMD_REG_SOFT_RESET:
        addr = ADDR_COMMAND;
        break;
      default:
        break;
      }

      return addr;
    }
    uint32_t SY7T609_UART::getRegisterWriteValueByState(process_state state)
    {
      uint32_t data = CMD_NONE;
      switch (state)
      {
      case PROCESS_STATE_WRITE_ADDR_IGAIN:
        data = IGAIN;
        break;
      case PROCESS_STATE_WRITE_ADDR_VGAIN:
        data = VGAIN;
        break;
      case PROCESS_STATE_WRITE_ADDR_ISCALE:
        data = ISCALE;
        break;
      case PROCESS_STATE_WRITE_ADDR_VSCALE:
        data = VSCALE;
        break;
      case PROCESS_STATE_WRITE_ADDR_PSCALE:
        data = PSCALE;
        break;
      case PROCESS_STATE_WRITE_ADDR_ACCUM:
        data = ACCUM;
        break;
      case PROCESS_STATE_WRITE_ADDR_IRMS_TARGET:
        data = IRMS_TARGET;
        break;
      case PROCESS_STATE_WRITE_ADDR_VRMS_TARGET:
        data = VRMS_TARGET;
        break;
      case PROCESS_STATE_WRITE_ADDR_POWER_TARGET:
        data = POWER_TARGET;
        break;
      case PROCESS_STATE_WRITE_BUCKETL:
        data = BUCKETL;
        break;
      case PROCESS_STATE_WRITE_BUCKETH:
        data = BUCKETH;
        break;
      case PROCESS_STATE_WRITE_CONTROL_REGISTER:
        data = CONTROL;
        break;
      case PROCESS_STATE_WRITE_CMD_REG_CLEAR_ENGERGY_COUNTERS:
        data = CMD_REG_CLEAR_ENGERGY_COUNTERS;
        break;
      case PROCESS_STATE_WRITE_CMD_REG_SAVE_TO_FLASH:
        data = CMD_REG_SAVE_TO_FLASH;
        break;
      case PROCESS_STATE_WRITE_CALIBRATION_ALL_REGISTER:
        data = CMD_REG_CALIBRATION_ALL;
        break;
      case PROCESS_STATE_WRITE_CMD_REG_SOFT_RESET:
        data = CMD_REG_SOFT_RESET;
        break;
      default:
        break;
      }

      return data;
    }

    std::string SY7T609_UART::getProcessNameByState(process_state state)
    {
      std::string value{};
      switch (state)
      {
      case PROCESS_STATE_READ_PF:
        value = "READ_PF";
        break;
      case PROCESS_STATE_READ_VRMS:
        value = "READ_VRMS";
        break;
      case PROCESS_STATE_READ_IRMS:
        value = "READ_IRMS";
        break;
      case PROCESS_STATE_READ_POWER:
        value = "READ_POWER";
        break;
      case PROCESS_STATE_READ_REACTIVE_POWER:
        value = "READ_VAR";
        break;
      case PROCESS_STATE_READ_EPPCNT:
        value = "READ_EPPCNT";
        break;
      case PROCESS_STATE_READ_FREQUENCY:
        value = "READ_FREQUENCY";
        break;
      case PROCESS_STATE_READ_TEMPERATURE:
        value = "READ_CTEMP";
        break;
      case PROCESS_STATE_READ_ADDR_IGAIN:
        value = "READ_IGAIN";
        break;
      case PROCESS_STATE_READ_ADDR_VGAIN:
        value = "READ_VGAIN";
        break;
      case PROCESS_STATE_READ_ADDR_ISCALE:
        value = "READ_ISCALE";
        break;
      case PROCESS_STATE_READ_ADDR_VSCALE:
        value = "READ_VSCALE";
        break;
      case PROCESS_STATE_READ_ADDR_PSCALE:
        value = "READ_PSCALE";
        break;
      case PROCESS_STATE_READ_ADDR_ACCUM:
        value = "READ_ACCUM";
        break;
      case PROCESS_STATE_READ_ADDR_IRMS_TARGET:
        value = "READ_IRMS_TARGET";
        break;
      case PROCESS_STATE_READ_ADDR_VRMS_TARGET:
        value = "READ_VRMS_TARGET";
        break;
      case PROCESS_STATE_READ_ADDR_POWER_TARGET:
        value = "READ_POWER_TARGET";
        break;
      case PROCESS_STATE_READ_ADDR_BAUD:
        value = "READ_BAUD_RATE";
        break;
      case PROCESS_STATE_READ_BUCKETL:
        value = "READ_BUCKETL";
        break;
      case PROCESS_STATE_READ_BUCKETH:
        value = "READ_BUCKETH";
        break;
      case PROCESS_STATE_READ_CONTROL_REGISTER:
        value = "READ_CONTROL";
        break;
      case PROCESS_STATE_READ_CALIBRATION_ALL_REGISTER:
        value = "READ_COMMAND_CALIBRATION";
        break;
      case PROCESS_STATE_WRITE_ADDR_IGAIN:
        value = "WRITE_IGAIN";
        break;
      case PROCESS_STATE_WRITE_ADDR_VGAIN:
        value = "WRITE_VGAIN";
        break;
      case PROCESS_STATE_WRITE_ADDR_ISCALE:
        value = "WRITE_ISCALE";
        break;
      case PROCESS_STATE_WRITE_ADDR_VSCALE:
        value = "WRITE_VSCALE";
        break;
      case PROCESS_STATE_WRITE_ADDR_PSCALE:
        value = "WRITE_PSCALE";
        break;
      case PROCESS_STATE_WRITE_ADDR_ACCUM:
        value = "WRITE_ACCUM";
        break;
      case PROCESS_STATE_WRITE_ADDR_IRMS_TARGET:
        value = "WRITE_IRMS_TARGET";
        break;
      case PROCESS_STATE_WRITE_ADDR_VRMS_TARGET:
        value = "WRITE_VRMS_TARGET";
        break;
      case PROCESS_STATE_WRITE_ADDR_POWER_TARGET:
        value = "WRITE_POWER_TARGET";
        break;
      case PROCESS_STATE_WRITE_BUCKETL:
        value = "WRITE_BUCKETL";
        break;
      case PROCESS_STATE_WRITE_BUCKETH:
        value = "WRITE_BUCKETH";
        break;
      case PROCESS_STATE_WRITE_CONTROL_REGISTER:
        value = "WRITE_CONTROL";
        break;
      case PROCESS_STATE_WRITE_CMD_REG_CLEAR_ENGERGY_COUNTERS:
        value = "WRITE_COMMAND_CLEAR_ENGERGY";
        break;
      case PROCESS_STATE_WRITE_CMD_REG_SAVE_TO_FLASH:
        value = "WRITE_COMMAND_SAVE_TO_FLASH";
        break;
      case PROCESS_STATE_WRITE_CALIBRATION_ALL_REGISTER:
        value = "WRITE_COMMAND_CALIBRATION";
        break;
      case PROCESS_STATE_WRITE_CMD_REG_SOFT_RESET:
        value = "WRITE_COMMAND_SOFT_RESET";
        break;
      default:
        break;
      }
      return value;
    }

    void SY7T609_UART::uartSendReadCmd(uint16_t addr)
    {
      std::array<uint8_t, SSI_UART_READ_SEND_PKG_SIZE> data{};

      data[0] = SSI_HEADER;
      data[1] = SSI_UART_READ_SEND_PKG_SIZE;
      data[2] = CMD_SELECT_REGISTER_ADDRESS;
      data[3] = addr & 0xFF;
      data[4] = (addr >> 8) & 0xFF;
      data[5] = CMD_READ_REGITSTER_3BYTES;
      data[6] = checksum<SSI_UART_READ_SEND_PKG_SIZE>(data);

      this->write_array(data);
      this->flush();
      delay(0);
    }

    void SY7T609_UART::uartSendWriteCmd(uint16_t addr, uint32_t value)
    {
      std::array<uint8_t, SSI_UART_WRITE_SEND_PKG_SIZE> data{};

      data[0] = SSI_HEADER;
      data[1] = SSI_UART_WRITE_SEND_PKG_SIZE;
      data[2] = CMD_SELECT_REGISTER_ADDRESS;
      data[3] = addr & 0xFF;
      data[4] = (addr >> 8) & 0xFF;
      data[5] = CMD_WRITE_RETISTER_3BYTES;
      data[6] = value & 0xFF;
      data[7] = (value >> 8) & 0xFF;
      data[8] = (value >> 16) & 0xFF;
      data[9] = checksum<SSI_UART_WRITE_SEND_PKG_SIZE>(data);

      this->write_array(data);
      this->flush();
      delay(0);
    }

    void SY7T609_UART::write_state_(process_state state)
    {
      if (state == PROCESS_DONE)
      {
        this->m_process_state = state;
        return;
      }
      // write-register
      if (isWriteRegisterProcess(state))
      {
        uint16_t addr = getRegisterAddrByState(state);
        if (addr == ADDR_ERROR)
        {
          ESP_LOGE(TAG, "SY7T609_UART process_state[%s],write register failure! ADDR_ERROR", getProcessNameByState(state).c_str());
          this->m_process_state = PROCESS_DONE;
          return;
        }
        uint32_t value = getRegisterWriteValueByState(state);
        if (value == CMD_NONE)
        {
          ESP_LOGE(TAG, "SY7T609_UART process_state[%s],write register failure! CMD_NONE", getProcessNameByState(state).c_str());
          this->m_process_state = PROCESS_DONE;
          return;
        }

        ESP_LOGW(TAG, "Debug: process_state[%s],Write register,addr:0x[%03x] value:[0x%02x 0x%02x 0x%02x]!",
                 getProcessNameByState(state).c_str(),
                 getRegisterAddrByState(state),
                 (value >> 16) & 0xFF,
                 (value >> 8) & 0xFF,
                 value & 0xFF);

        uartSendWriteCmd(addr, value);
        this->m_process_state = state;
        return;
      }

      // read-register
      if (isReadRegisterProcess(state))
      {
        uint16_t addr = getRegisterAddrByState(state);
        if (ADDR_ERROR == addr)
        {
          ESP_LOGE(TAG, "SY7T609_UART invalid process_state[%d]!", state);
          return;
        }
        uartSendReadCmd(addr);
        this->m_process_state = state;
        return;
      }
      if (isReadCalibrationRegisterProcess(state))
      {
        uint16_t addr = getRegisterAddrByState(state);
        if (ADDR_ERROR == addr)
        {
          ESP_LOGE(TAG, "SY7T609_UART invalid process_state[%d]!", state);
          return;
        }
        uartSendReadCmd(addr);
        this->m_process_state = state;
        return;
      }
    }
    void SY7T609_UART::dump_config()
    {
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

    float SY7T609_UART::readVRMS(const std::array<uint8_t, SSI_UART_READ_RECV_PKG_SIZE> &array)
    {
      uint32_t data = 0;
      data = ((uint32_t)array[4] << 16) | ((uint32_t)array[3] << 8) | (uint32_t)array[2];

      return data / 1000.0f;
    }

    float SY7T609_UART::readIRMS(const std::array<uint8_t, SSI_UART_READ_RECV_PKG_SIZE> &array)
    {
      uint32_t data = 0;
      data = ((uint32_t)array[4] << 16) | ((uint32_t)array[3] << 8) | (uint32_t)array[2];
      // data = (uint32_t)(data/128);

      return data / 1000.0f;
    }

    float SY7T609_UART::readPower(const std::array<uint8_t, SSI_UART_READ_RECV_PKG_SIZE> &array)
    {
      uint32_t data = 0;
      data = ((int32_t)array[4] << 16) | ((int32_t)array[3] << 8) | (uint32_t)array[2];
      if (data >= 0x800000)
      {
        data = 0x01000000 - data;
      }

      return data / 1000.0f;
    }

    float SY7T609_UART::readReactivePower(const std::array<uint8_t, SSI_UART_READ_RECV_PKG_SIZE> &array)
    {
      uint32_t data = 0;
      data = ((uint32_t)array[4] << 16) | ((uint32_t)array[3] << 8) | (uint32_t)array[2];
      if (data >= 0x800000)
      {
        data = 0x01000000 - data;
      }
      return data / 1000.0f;
    }

    float SY7T609_UART::readPF(const std::array<uint8_t, SSI_UART_READ_RECV_PKG_SIZE> &array)
    {
      uint32_t data = 0;
      data = ((uint32_t)array[4] << 16) | ((uint32_t)array[3] << 8) | (uint32_t)array[2];
      if (data >= 0x800000)
      {
        data = 0x01000000 - data;
      }
      return data / 1000.0f;
    }

    float SY7T609_UART::readEnergy(const std::array<uint8_t, SSI_UART_READ_RECV_PKG_SIZE> &array)
    {
      uint32_t data = 0;
      data = ((uint32_t)array[4] << 16) | ((uint32_t)array[3] << 8) | (uint32_t)array[2];

      return data / 1.0f;
    }
    float SY7T609_UART::readFrequency(const std::array<uint8_t, SSI_UART_READ_RECV_PKG_SIZE> &array)
    {
      uint32_t data = 0;
      data = ((uint32_t)array[4] << 16) | ((uint32_t)array[3] << 8) | (uint32_t)array[2];

      return data / 1000.0f;
    }

    float SY7T609_UART::readTemperature(const std::array<uint8_t, SSI_UART_READ_RECV_PKG_SIZE> &array)
    {
      uint32_t data = 0;
      data = ((uint32_t)array[4] << 16) | ((uint32_t)array[3] << 8) | (uint32_t)array[2];

      return data / 1000.0f;
    }

    int SY7T609_UART::addActionCallBack(ActionCallbackFuncPtr ptrFunc)
    {
      m_vecActionCallback.push_back(ptrFunc);
      return m_vecActionCallback.size();
    }

    void SY7T609_UART::handleActionCallback()
    {
      if (m_vecActionCallback.size() == 0)
      {
        return;
      }
      ActionCallbackFuncPtr ptrFunc = nullptr;
      for (int i = 0; i < m_vecActionCallback.size(); i++)
      {
        ptrFunc = m_vecActionCallback[i];
        if (ptrFunc)
        {
          ESP_LOGI(TAG, "SY7T609_UART Sy7t609 handleActionCallback[%d]...", i);
          (this->*ptrFunc)();
        }
        delay(0);
      }

      while (this->available())
      {
        this->read();
        delay(0);
      }

      m_vecActionCallback.clear();
      if (m_process_state != PROCESS_DONE)
      {
        write_state_(PROCESS_DONE);
      }
    }

    bool SY7T609_UART::isReadRegisterProcess(process_state state)
    {
      if (PROCESS_STATE_READ_REGISTER_MIN < state && state < PROCESS_STATE_READ_REGISTER_MAX)
      {
        return true;
      }
      else
      {
        return false;
      }
    }

    bool SY7T609_UART::isWriteRegisterProcess(process_state state)
    {
      if (PROCESS_STATE_WRITE_REGISTER_MIN < state && state < PROCESS_STATE_WRITE_REGISTER_MAX)
      {
        return true;
      }
      else
      {
        return false;
      }
    }

    bool SY7T609_UART::isReadCalibrationRegisterProcess(process_state state)
    {
      if (state == PROCESS_STATE_READ_CALIBRATION_ALL_REGISTER)
      {
        return true;
      }
      else
      {
        return false;
      }
    }

    void SY7T609_UART::reset_energy_()
    {
      ESP_LOGI(TAG, "SY7T609_UART Sy7t609 RESET energy, begin...");
      write_state_(PROCESS_STATE_WRITE_CMD_REG_CLEAR_ENGERGY_COUNTERS);
      delay(20);
      ESP_LOGI(TAG, "SY7T609_UART Sy7t609 RESET energy, end.");
    }

    void SY7T609_UART::reset_calibration_()
    {
      ESP_LOGI(TAG, "SY7T609_UART Sy7t609 RESET calibration, begin...");

      for (int state = PROCESS_STATE_WRITE_ADDR_IGAIN; state < PROCESS_STATE_WRITE_CMD_REG_SAVE_TO_FLASH; state++)
      {
        write_state_((process_state)state);
        delay(20);
      }
      write_state_(PROCESS_STATE_WRITE_CMD_REG_SAVE_TO_FLASH);
      delay(20);
      ESP_LOGI(TAG, "SY7T609_UART Sy7t609 RESET calibration, end.");
    }

    void SY7T609_UART::print_debug_msg_()
    {
      ESP_LOGI(TAG, "SY7T609_UART Sy7t609 PRINT debug message, begin...");
      for (int state = PROCESS_STATE_READ_REGISTER_MIN + 1; state < PROCESS_STATE_READ_REGISTER_MAX; state++)
      {
        while (this->available())
        {
          this->read();
        }
        // write_state_((process_state)state);
        delay(20);
        printRegisterValue();
      }
      ESP_LOGI(TAG, "SY7T609_UART Sy7t609 PRINT debug message, end.");
    }

    void SY7T609_UART::debugPacket(uint8_t header)
    {
      if (header == REPLY_ACK_WITH_DATA)
      {
        ESP_LOGI(TAG, "Debug: REPLY_ACK_WITH_DATA");
        auto resp = *this->read_array<SSI_UART_READ_RECV_PKG_SIZE>();
        uint8_t sum = checksumWithHeader<SSI_UART_READ_RECV_PKG_SIZE>(header, resp);
        if (sum != resp[resp.max_size() - 1])
        {
          ESP_LOGE(TAG, "Debug: REPLY_ACK_WITH_DATA: invalid checksum! 0x%02X != 0x%02X",
                   sum,
                   resp[resp.max_size() - 1]);
        }
        ESP_LOGW(TAG, "Debug: REPLY_ACK_WITH_DATA: Read register,addr:0x[%03x] value:[0x%02x 0x%02x 0x%02x]!",
                 resp[1],
                 resp[4],
                 resp[3],
                 resp[2]);
      }
      else if (header == REPLY_AUTO_REPORTING_HEADER)
      {
        unsigned int size = this->read();
        ESP_LOGI(TAG, "Debug: REPLY_AUTO_REPORTING, size=0x%02x", size);
        auto resp = *this->read_array<SSI_UART_AUTO_REPORT_PKG_SIZE>();
        uint8_t sum = checksumWithHeader<SSI_UART_AUTO_REPORT_PKG_SIZE>(header, resp);
        if (sum != resp[resp.max_size() - 1])
        {
          ESP_LOGE(TAG, "Debug: REPLY_AUTO_REPORTING: invalid checksum! 0x%02X != 0x%02X",
                   sum,
                   resp[resp.max_size() - 1]);
        }
        ESP_LOGW(TAG, "Debug: ===REPLY_AUTO_REPORTING===");
        ESP_LOGW(TAG, "Temperature: %.3f", (parseReportTemperature(resp)));
        ESP_LOGW(TAG, "Frequency: %.3f", (parseReportFrequency(resp)));
        ESP_LOGW(TAG, "Vavg: %.3f", (parseReportVavg(resp)));
        ESP_LOGW(TAG, "Iavg: %.3f", (parseReportIavg(resp)));
        ESP_LOGW(TAG, "Vrms: %.3f", (parseReportVrms(resp)));
        ESP_LOGW(TAG, "Irms: %.3f", (parseReportIrms(resp)));
        ESP_LOGW(TAG, "Pwr: %.3f", (parseReportPwr(resp)));
        ESP_LOGW(TAG, "VAR: %.3f", (parseReportVAR(resp)));
        ESP_LOGW(TAG, "VA: %.3f", (parseReportVA(resp)));
        ESP_LOGW(TAG, "PF: %.3f", (parseReportPF(resp)));
      }
      else
      {
        ESP_LOGE(TAG, "Debug: invalid packet header 0x%02X", header);
      }
    }

    void SY7T609_UART::printRegisterValue()
    {
      if (!this->available())
      {
        ESP_LOGW(TAG, "Debug: no data in buffer.");
        return;
      }
      this->debugPacket(this->read());
    }
  } // namespace sy7t906
} // namespace esphome
