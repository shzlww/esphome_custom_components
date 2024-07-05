#pragma once

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/datatypes.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "sy7t609_def.h"
namespace esphome
{
  namespace sy7t609
  {

    template <typename... Ts>
    class ResetEnergyAction;
    template <typename... Ts>
    class ResetCalibrationAction;
    template <typename... Ts>
    class PrintDebugMsgAction;
    class SY7T609_UART;
    typedef void (SY7T609_UART::*ActionCallbackFuncPtr)(void);

    class SY7T609_UART : public PollingComponent, public uart::UARTDevice
    {
    public:
      // SY7T609_UART(UARTComponent *parent) : UARTDevice(parent);

      void set_power_factor_sensor(sensor::Sensor *power_factor_sensor) { power_factor_sensor_ = power_factor_sensor; }

      void set_voltage_sensor(sensor::Sensor *voltage_sensor) { voltage_sensor_ = voltage_sensor; }

      void set_current_sensor(sensor::Sensor *current_sensor) { current_sensor_ = current_sensor; }

      void set_power_sensor(sensor::Sensor *power_sensor) { power_sensor_ = power_sensor; }

      void set_power_reactive_sensor(sensor::Sensor *power_reactive_sensor) { power_reactive_sensor_ = power_reactive_sensor; }

      void set_energy_sensor(sensor::Sensor *energy_sensor) { energy_sensor_ = energy_sensor; }

      void set_frequency_sensor(sensor::Sensor *frequency_sensor) { frequency_sensor_ = frequency_sensor; }

      void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }

      void setup() override;

      // void loop() override;

      void update() override;

      void dump_config() override;

    protected:
      void reset_energy_();

      void reset_calibration_();

      void print_debug_msg_();

      void recvPacket();

      void debugPacket(uint8_t header);

      void printRegisterValue();

      void write_state_(process_state state);

      uint16_t getRegisterAddrByState(process_state state);

      uint32_t getRegisterWriteValueByState(process_state state);

      std::string getProcessNameByState(process_state state);

      void uartSendReadCmd(uint16_t addr);

      void uartSendWriteCmd(uint16_t addr, uint32_t value);

      bool isReadRegisterProcess(process_state state);

      bool isWriteRegisterProcess(process_state state);

      bool isReadCalibrationRegisterProcess(process_state state);

      float readPF(const std::array<uint8_t, SSI_UART_READ_RECV_PKG_SIZE> &array);

      float readVRMS(const std::array<uint8_t, SSI_UART_READ_RECV_PKG_SIZE> &array);

      float readIRMS(const std::array<uint8_t, SSI_UART_READ_RECV_PKG_SIZE> &array);

      float readPower(const std::array<uint8_t, SSI_UART_READ_RECV_PKG_SIZE> &array);

      float readReactivePower(const std::array<uint8_t, SSI_UART_READ_RECV_PKG_SIZE> &array);

      float readEnergy(const std::array<uint8_t, SSI_UART_READ_RECV_PKG_SIZE> &array);

      float readFrequency(const std::array<uint8_t, SSI_UART_READ_RECV_PKG_SIZE> &array);

      float readTemperature(const std::array<uint8_t, SSI_UART_READ_RECV_PKG_SIZE> &array);

      template <std::size_t SIZE>
      uint8_t checksum(const std::array<uint8_t, SIZE> &array)
      {
        uint8_t sum = 0;
        for (int i = 0; i < array.max_size() - 1; i++)
        {
          sum += array[i];
        }
        sum = ~sum + 1;
        return sum;
      }

      
      template <std::size_t SIZE>
      uint8_t checksumWithHeader(const uint8_t header, const std::array<uint8_t, SIZE> &array)
      {
        uint8_t sum = header;
        for (int i = 0; i < array.max_size() - 1; i++)
        {
          sum += array[i];
        }
        sum = ~sum + 1;
        return sum;
      }

      int addActionCallBack(ActionCallbackFuncPtr ptrFunc);
      void handleActionCallback();
      bool isNeedHandleActionCallback() { return (m_vecActionCallback.size() > 0); }

    protected:
      process_state m_process_state{PROCESS_DONE};
      uint32_t last_read_{0};
      template <typename... Ts>
      friend class ResetEnergyAction;
      template <typename... Ts>
      friend class ResetCalibrationAction;
      template <typename... Ts>
      friend class PrintDebugMsgAction;
      sensor::Sensor *power_factor_sensor_{nullptr};
      sensor::Sensor *voltage_sensor_{nullptr};
      sensor::Sensor *current_sensor_{nullptr};
      sensor::Sensor *power_sensor_{nullptr};
      sensor::Sensor *power_reactive_sensor_{nullptr};
      sensor::Sensor *energy_sensor_{nullptr};
      sensor::Sensor *frequency_sensor_{nullptr};
      sensor::Sensor *temperature_sensor_{nullptr};

    private:
      std::vector<ActionCallbackFuncPtr> m_vecActionCallback{};
    };

    template <typename... Ts>
    class ResetEnergyAction : public Action<Ts...>
    {
    public:
      ResetEnergyAction(SY7T609_UART *sy7t609) : sy7t609_(sy7t609) {}

      void play(Ts... x) override
      {
        this->sy7t609_->addActionCallBack(&SY7T609_UART::reset_energy_);
        // ESP_LOGI("ResetEnergyAction", "SY7T609_UART addActionCallBack,[reset_energy_]");
      }

    protected:
      SY7T609_UART *sy7t609_;
    };

    template <typename... Ts>
    class ResetCalibrationAction : public Action<Ts...>
    {
    public:
      ResetCalibrationAction(SY7T609_UART *sy7t609) : sy7t609_(sy7t609) {}

      void play(Ts... x) override
      {
        this->sy7t609_->addActionCallBack(&SY7T609_UART::reset_calibration_);
        // ESP_LOGI("ResetCalibrationAction", "SY7T609_UART addActionCallBack,[reset_calibration_]");
      }

    protected:
      SY7T609_UART *sy7t609_;
    };

    template <typename... Ts>
    class PrintDebugMsgAction : public Action<Ts...>
    {
    public:
      PrintDebugMsgAction(SY7T609_UART *sy7t609) : sy7t609_(sy7t609) {}

      void play(Ts... x) override
      {
        this->sy7t609_->addActionCallBack(&SY7T609_UART::print_debug_msg_);
        // ESP_LOGI("PrintDebugMsgAction", "SY7T609_UART addActionCallBack,[print_debug_msg_]");
      }

    protected:
      SY7T609_UART *sy7t609_;
    };
  } // namespace sy7t609
} // namespace esphome
