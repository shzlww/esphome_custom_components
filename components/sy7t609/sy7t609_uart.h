#pragma once

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "sy7t609_def.h"
namespace esphome {
namespace sy7t609 {

template<typename... Ts> class ResetEnergyAction;

class SY7T609_UART : public PollingComponent, public uart::UARTDevice {
 public:
  void set_power_factor_sensor(sensor::Sensor *power_factor_sensor) { power_factor_sensor_ = power_factor_sensor; }
  void set_voltage_sensor(sensor::Sensor *voltage_sensor) { voltage_sensor_ = voltage_sensor; }
  void set_current_sensor(sensor::Sensor *current_sensor) { current_sensor_ = current_sensor; }
  void set_power_sensor(sensor::Sensor *power_sensor) { power_sensor_ = power_sensor; }
  void set_power_reactive_sensor(sensor::Sensor *power_reactive_sensor) { power_reactive_sensor_ = power_reactive_sensor; }
  void set_energy_sensor(sensor::Sensor *energy_sensor) { energy_sensor_ = energy_sensor; }
  void set_frequency_sensor(sensor::Sensor *frequency_sensor) { frequency_sensor_ = frequency_sensor; }
  void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }
  void setup() override;

  void loop() override;

  void update() override;

  void dump_config() override;


 protected:
  template<typename... Ts> friend class ResetEnergyAction;
  sensor::Sensor *power_factor_sensor_{nullptr};
  sensor::Sensor *voltage_sensor_{nullptr};
  sensor::Sensor *current_sensor_{nullptr};
  sensor::Sensor *power_sensor_{nullptr};
  sensor::Sensor *power_reactive_sensor_{nullptr};
  sensor::Sensor *energy_sensor_{nullptr};
  sensor::Sensor *frequency_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};
  
  void reset_energy_();
  
  void write_state_(process_state state);

  uint16_t get_reg_addr(process_state state);
  uint32_t get_reg_writecmd(process_state state);
  
  float readPF(const std::array<uint8_t,SSI_UART_READ_RECV_PKG_SIZE> &array) ;

  float readVRMS(const std::array<uint8_t,SSI_UART_READ_RECV_PKG_SIZE> &array) ;

  float readIRMS(const std::array<uint8_t,SSI_UART_READ_RECV_PKG_SIZE> &array) ;

  float readPower(const std::array<uint8_t,SSI_UART_READ_RECV_PKG_SIZE> &array) ;

  float readReactivePower(const std::array<uint8_t,SSI_UART_READ_RECV_PKG_SIZE> &array) ;

  float readEnergy(const std::array<uint8_t,SSI_UART_READ_RECV_PKG_SIZE> &array) ;

  float readFrequency(const std::array<uint8_t,SSI_UART_READ_RECV_PKG_SIZE> &array);
  
  float readTemperature(const std::array<uint8_t,SSI_UART_READ_RECV_PKG_SIZE> &array);
  
  process_state m_process_state{PROCESS_DONE};
  uint32_t last_read_{0};

  bool m_interrupt{false};

};

template<typename... Ts> class ResetEnergyAction : public Action<Ts...> {
 public:
  ResetEnergyAction(SY7T609_UART *sy7t609) : sy7t609_(sy7t609) {}

  void play(Ts... x) override { this->sy7t609_->reset_energy_(); }

 protected:
  SY7T609_UART *sy7t609_;
};

}  // namespace pzem004t
}  // namespace esphome
