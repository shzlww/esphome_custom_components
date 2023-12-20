#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "sy7t609_def.h"
namespace esphome {
namespace sy7t609 {

class SY7T609_UART : public PollingComponent, public uart::UARTDevice {
 public:
  void set_power_factor_sensor(sensor::Sensor *power_factor_sensor) { power_factor_sensor_ = power_factor_sensor; }
  void set_voltage_sensor(sensor::Sensor *voltage_sensor) { voltage_sensor_ = voltage_sensor; }
  void set_current_sensor(sensor::Sensor *current_sensor) { current_sensor_ = current_sensor; }
  void set_power_sensor(sensor::Sensor *power_sensor) { power_sensor_ = power_sensor; }
  void set_energy_sensor(sensor::Sensor *energy_sensor) { energy_sensor_ = energy_sensor; }
  void set_power_avg_sensor(sensor::Sensor *power_avg_sensor) { power_avg_sensor_ = power_avg_sensor; }
  void set_energy_positive_active_sensor(sensor::Sensor *energy_positive_active_sensor) { energy_positive_active_sensor_ = energy_positive_active_sensor; }
  void set_energy_negative_active_sensor(sensor::Sensor *energy_negative_active_sensor) { energy_negative_active_sensor_ = energy_negative_active_sensor; }
  void set_frequency_sensor(sensor::Sensor *frequency_sensor) { frequency_sensor_ = frequency_sensor; }
 
  void setup() override;

  void loop() override;

  void update() override;

  void dump_config() override;

 protected:
  sensor::Sensor *power_factor_sensor_{nullptr};
  sensor::Sensor *voltage_sensor_{nullptr};
  sensor::Sensor *current_sensor_{nullptr};
  sensor::Sensor *power_sensor_{nullptr};
  sensor::Sensor *power_avg_sensor_{nullptr};
  sensor::Sensor *energy_sensor_{nullptr};
  sensor::Sensor *energy_positive_active_sensor_{nullptr};
  sensor::Sensor *energy_negative_active_sensor_{nullptr};
  sensor::Sensor *frequency_sensor_{nullptr};

  enum process_state {
      PROCESS_STATE_READ_PF = 0,
      PROCESS_STATE_READ_VRMS,
      PROCESS_STATE_READ_IRMS,
      PROCESS_STATE_READ_POWER,
      PROCESS_STATE_READ_AVG_POWER,
      PROCESS_STATE_READ_EPPCNT,
      PROCESS_STATE_READ_EPMCNT,
      PROCESS_STATE_READ_FREQUENCY,
      PROCESS_STATE_DELAY_1,
      PROCESS_STATE_DELAY_2,
      PROCESS_STATE_UPDATE_INFO,
      PROCESS_DONE
  } read_state_{PROCESS_STATE_READ_PF};

  void write_state_(process_state state);

  uint16_t get_reg_addr(process_state state);
  
  float readPF(const std::array<uint8_t,SSI_UART_RECV_PKG_SIZE> &array) ;

  float readVRMS(const std::array<uint8_t,SSI_UART_RECV_PKG_SIZE> &array) ;

  float readIRMS(const std::array<uint8_t,SSI_UART_RECV_PKG_SIZE> &array) ;

  float readPower(const std::array<uint8_t,SSI_UART_RECV_PKG_SIZE> &array) ;

  float readAvgPower(const std::array<uint8_t,SSI_UART_RECV_PKG_SIZE> &array) ;

  float readPositiveActiveEnergy(const std::array<uint8_t,SSI_UART_RECV_PKG_SIZE> &array) ;

  float readNegativeActiveEnergy(const std::array<uint8_t,SSI_UART_RECV_PKG_SIZE> &array) ;
  
  float readFrequency(const std::array<uint8_t,SSI_UART_RECV_PKG_SIZE> &array);
  uint32_t last_read_{0};

};

}  // namespace pzem004t
}  // namespace esphome
