#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "sy7t609_def.h"
namespace esphome {
namespace sy7t609 {

class SY7T609_UART : public PollingComponent, public uart::UARTDevice {
 public:
  void set_voltage_sensor(sensor::Sensor *voltage_sensor) { voltage_sensor_ = voltage_sensor; }
  void set_current_sensor(sensor::Sensor *current_sensor) { current_sensor_ = current_sensor; }
  void set_power_sensor(sensor::Sensor *power_sensor) { power_sensor_ = power_sensor; }
  void set_energy_sensor(sensor::Sensor *energy_sensor) { energy_sensor_ = energy_sensor; }

  void setup() override;

  void loop() override;

  void update() override;

  void dump_config() override;

 protected:
  sensor::Sensor *voltage_sensor_{nullptr};
  sensor::Sensor *current_sensor_{nullptr};
  sensor::Sensor *power_sensor_{nullptr};
  sensor::Sensor *energy_sensor_{nullptr};

  enum process_state {
      PROCESS_STATE_READ_PF = 0,
      PROCESS_STATE_READ_VRMS,
      PROCESS_STATE_READ_IRMS,
      PROCESS_STATE_READ_POWER,
      PROCESS_STATE_READ_AVG_POWER,
      PROCESS_STATE_READ_EPPCNT,
      PROCESS_STATE_READ_EPMCNT,
      PROCESS_STATE_DELAY_1,
      PROCESS_STATE_DELAY_2,
      PROCESS_STATE_UPDATE_INFO,
      PROCESS_DONE
  } process_state_t{PROCESS_STATE_READ_PF};

  void write_state_(process_state state);

  uint16_t get_reg_addr(process_state state);
  float readVRMS(std::array<uint8_t> *array);
  float readPOWER(std::array<uint8_t> *array);
  uint32_t last_read_{0};
  process_state_t read_state_;
};

}  // namespace pzem004t
}  // namespace esphome
