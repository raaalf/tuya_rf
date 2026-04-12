#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "tuya_rf.h"

namespace esphome {
namespace tuya_rf {

template<typename... Ts> class TurnOffReceiverAction : public Action<Ts...> {
 public:
  TurnOffReceiverAction(TuyaRfComponent *tuya_rf) : tuya_rf_(tuya_rf) {}

  void play(Ts... x) override { this->tuya_rf_->turn_off_receiver(); }

 protected:
  TuyaRfComponent *tuya_rf_;
};

template<typename... Ts> class TurnOnReceiverAction : public Action<Ts...> {
 public:
  TurnOnReceiverAction(TuyaRfComponent *tuya_rf) : tuya_rf_(tuya_rf) {}

  void play(Ts... x) override { this->tuya_rf_->turn_on_receiver(); }

 protected:
  TuyaRfComponent *tuya_rf_;
};

template<typename... Ts> class QueueTransmitAction : public Action<Ts...> {
 public:
  QueueTransmitAction(TuyaRfComponent *tuya_rf) : tuya_rf_(tuya_rf) {}

  TEMPLATABLE_VALUE(std::vector<int32_t>, data)
  void set_frequency(uint16_t frequency_mhz) { this->frequency_mhz_ = frequency_mhz; }

  // Explicit overload for brace-enclosed initializer lists.
  // ESPHome codegen emits set_data({625, -593, ...}) for static data,
  // but C++ cannot deduce template parameter V from an initializer_list.
  void set_data(std::initializer_list<int32_t> list) { this->data_ = std::vector<int32_t>(list); }

  void play(Ts... x) override {
    auto data = this->data_.value(x...);
    this->tuya_rf_->queue_transmit(data, this->frequency_mhz_);
  }

 protected:
  TuyaRfComponent *tuya_rf_;
  uint16_t frequency_mhz_{0};
};

}  // namespace tuya_rf
}  // namespace esphome
