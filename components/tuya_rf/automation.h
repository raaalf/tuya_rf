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

  void play(Ts... x) override {
    auto data = this->data_.value(x...);
    this->tuya_rf_->queue_transmit(data);
  }

 protected:
  TuyaRfComponent *tuya_rf_;
};

}  // namespace tuya_rf
}  // namespace esphome
