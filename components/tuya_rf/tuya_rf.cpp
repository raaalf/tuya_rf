#include "tuya_rf.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome {
namespace tuya_rf {

static const char *const TAG = "tuya_rf";

bool TuyaRfComponent::queue_transmit(const std::vector<int32_t> &data, uint16_t frequency_mhz) {
  if (frequency_mhz == 0) {
    frequency_mhz = this->frequency_mhz_;
  }
  if (frequency_mhz != 315 && frequency_mhz != 433 && frequency_mhz != 868) {
    ESP_LOGW(TAG, "Unsupported transmit frequency: %u MHz", frequency_mhz);
    return false;
  }

  // Check if queue is full
  if (this->transmit_queue_.size() >= this->queue_max_size_) {
    ESP_LOGW(TAG, "Transmit queue full (size: %u), dropping command", this->transmit_queue_.size());
    return false;
  }

  // Add to queue
  TransmitQueueItem item;
  item.data = data;
  item.queued_at = millis();
  item.frequency_mhz = frequency_mhz;
  this->transmit_queue_.push_back(item);

  ESP_LOGD(TAG, "Command queued (queue size: %u, frequency=%u MHz)", this->transmit_queue_.size(), frequency_mhz);
  return true;
}

void TuyaRfComponent::process_transmit_queue() {
  // Check if queue is empty
  if (this->transmit_queue_.empty()) {
    return;
  }

  // Check if we're already transmitting
  if (this->transmitting_) {
    return;
  }

  // Check if enough time has passed since last transmission
  uint32_t now = millis();
  if (this->last_transmit_time_ != 0 && (now - this->last_transmit_time_) < this->queue_delay_ms_) {
    return;
  }

  // Get the next item from queue
  TransmitQueueItem item = this->transmit_queue_.front();
  this->transmit_queue_.pop_front();

  // Log queue age if it's getting old
  uint32_t age_ms = now - item.queued_at;
  if (age_ms > 1000) {
    ESP_LOGW(TAG, "Processing queued command that is %u ms old", age_ms);
  } else {
    ESP_LOGD(TAG, "Processing queued command (age: %u ms, remaining: %u)", age_ms, this->transmit_queue_.size());
  }

  // Transmit the data
  this->RemoteTransmitterBase::temp_.set_data(item.data);
  this->next_transmit_frequency_mhz_ = item.frequency_mhz;
  this->send_(1, 0);  // send_times=1, send_wait=0
  this->next_transmit_frequency_mhz_ = 0;

  // Update last transmit time
  this->last_transmit_time_ = millis();
}

}  // namespace tuya_rf
}  // namespace esphome
