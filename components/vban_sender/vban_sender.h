#pragma once
#include "esphome.h"
#include <WiFiUdp.h>

namespace esphome {
namespace vban_sender {

// VBAN sample rate index for 16000 Hz
static const uint8_t VBAN_SR_16000 = 8;

class VBANSender : public Component {
 public:
  void set_target_ip(const std::string &ip) { target_ip_ = ip; }
  void set_target_port(uint16_t port) { target_port_ = port; }
  void set_stream_name(const std::string &name) { stream_name_ = name; }

  void setup() override {
    udp_.begin(0);
    ESP_LOGI("vban", "VBAN sender ready -> %s:%d stream=%s",
             target_ip_.c_str(), target_port_, stream_name_.c_str());
  }

  // Send raw PCM 16-bit mono audio data via VBAN
  // data: pointer to PCM samples (int16_t), len: byte length
  void send_audio(const uint8_t *data, size_t len) {
    if (!data || len == 0) return;
    size_t sample_count = len / 2;
    if (sample_count == 0) return;

    size_t offset = 0;
    while (offset < sample_count) {
      size_t chunk = std::min(sample_count - offset, (size_t)256);
      send_packet_(data + offset * 2, chunk);
      offset += chunk;
    }
  }

 protected:
  void send_packet_(const uint8_t *data, size_t sample_count) {
    uint8_t header[28] = {};

    // VBAN magic
    header[0] = 'V'; header[1] = 'B'; header[2] = 'A'; header[3] = 'N';

    // Sample rate index (16000 Hz)
    header[4] = VBAN_SR_16000;

    // Num samples - 1
    header[5] = (uint8_t)(sample_count - 1);

    // Num channels - 1 (mono = 0)
    header[6] = 0;

    // Data format: PCM 16-bit (0x01)
    header[7] = 0x01;

    // Stream name (16 bytes, null-padded)
    strncpy((char *)&header[8], stream_name_.c_str(), 16);

    // Frame counter (little-endian uint32)
    header[24] = frame_counter_ & 0xFF;
    header[25] = (frame_counter_ >> 8) & 0xFF;
    header[26] = (frame_counter_ >> 16) & 0xFF;
    header[27] = (frame_counter_ >> 24) & 0xFF;
    frame_counter_++;

    IPAddress dest;
    dest.fromString(target_ip_.c_str());
    udp_.beginPacket(dest, target_port_);
    udp_.write(header, 28);
    udp_.write(data, sample_count * 2);
    udp_.endPacket();
  }

  WiFiUDP udp_;
  std::string target_ip_;
  uint16_t target_port_{6980};
  std::string stream_name_{"AtomEcho"};
  uint32_t frame_counter_{0};
};

}  // namespace vban_sender
}  // namespace esphome
