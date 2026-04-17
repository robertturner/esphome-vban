#pragma once
#include "esphome.h"
#include "esphome/components/microphone/microphone.h"

#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <arpa/inet.h>

#include <cstring>
#include <algorithm>
#include <atomic>
#include <vector>

namespace esphome {
namespace vban_sender {

static const uint8_t VBAN_SR_16000 = 8;

class VBANSender : public Component {
 public:
  void set_microphone(microphone::Microphone *mic) { mic_ = mic; }
  void set_target_ip(const std::string &ip) { target_ip_ = ip; }
  void set_target_port(uint16_t port) { target_port_ = port; }
  void set_stream_name(const std::string &name) { stream_name_ = name; }
  void set_gain(float gain) { gain_ = gain; }

  float get_setup_priority() const override { return setup_priority::LATE; }

  void setup() override {
    sock_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock_ < 0) {
      ESP_LOGE("vban", "socket() failed: errno=%d", errno);
      mark_failed();
      return;
    }

    uint8_t ttl = 2;
    ::setsockopt(sock_, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl));

    int bcast = 1;
    ::setsockopt(sock_, SOL_SOCKET, SO_BROADCAST, &bcast, sizeof(bcast));

    std::memset(&dest_addr_, 0, sizeof(dest_addr_));
    dest_addr_.sin_family = AF_INET;
    dest_addr_.sin_port = htons(target_port_);
    if (::inet_aton(target_ip_.c_str(), &dest_addr_.sin_addr) == 0) {
      ESP_LOGE("vban", "invalid target_ip: %s", target_ip_.c_str());
      ::close(sock_);
      sock_ = -1;
      mark_failed();
      return;
    }

    if (mic_ != nullptr) {
      mic_->add_data_callback([this](const std::vector<uint8_t> &data) {
        this->send_audio(data.data(), data.size());
      });
    }

    ESP_LOGI("vban", "VBAN sender ready -> %s:%d stream=%s",
             target_ip_.c_str(), target_port_, stream_name_.c_str());
  }

  void loop() override {
    if (!mic_started_ && mic_ != nullptr) {
      mic_->start();
      mic_started_ = true;
      ESP_LOGI("vban", "Microphone started, streaming to %s:%d",
               target_ip_.c_str(), target_port_);
    }
  }

  void dump_config() override {
    ESP_LOGCONFIG("vban", "VBAN Sender:");
    ESP_LOGCONFIG("vban", "  Target: %s:%d", target_ip_.c_str(), target_port_);
    ESP_LOGCONFIG("vban", "  Stream name: %s", stream_name_.c_str());
    ESP_LOGCONFIG("vban", "  Socket: %s", sock_ >= 0 ? "OK" : "FAILED");
    ESP_LOGCONFIG("vban", "  Mic started: %s", mic_started_ ? "YES" : "NO");
    ESP_LOGCONFIG("vban", "  Frames sent: %u", (unsigned) frame_counter_.load());
  }

  void send_audio(const uint8_t *data, size_t len) {
    if (!data || len == 0 || sock_ < 0) return;
    size_t sample_count = len / 2;
    if (sample_count == 0) return;

    const int16_t *src = reinterpret_cast<const int16_t *>(data);
    const int16_t *samples = src;
    if (gain_ != 1.0f) {
      if (gain_scratch_.size() < sample_count) {
        gain_scratch_.resize(sample_count);
      }
      for (size_t i = 0; i < sample_count; i++) {
        int32_t v = (int32_t)(src[i] * gain_);
        if (v > 32767) v = 32767;
        else if (v < -32768) v = -32768;
        gain_scratch_[i] = (int16_t) v;
      }
      samples = gain_scratch_.data();
    }

    size_t offset = 0;
    while (offset < sample_count) {
      size_t chunk = std::min(sample_count - offset, (size_t)256);
      send_packet_(reinterpret_cast<const uint8_t *>(samples + offset), chunk);
      offset += chunk;
    }
  }

 protected:
  void send_packet_(const uint8_t *data, size_t sample_count) {
    uint8_t packet[28 + 512];
    uint8_t *header = packet;

    header[0] = 'V'; header[1] = 'B'; header[2] = 'A'; header[3] = 'N';
    header[4] = VBAN_SR_16000;
    header[5] = (uint8_t)(sample_count - 1);
    header[6] = 0;      // mono
    header[7] = 0x01;   // PCM 16-bit

    std::memset(&header[8], 0, 16);
    std::strncpy((char *)&header[8], stream_name_.c_str(), 16);

    uint32_t fc = frame_counter_.fetch_add(1, std::memory_order_relaxed);
    header[24] = fc & 0xFF;
    header[25] = (fc >> 8) & 0xFF;
    header[26] = (fc >> 16) & 0xFF;
    header[27] = (fc >> 24) & 0xFF;

    size_t payload = sample_count * 2;
    std::memcpy(packet + 28, data, payload);

    ::sendto(sock_, packet, 28 + payload, 0,
             (struct sockaddr *)&dest_addr_, sizeof(dest_addr_));
  }

  microphone::Microphone *mic_{nullptr};
  bool mic_started_{false};
  int sock_{-1};
  struct sockaddr_in dest_addr_{};
  std::string target_ip_;
  uint16_t target_port_{6980};
  std::string stream_name_{"AtomEcho"};
  float gain_{1.0f};
  std::vector<int16_t> gain_scratch_;
  std::atomic<uint32_t> frame_counter_{0};
};

}  // namespace vban_sender
}  // namespace esphome
