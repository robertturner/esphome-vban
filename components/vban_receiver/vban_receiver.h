#pragma once
#include "esphome.h"
#include "esphome/components/speaker/speaker.h"
#include "esphome/components/microphone/microphone.h"

#include <lwip/sockets.h>
#include <arpa/inet.h>
#include <fcntl.h>

#include <cstring>
#include <vector>
#include <algorithm>

namespace esphome {
namespace vban_receiver {

static const uint8_t VBAN_SR_16000 = 8;

class VBANReceiver : public Component {
 public:
  void set_speaker(speaker::Speaker *spk) { speaker_ = spk; }
  void set_microphone(microphone::Microphone *mic) { mic_ = mic; }
  void set_listen_port(uint16_t port) { listen_port_ = port; }
  void set_stream_name(const std::string &name) { stream_name_ = name; }
  void set_idle_timeout_ms(uint32_t ms) { idle_timeout_ms_ = ms; }

  float get_setup_priority() const override { return setup_priority::LATE; }

  void setup() override {
    ring_.assign(kRingCapacity, 0);

    sock_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock_ < 0) {
      ESP_LOGE("vban_rx", "socket() failed: errno=%d", errno);
      mark_failed();
      return;
    }

    int reuse = 1;
    ::setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in bind_addr{};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind_addr.sin_port = htons(listen_port_);

    if (::bind(sock_, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
      ESP_LOGE("vban_rx", "bind(%d) failed: errno=%d", listen_port_, errno);
      mark_failed();
      return;
    }

    int flags = ::fcntl(sock_, F_GETFL, 0);
    ::fcntl(sock_, F_SETFL, flags | O_NONBLOCK);

    ESP_LOGI("vban_rx", "Listening on UDP %d for VBAN stream '%s'",
             listen_port_, stream_name_.c_str());
  }

  void loop() override {
    uint8_t buf[2048];
    struct sockaddr_in from;
    socklen_t fromlen = sizeof(from);

    for (int i = 0; i < 8; i++) {
      int n = ::recvfrom(sock_, buf, sizeof(buf), 0,
                        (struct sockaddr *)&from, &fromlen);
      if (n < 28) break;
      handle_packet_(buf, n);
    }

    drain_ring_to_speaker_();

    if (playing_ && (millis() - last_packet_ms_) > idle_timeout_ms_) {
      stop_playback_();
    }
  }

  void dump_config() override {
    ESP_LOGCONFIG("vban_rx", "VBAN Receiver:");
    ESP_LOGCONFIG("vban_rx", "  Listen port: %d", listen_port_);
    ESP_LOGCONFIG("vban_rx", "  Stream name: %s", stream_name_.c_str());
    ESP_LOGCONFIG("vban_rx", "  Idle timeout: %u ms", (unsigned) idle_timeout_ms_);
    ESP_LOGCONFIG("vban_rx", "  Socket: %s", sock_ >= 0 ? "OK" : "FAILED");
    ESP_LOGCONFIG("vban_rx", "  Packets received: %u", (unsigned) packets_received_);
  }

 protected:
  void handle_packet_(const uint8_t *buf, int n) {
    if (buf[0] != 'V' || buf[1] != 'B' || buf[2] != 'A' || buf[3] != 'N') return;

    uint8_t sub_protocol = buf[4] & 0xE0;
    if (sub_protocol != 0x00) return;  // only audio sub-protocol

    uint8_t format_bit = buf[7] & 0x07;
    if (format_bit != 0x01) {
      log_format_warning_("format", format_bit);
      return;
    }

    uint8_t sr_index = buf[4] & 0x1F;
    if (sr_index != VBAN_SR_16000) {
      log_format_warning_("sample_rate", sr_index);
      return;
    }

    uint8_t channels = buf[6] + 1;
    if (channels != 1) {
      log_format_warning_("channels", channels);
      return;
    }

    char name[17] = {};
    std::memcpy(name, buf + 8, 16);
    if (std::strncmp(name, stream_name_.c_str(), 16) != 0) return;

    const uint8_t *pcm = buf + 28;
    size_t pcm_len = n - 28;
    if (pcm_len == 0) return;

    packets_received_++;
    last_packet_ms_ = millis();

    if (!playing_) {
      start_playback_();
    }

    bool can_play_live = speaker_ != nullptr && speaker_->is_running() && ring_empty_();
    if (can_play_live) {
      size_t written = speaker_->play(pcm, pcm_len);
      if (written < pcm_len) {
        ring_write_(pcm + written, pcm_len - written);
      }
    } else {
      ring_write_(pcm, pcm_len);
    }
  }

  void drain_ring_to_speaker_() {
    if (!playing_ || speaker_ == nullptr || !speaker_->is_running()) return;
    while (!ring_empty_()) {
      size_t tail = (write_idx_ + kRingCapacity - ring_size_) % kRingCapacity;
      size_t contig = std::min(ring_size_, kRingCapacity - tail);
      size_t written = speaker_->play(ring_.data() + tail, contig);
      if (written == 0) return;
      ring_size_ -= written;
      if (written < contig) return;  // speaker back-pressure
    }
  }

  void ring_write_(const uint8_t *data, size_t len) {
    size_t room = kRingCapacity - ring_size_;
    size_t to_copy = std::min(len, room);
    if (to_copy == 0) return;
    size_t first = std::min(to_copy, kRingCapacity - write_idx_);
    std::memcpy(ring_.data() + write_idx_, data, first);
    if (to_copy > first) {
      std::memcpy(ring_.data(), data + first, to_copy - first);
    }
    write_idx_ = (write_idx_ + to_copy) % kRingCapacity;
    ring_size_ += to_copy;
  }

  bool ring_empty_() const { return ring_size_ == 0; }

  void ring_reset_() {
    ring_size_ = 0;
    write_idx_ = 0;
  }

  void log_format_warning_(const char *field, uint8_t value) {
    uint32_t now = millis();
    if (now - last_format_warning_ms_ < 5000) return;
    last_format_warning_ms_ = now;
    ESP_LOGW("vban_rx", "Rejecting packet: unsupported %s=%u (expected mono PCM16 @ 16kHz)",
             field, (unsigned) value);
  }

  void start_playback_() {
    if (mic_ != nullptr && mic_->is_running()) {
      mic_->stop();
    }
    if (speaker_ != nullptr && !speaker_->is_running()) {
      speaker_->start();
    }
    playing_ = true;
    ESP_LOGD("vban_rx", "Stream '%s' started", stream_name_.c_str());
  }

  void stop_playback_() {
    if (speaker_ != nullptr && speaker_->is_running()) {
      speaker_->stop();
    }
    if (mic_ != nullptr && mic_->is_stopped()) {
      mic_->start();
    }
    ring_reset_();
    playing_ = false;
    ESP_LOGD("vban_rx", "Stream idle, mic resumed");
  }

  speaker::Speaker *speaker_{nullptr};
  microphone::Microphone *mic_{nullptr};
  int sock_{-1};
  uint16_t listen_port_{6980};
  std::string stream_name_;
  uint32_t idle_timeout_ms_{1500};
  uint32_t last_packet_ms_{0};
  uint32_t last_format_warning_ms_{0};
  uint32_t packets_received_{0};
  bool playing_{false};

  // Fixed-capacity circular buffer (~2 s at 16 kHz 16-bit mono).
  static constexpr size_t kRingCapacity = 64 * 1024;
  std::vector<uint8_t> ring_;
  size_t write_idx_{0};
  size_t ring_size_{0};
};

}  // namespace vban_receiver
}  // namespace esphome
