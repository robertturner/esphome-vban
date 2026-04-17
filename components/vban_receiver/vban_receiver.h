#pragma once
#include "esphome.h"
#include "esphome/components/speaker/speaker.h"
#include "esphome/components/microphone/microphone.h"

#include <lwip/sockets.h>
#include <arpa/inet.h>
#include <fcntl.h>

#include <cstring>
#include <vector>

namespace esphome {
namespace vban_receiver {

class VBANReceiver : public Component {
 public:
  void set_speaker(speaker::Speaker *spk) { speaker_ = spk; }
  void set_microphone(microphone::Microphone *mic) { mic_ = mic; }
  void set_listen_port(uint16_t port) { listen_port_ = port; }
  void set_stream_name(const std::string &name) { stream_name_ = name; }
  void set_idle_timeout_ms(uint32_t ms) { idle_timeout_ms_ = ms; }

  float get_setup_priority() const override { return setup_priority::LATE; }

  void setup() override {
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

    if (playing_ && speaker_ != nullptr && speaker_->is_running() && !pending_buffer_.empty()) {
      size_t written = speaker_->play(pending_buffer_.data(), pending_buffer_.size());
      if (written > 0) {
        pending_buffer_.erase(pending_buffer_.begin(),
                              pending_buffer_.begin() + written);
      }
    }

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
    if (format_bit != 0x01) return;  // only PCM 16-bit

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

    bool can_play_live = speaker_ != nullptr && speaker_->is_running() && pending_buffer_.empty();
    if (can_play_live) {
      size_t written = speaker_->play(pcm, pcm_len);
      if (written < pcm_len) {
        size_t leftover = pcm_len - written;
        if (pending_buffer_.size() + leftover <= MAX_PENDING_BYTES) {
          pending_buffer_.insert(pending_buffer_.end(),
                                 pcm + written, pcm + pcm_len);
        }
      }
    } else {
      if (pending_buffer_.size() + pcm_len <= MAX_PENDING_BYTES) {
        pending_buffer_.insert(pending_buffer_.end(), pcm, pcm + pcm_len);
      }
    }
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
    pending_buffer_.clear();
    pending_buffer_.shrink_to_fit();
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
  uint32_t packets_received_{0};
  bool playing_{false};
  std::vector<uint8_t> pending_buffer_;
  static constexpr size_t MAX_PENDING_BYTES = 64 * 1024;  // ~2s of 16kHz 16-bit mono
};

}  // namespace vban_receiver
}  // namespace esphome
