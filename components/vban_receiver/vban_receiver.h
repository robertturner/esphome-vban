#pragma once
#include "esphome.h"
//#include "esphome/components/speaker/speaker.h"
#include <driver/i2s_std.h>

#include <lwip/sockets.h>
#include <arpa/inet.h>
#include <fcntl.h>
//#include "FreeRTOS.h"

#include <functional>
#include <stdint.h>
#include <cstring>
#include <vector>
#include <algorithm>
#include <memory>

namespace esphome {
namespace vban_receiver {

static const uint8_t VBAN_SR_16000 = 8;
static const uint8_t VBAN_SR_48000 = 3;

class AudioOutput
{
public:
    AudioOutput() : _tx_handle(0) { }
    virtual ~AudioOutput() { }

    virtual bool setRate(int hz) = 0;
    virtual int getRate() const = 0;

    virtual bool begin(std::function<void(i2s_chan_handle_t)> initCallback = 0) = 0;
    virtual bool consumeSample(const int16_t sample[2]) = 0;
    virtual bool stop() = 0;

    i2s_chan_handle_t getHandle() const { return _tx_handle; }
protected:
    i2s_chan_handle_t _tx_handle;
};

static constexpr uint32_t I2S_DMA_BUF_COUNT_DEFAULT  = 8;
static constexpr uint32_t I2S_DMA_BUF_SIZE_DEFAULT   = 256;

class AudioOutputI2S : public AudioOutput
{
public:


    AudioOutputI2S(int dout_pin, int mclk_pin, int bclk_pin, int ws_pin) : hertz(48000),
                                                                                        i2sOn(false),
                                                                                        dout_pin(dout_pin),
                                                                                        mclk_pin(mclk_pin),
                                                                                        bclk_pin(bclk_pin),
                                                                                        ws_pin(ws_pin)
	{ }
    virtual ~AudioOutputI2S() { stop(); }

    //bool setPinout(int dout_pin, int mclk_pin, int bclk_pin, int ws_pin);
    bool setBuffers(int dmaBufferCount = I2S_DMA_BUF_COUNT_DEFAULT, int dmaBufferBytes = I2S_DMA_BUF_SIZE_DEFAULT) {
		if (i2sOn || (dmaBufferCount < 3) || (dmaBufferBytes & 3))
			return false;
		_buffers = dmaBufferCount;
		_bufferWords = dmaBufferBytes / 4;
		return true;
	}

    bool setRate(int hz) override {
		if (hz == hertz)
			return true;
		hertz = hz;
		if (i2sOn)
		{
			i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG((uint32_t)hz);
	#if SOC_CLK_APLL_SUPPORTED
			clk_cfg.clk_src = i2s_clock_src_t::I2S_CLK_SRC_APLL;
	#endif
			i2s_channel_disable(_tx_handle);
			i2s_channel_reconfig_std_clock(_tx_handle, &clk_cfg);
			i2s_channel_enable(_tx_handle);
		}
		return true;
	}
    int getRate() const override { return hertz; }

    bool begin(std::function<void(i2s_chan_handle_t)> initCallback = 0) override {
		if (i2sOn)
			return false;

		i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
		chan_cfg.dma_desc_num = _buffers;
		chan_cfg.dma_frame_num = _bufferWords;
		chan_cfg.auto_clear = true;
		assert(ESP_OK == i2s_new_channel(&chan_cfg, &_tx_handle, nullptr));

		i2s_std_config_t std_cfg = {
			.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG((uint32_t)hertz),
			.slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
			.gpio_cfg = {
				.mclk = (mclk_pin < 0) ? I2S_GPIO_UNUSED : (gpio_num_t)mclk_pin,
				.bclk = (bclk_pin < 0) ? I2S_GPIO_UNUSED : (gpio_num_t)bclk_pin,
				.ws = (ws_pin < 0) ? I2S_GPIO_UNUSED : (gpio_num_t)ws_pin,
				.dout = (dout_pin < 0) ? I2S_GPIO_UNUSED : (gpio_num_t)dout_pin,
				.din = I2S_GPIO_UNUSED,
				.invert_flags = {
					.mclk_inv = false,
					.bclk_inv = false,
					.ws_inv = false,
				},
			},
		};
		//I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG
		//I2S_STD_MSB_SLOT_DEFAULT_CONFIG
	#if SOC_CLK_APLL_SUPPORTED
		std_cfg.clk_cfg.clk_src = i2s_clock_src_t::I2S_CLK_SRC_APLL;
	#endif
		std_cfg.slot_cfg.bit_shift = true;
		assert(ESP_OK == i2s_channel_init_std_mode(_tx_handle, &std_cfg));

		int16_t a[2] = {0, 0};
		size_t written = 0;
		do {
			i2s_channel_preload_data(_tx_handle, (void*)a, sizeof(a), &written);
		} while (written);

		if (initCallback)
			initCallback(_tx_handle);
		assert(ESP_OK == i2s_channel_enable(_tx_handle));

		i2sOn = true;
		return true;
	}
    bool consumeSample(const int16_t sample[2]) override {
		if (!i2sOn)
			return false;

		uint32_t s32 = ((uint32_t)((uint16_t)sample[1]) << 16) | (uint32_t)((uint16_t)sample[0] & 0xffff);
		//uint32_t s32 = (((sample[1])) << 16) | ((sample[0]) & 0xffff);

		size_t i2s_bytes_written = sizeof(uint32_t);
		i2s_channel_write(_tx_handle, (const char*)&s32, sizeof(uint32_t), &i2s_bytes_written, 0);
		return i2s_bytes_written;
	}
    bool stop() override {
		if (!i2sOn) {
			i2sOn = false;
			i2s_channel_disable(_tx_handle);
			i2s_del_channel(_tx_handle);
		}
		return true;
	}

private:
    uint16_t hertz;
    bool i2sOn;
    int8_t dout_pin, mclk_pin, bclk_pin, ws_pin;
    size_t _buffers;
    size_t _bufferWords;
};


class VBANReceiver : public Component {
 public:
  //void set_speaker(speaker::Speaker *spk) { speaker_ = spk; }
  void set_listen_port(uint16_t port) { listen_port_ = port; }
  void set_stream_name(const std::string &name) { stream_name_ = name; }
  void set_idle_timeout_ms(uint32_t ms) { idle_timeout_ms_ = ms; }

  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  void setup() override {
    //ring_.assign(kRingCapacity, 0);
	sockBuff_.assign(2000, 0);

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
      ::close(sock_);
      sock_ = -1;
      mark_failed();
      return;
    }

#if 0
    int flags = ::fcntl(sock_, F_GETFL, 0);
    ::fcntl(sock_, F_SETFL, flags | O_NONBLOCK);
#else
	struct timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = 5000;
	::setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
#endif	

	xTaskCreate(socketTask_, "vban", 3000, this, ESP_TASK_PRIO_MAX - 1, 0);

    ESP_LOGI("vban_rx", "Listening on UDP %d for VBAN stream '%s'",
             listen_port_, stream_name_.c_str());
  }

  void loop() override {
#if 0	  
    struct sockaddr_in from;
    socklen_t fromlen = sizeof(from);

    for (int i = 0; i < 16; i++) {
      int n = ::recvfrom(sock_, sockBuff_.data(), sockBuff_.size(), 0,
                        (struct sockaddr *)&from, &fromlen);
	  if (n >= 0) {
		raw_packets_received_++;
		handle_packet_(sockBuff_.data(), n);
	  }
	  else
		break;
      
    }
#endif
    //drain_ring_to_speaker_();

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
    ESP_LOGCONFIG("vban_rx", "  Raw packets received: %u", (unsigned) raw_packets_received_);
    ESP_LOGCONFIG("vban_rx", "  Packets received: %u", (unsigned) packets_received_);
    ESP_LOGCONFIG("vban_rx", "  Packets lost:     %u", (unsigned) packets_lost_);
    ESP_LOGCONFIG("vban_rx", "  Out of order:     %u", (unsigned) packets_out_of_order_);
  }

 protected:
 
  static void socketTask_(void *pvParams)
  {
	VBANReceiver *inst = (VBANReceiver*)pvParams;
	inst->socketTask();
  }
  void socketTask()
  {	  
	struct sockaddr_in from;
    socklen_t fromlen = sizeof(from);

    for (int i = 0; i < 16; i++) {
      int n = ::recvfrom(sock_, sockBuff_.data(), sockBuff_.size(), 0,
                        (struct sockaddr *)&from, &fromlen);
						
	  if (n >= 0) {
		raw_packets_received_++;
		handle_packet_(sockBuff_.data(), n);
	  }
	  else {
	    if (errno == EINPROGRESS || errno == EAGAIN || errno == EWOULDBLOCK) {
          // not an error
          vTaskDelay(1);
        }
		else
			ESP_LOGD("vban_rx", "Socket error, errno: %d", errno);
		break;
      }
    }
	  
  }
 
  void handle_packet_(const uint8_t *buf, int n) {
	if (n < 24)
		return;
	
    if (buf[0] != 'V' || buf[1] != 'B' || buf[2] != 'A' || buf[3] != 'N') {
		log_format_warning_("header", *((uint32_t*)buf));
		return;
	}

    uint8_t sub_protocol = buf[4] & 0xE0;
    if (sub_protocol != 0x00) 
		return;  // only audio sub-protocol

    uint8_t format_bit = buf[7] & 0x07;
    if (format_bit != 0x01) { // 16-bit
      log_format_warning_("format", format_bit);
      return;
    }

    uint8_t sr_index = buf[4] & 0x1F;
    if (sr_index != VBAN_SR_48000) {
      log_format_warning_("sample_rate", sr_index);
      return;
    }

    uint8_t channels = buf[6] + 1;
    if (channels != 2) {
      log_format_warning_("channels", channels);
      return;
    }

    if (std::strncmp((char*)(buf + 8), stream_name_.c_str(), 16) != 0) {
		log_format_warning_("stream", 0);
		return;
	}

    const uint8_t *pcm = buf + 28;
    size_t pcm_len = n - 28;
    if (n <= 28) {
		log_format_warning_("pcm_len", 0);
		return;
	}

    uint32_t frame = ((uint32_t) buf[24])
                   | ((uint32_t) buf[25] << 8)
                   | ((uint32_t) buf[26] << 16)
                   | ((uint32_t) buf[27] << 24);
    if (packets_received_ > 0) {
      uint32_t expected = last_frame_counter_ + 1;
      if (frame != expected) {
        int32_t diff = (int32_t)(frame - expected);
        if (diff > 0) {
          packets_lost_ += diff;
        } else {
          packets_out_of_order_++;
        }
      }
    }
    last_frame_counter_ = frame;

    packets_received_++;
    last_packet_ms_ = millis();

    if (!playing_) {
      start_playback_(48000);
    }

#if 0
    bool can_play_live = speaker_ != nullptr && speaker_->is_running() && ring_empty_();
    if (can_play_live) {
      size_t written = speaker_->play(pcm, pcm_len);
      if (written < pcm_len) {
        ring_write_(pcm + written, pcm_len - written);
      }
    } else 
#endif		
	{
      ring_write_(pcm, pcm_len);
    }
  }
#if 0
  void drain_ring_to_speaker_() {
    if (!playing_ || speaker_ == nullptr || !speaker_->is_running()) return;
    while (!ring_empty_()) {
      size_t tail = (write_idx_ + kRingCapacity - ring_size_) % kRingCapacity;
      size_t contig = std::min(ring_size_, kRingCapacity - tail);
      size_t written = speaker_->play(ring_.data() + tail, contig);
      if (written == 0) 
		  return;
      ring_size_ -= written;
      if (written < contig) 
		  return;  // speaker back-pressure
    }
  }

  void ring_write_(const uint8_t *data, size_t len) {	  
    size_t room = kRingCapacity - ring_size_;
    size_t to_copy = std::min(len, room);
    if (to_copy == 0) 
		return;
    size_t first = std::min(to_copy, kRingCapacity - write_idx_);
    std::memcpy(ring_.data() + write_idx_, data, first);
    if (to_copy > first) {
      std::memcpy(ring_.data(), data + first, to_copy - first);
    }
    write_idx_ = (write_idx_ + to_copy) % kRingCapacity;
    ring_size_ += to_copy;
  }
#else
    void ring_write_(const uint8_t *pcmData, size_t len) {	 
		unsigned vbanNbr = len / 2;
		const int16_t *pcmSamples = (const int16_t *)pcmData;
		if (audioOut && vbanNbr > 0 && (vbanNbr & 1) == 0)
        {
            //static uint32_t lvl;
            //gpio_set_level(GPIO_NUM_5, lvl);
            //lvl ^= 1;

            for (; vbanNbr > 0; )
            {
                if (audioOut->consumeSample(pcmSamples))
                {
                    vbanNbr -= 2;
                    pcmSamples += 2;
                }
                else
                {
                    //overflowCnt++;
                    break;
                }
            }
        }


	}
#endif	

#if 0
  bool ring_empty_() const { return ring_size_ == 0; }

  void ring_reset_() {
    ring_size_ = 0;
    write_idx_ = 0;
  }
#endif
  void log_format_warning_(const char *field, uint8_t value) {
    uint32_t now = millis();
    if (now - last_format_warning_ms_ < 1000) 
		return;
    last_format_warning_ms_ = now;
	//ESP_LOGD
    ESP_LOGCONFIG("vban_rx", "Rejecting packet: unsupported %s=%u (expected mono PCM16 @ 16kHz)",
             field, (unsigned) value);
  }

  void start_playback_(unsigned sampleRate) {
#if 0
    if (speaker_ != nullptr && !speaker_->is_running()) {
      speaker_->start();
    }
#else
	//AudioOutputI2S *i2sPtr = new AudioOutputI2S(GPIO_NUM_5, -1, GPIO_NUM_33, GPIO_NUM_17);
	//std::unique_ptr<AudioOutputI2S> i2s = std::unique_ptr<AudioOutputI2S>(i2sPtr);
	std::unique_ptr<AudioOutputI2S> i2s = std::make_unique<AudioOutputI2S>(GPIO_NUM_5, -1, GPIO_NUM_33, GPIO_NUM_17);
	i2s->setBuffers(16, 2*1024);
	audioOut = std::move(i2s);
	audioOut->setRate(sampleRate);
	audioOut->begin();
#endif
    playing_ = true;
    ESP_LOGD("vban_rx", "Stream '%s' started", stream_name_.c_str());
  }

  void stop_playback_() {
#if 0	  
    if (speaker_ != nullptr && speaker_->is_running()) {
      speaker_->stop();
    }
    ring_reset_();
#else	
	audioOut = 0;
#endif	
    playing_ = false;
    ESP_LOGD("vban_rx", "Stream idle");
  }

  //speaker::Speaker *speaker_{nullptr};
  std::unique_ptr<AudioOutput> audioOut;
  int sock_{-1};
  std::vector<uint8_t> sockBuff_;
  uint16_t listen_port_{6980};
  std::string stream_name_;
  uint32_t idle_timeout_ms_{1500};
  uint32_t last_packet_ms_{0};
  uint32_t last_format_warning_ms_{0};
  uint32_t raw_packets_received_{0};
  uint32_t packets_received_{0};
  uint32_t packets_lost_{0};
  uint32_t packets_out_of_order_{0};
  uint32_t last_frame_counter_{0};
  bool playing_{false};

  // Fixed-capacity circular buffer (~1 s at 16 kHz 16-bit mono).
#if 0  
  static constexpr size_t kRingCapacity = 32 * 1024;
  std::vector<uint8_t> ring_;
  size_t write_idx_{0};
  size_t ring_size_{0};
#endif  
};

}  // namespace vban_receiver
}  // namespace esphome
