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

	virtual bool isRunning() const = 0;

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
	bool isRunning() const override { return i2sOn; }

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
		
		size_t i2s_bytes_written = sizeof(uint32_t);
		i2s_channel_write(_tx_handle, (const char*)&s32, sizeof(uint32_t), &i2s_bytes_written, 0);
		return i2s_bytes_written;
	}
    bool stop() override {
		if (i2sOn) {
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

class VBanPacket
{
public:
  #define VBAN_HEADER_SIZE            (4 + 1 + 1 + 1 + 1 + 16)
  #define VBAN_STREAM_NAME_SIZE       16
  #define VBAN_PROTOCOL_MAX_SIZE      1464
  #define VBAN_DATA_MAX_SIZE          (VBAN_PROTOCOL_MAX_SIZE - VBAN_HEADER_SIZE)
  #define VBAN_CHANNELS_MAX_NB        256
  #define VBAN_SAMPLES_MAX_NB         256


  #define VBAN_PACKET_MAX_SAMPLES 256  
  #define VBAN_PACKET_HEADER_BYTES 24  
  #define VBAN_PACKET_COUNTER_BYTES 4
  #define VBAN_PACKET_MAX_LEN_BYTES (VBAN_PACKET_HEADER_BYTES + VBAN_PACKET_COUNTER_BYTES + VBAN_PACKET_MAX_SAMPLES*2)

  #define VBAN_SR_MASK                0x1F
  #define VBAN_SR_MAXNUMBER           21
  inline constexpr static long VBanSRList[VBAN_SR_MAXNUMBER]=
  {
    6000, 12000, 24000, 48000, 96000, 192000, 384000,
    8000, 16000, 32000, 64000, 128000, 256000, 512000,
    11025, 22050, 44100, 88200, 176400, 352800, 705600
  };

  enum VBanSampleRates
  {
    SAMPLE_RATE_6000_HZ,
    SAMPLE_RATE_12000_HZ,
    SAMPLE_RATE_24000_HZ,
    SAMPLE_RATE_48000_HZ,
    SAMPLE_RATE_96000_HZ,
    SAMPLE_RATE_192000_HZ,
    SAMPLE_RATE_384000_HZ,
    SAMPLE_RATE_8000_HZ,
    SAMPLE_RATE_16000_HZ,
    SAMPLE_RATE_32000_HZ,
    SAMPLE_RATE_64000_HZ,
    SAMPLE_RATE_128000_HZ,
    SAMPLE_RATE_256000_HZ,
    SAMPLE_RATE_512000_HZ,
    SAMPLE_RATE_11025_HZ,
    SAMPLE_RATE_22050_HZ,
    SAMPLE_RATE_44100_HZ,
    SAMPLE_RATE_88200_HZ,
    SAMPLE_RATE_176400_HZ,
    SAMPLE_RATE_352800_HZ,
    SAMPLE_RATE_705600_HZ
  };


  #define VBAN_PROTOCOL_MASK          0xE0
  enum VBanProtocol
  {
    VBAN_PROTOCOL_AUDIO         =   0x00,
    VBAN_PROTOCOL_SERIAL        =   0x20,
    VBAN_PROTOCOL_TXT           =   0x40,
    VBAN_PROTOCOL_UNDEFINED_1   =   0x80,
    VBAN_PROTOCOL_UNDEFINED_2   =   0xA0,
    VBAN_PROTOCOL_UNDEFINED_3   =   0xC0,
    VBAN_PROTOCOL_UNDEFINED_4   =   0xE0
  };

  #define VBAN_BIT_RESOLUTION_MASK    0x07
  enum VBanBitResolution
  {
    VBAN_BITFMT_8_INT = 0,
    VBAN_BITFMT_16_INT,
    VBAN_BITFMT_24_INT,
    VBAN_BITFMT_32_INT,
    VBAN_BITFMT_32_FLOAT,
    VBAN_BITFMT_64_FLOAT,
    VBAN_BITFMT_12_INT,
    VBAN_BITFMT_10_INT,
    VBAN_BIT_RESOLUTION_MAX
  };

  inline constexpr static int VBanBitResolutionSize[VBAN_BIT_RESOLUTION_MAX]
  {
    1, 2, 3, 4, 4, 8
  };

  #define VBAN_RESERVED_MASK          0x08

  #define VBAN_CODEC_MASK             0xF0
  enum VBanCodec
  {
    VBAN_CODEC_PCM              =   0x00,
    VBAN_CODEC_VBCA             =   0x10,
    VBAN_CODEC_VBCV             =   0x20,
    VBAN_CODEC_UNDEFINED_3      =   0x30,
    VBAN_CODEC_UNDEFINED_4      =   0x40,
    VBAN_CODEC_UNDEFINED_5      =   0x50,
    VBAN_CODEC_UNDEFINED_6      =   0x60,
    VBAN_CODEC_UNDEFINED_7      =   0x70,
    VBAN_CODEC_UNDEFINED_8      =   0x80,
    VBAN_CODEC_UNDEFINED_9      =   0x90,
    VBAN_CODEC_UNDEFINED_10     =   0xA0,
    VBAN_CODEC_UNDEFINED_11     =   0xB0,
    VBAN_CODEC_UNDEFINED_12     =   0xC0,
    VBAN_CODEC_UNDEFINED_13     =   0xD0,
    VBAN_CODEC_UNDEFINED_14     =   0xE0,
    VBAN_CODEC_USER             =   0xF0
  };


  /********************************************************
   *              TEXT SUB PROTOCOL                       *
   ********************************************************/

  #define VBAN_BPS_MASK           0xE0
  #define VBAN_BPS_MAXNUMBER      25
  inline constexpr static long VBanBPSList[VBAN_BPS_MAXNUMBER] =
  {
      0,      110,    150,    300,    600,
      1200,   2400,   4800,   9600,   14400,
      19200,  31250,  38400,  57600,  115200,
      128000, 230400, 250000, 256000, 460800,
      921600,1000000,1500000,2000000, 3000000
  };

  #define VBAN_DATATYPE_MASK          0x07
  #define VBAN_DATATYPE_MAXNUMBER     1
  enum VBanDataTypeList
  {
      VBAN_DATATYPE_8BITS = 0
  };

  #define VBAN_STREAMTYPE_MASK        0xF0
  enum VBanStreamType
  {
      VBAN_TXT_ASCII          =   0x00,
      VBAN_TXT_UTF8           =   0x10,
      VBAN_TXT_WCHAR          =   0x20,
      VBAN_TXT_UNDEFINED_3    =   0x30,
      VBAN_TXT_UNDEFINED_4    =   0x40,
      VBAN_TXT_UNDEFINED_5    =   0x50,
      VBAN_TXT_UNDEFINED_6    =   0x60,
      VBAN_TXT_UNDEFINED_7    =   0x70,
      VBAN_TXT_UNDEFINED_8    =   0x80,
      VBAN_TXT_UNDEFINED_9    =   0x90,
      VBAN_TXT_UNDEFINED_10   =   0xA0,
      VBAN_TXT_UNDEFINED_11   =   0xB0,
      VBAN_TXT_UNDEFINED_12   =   0xC0,
      VBAN_TXT_UNDEFINED_13   =   0xD0,
      VBAN_TXT_UNDEFINED_14   =   0xE0,
      VBAN_TXT_USER           =   0xF0
  };

  struct VBanHeaderCodec
  {
    VBanHeaderCodec() : sample_rate(0), num_samples(0), num_channels(0), sample_format(0) { }
    
    uint8_t     sample_rate;                          /* SR index (see SRList above) */
    uint8_t     num_samples;                         /* nb sample per frame (1 to 256) */
    uint8_t     num_channels;                         /* nb channel (1 to 256) */
    uint8_t     sample_format;  

    unsigned getNumChannels() const { return num_channels + 1; }

    unsigned getSampleRate() const 
    {
      unsigned vbanSampleRateIdx = sample_rate & VBAN_SR_MASK;
      return VBanSRList[vbanSampleRateIdx];
    }

    VBanProtocol getProtocol() const { return (VBanProtocol)(sample_rate & VBAN_PROTOCOL_MASK); }

    VBanBitResolution getBitrate() const { return (VBanBitResolution)(sample_format & VBAN_BIT_RESOLUTION_MASK); }
  };

  struct VBanHeader
  {
      char        preamble[4];                               /* contains 'V' 'B', 'A', 'N' */
      VBanHeaderCodec codecInfo;
      char        stream_name[VBAN_STREAM_NAME_SIZE];  /* stream name */
  } ;

  static const uint16_t DefaultPort = 6980;

  VBanPacket(const uint8_t *data, int len) : headerStart(data), pktLen(len)
  { }

  inline bool checkValid() const
  {
    if (pktLen <= (VBAN_PACKET_HEADER_BYTES + VBAN_PACKET_COUNTER_BYTES) || (pktLen > VBAN_PACKET_MAX_LEN_BYTES))
        return false;
    if (strncmp("VBAN", (char*)headerStart, 4) != 0)
        return 0;
    uint16_t vban_rx_data_bytes = pktLen - (VBAN_PACKET_HEADER_BYTES + VBAN_PACKET_COUNTER_BYTES);
    uint16_t vban_rx_sample_count = vban_rx_data_bytes / 2;
    if (vban_rx_sample_count > VBAN_PACKET_MAX_SAMPLES)
        return false;
    return true;
  }

  const int16_t * getSamplesData(int *sampleCnt = 0) const
  {
    uint16_t vban_rx_data_bytes = pktLen - (VBAN_PACKET_HEADER_BYTES + VBAN_PACKET_COUNTER_BYTES);
    uint16_t vban_rx_sample_count = vban_rx_data_bytes / 2;
    if (sampleCnt)
        *sampleCnt = vban_rx_sample_count;
    return (int16_t*)&headerStart[VBAN_PACKET_HEADER_BYTES + VBAN_PACKET_COUNTER_BYTES];
  }

  const VBanHeader & getHeader() const { return *((const VBanHeader *)headerStart); }

  std::string getStreamName() const
  {
    const VBanHeader &header = getHeader();
    char stream_name[sizeof(header.stream_name) + 1];
    strncpy(stream_name, header.stream_name, sizeof(header.stream_name));
    stream_name[sizeof(header.stream_name)] = '\0';
    return std::string(stream_name);
  }
  void getStreamName(std::string &name) const
  {
    const VBanHeader &header = getHeader();
	name.resize(sizeof(header.stream_name));
    strncpy(name.data(), header.stream_name, sizeof(header.stream_name));
    name.resize(strlen(name.c_str()));
  }
  bool checkStreamName(const char *name) const
  {
	const VBanHeader &header = getHeader();
	return ::strncmp(name, header.stream_name, sizeof(header.stream_name)) == 0;
  }

  unsigned getNumChannels() const { return getHeader().codecInfo.getNumChannels(); }

  unsigned getSampleRate() const { return getHeader().codecInfo.getSampleRate(); }

  VBanProtocol getProtocol() const { return getHeader().codecInfo.getProtocol(); }

  VBanBitResolution getBitrate() const { return getHeader().codecInfo.getBitrate(); }

  uint32_t getFrameNum() const {
    return ((uint32_t) headerStart[VBAN_PACKET_HEADER_BYTES])
		   | ((uint32_t) headerStart[VBAN_PACKET_HEADER_BYTES+1] << 8)
		   | ((uint32_t) headerStart[VBAN_PACKET_HEADER_BYTES+2] << 16)
		   | ((uint32_t) headerStart[VBAN_PACKET_HEADER_BYTES+3] << 24);	  
  }
private:
  const uint8_t *headerStart;
  int pktLen;
};

class VBANReceiver : public Component {
 public: 
  void set_listen_port(uint16_t port) { listen_port_ = port; }
  void set_stream_name(const std::string &name) { stream_name_ = name; }
  void set_idle_timeout_ms(uint32_t ms) { idle_timeout_ms_ = ms; }
  void set_dout_pin(int dout_pin) { dout_pin_ = (gpio_num_t)dout_pin; }
  void set_mclk_pin(int mclk_pin) { mclk_pin_ = (gpio_num_t)mclk_pin; }
  void set_bclk_pin(int bclk_pin) { bclk_pin_ = (gpio_num_t)bclk_pin; }
  void set_lrclk_pin(int lrclk_pin) { lrclk_pin_ = (gpio_num_t)lrclk_pin; }

  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

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
      ::close(sock_);
      sock_ = -1;
      mark_failed();
      return;
    }

	struct timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = 50000;
	::setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

	xTaskCreate(socketTask_, "vban", 3000, this, ESP_TASK_PRIO_MAX - 1, 0);

    ESP_LOGI("vban_rx", "Listening on UDP %d for VBAN stream '%s'", listen_port_, stream_name_.c_str());
  }
  
  void loop() override {
	  disable_loop(); // We use our own thread
  }

  void dump_config() override {
    ESP_LOGCONFIG("vban_rx", "VBAN Receiver:");
    ESP_LOGCONFIG("vban_rx", "  Listen port: %d", listen_port_);
    ESP_LOGCONFIG("vban_rx", "  Idle timeout: %u ms", (unsigned) idle_timeout_ms_);
    ESP_LOGCONFIG("vban_rx", "  Socket: %s", sock_ >= 0 ? "OK" : "FAILED");
    ESP_LOGCONFIG("vban_rx", "  Raw packets received: %u", (unsigned) raw_packets_received_);
    ESP_LOGCONFIG("vban_rx", "  Packets received: %u", (unsigned) packets_received_);
    ESP_LOGCONFIG("vban_rx", "  Packets lost:     %u", (unsigned) packets_lost_);
	ESP_LOGCONFIG("vban_rx", "  Data overflows:   %u", (unsigned) data_overflows_);
    ESP_LOGCONFIG("vban_rx", "  Out of order:     %u", (unsigned) packets_out_of_order_);
	ESP_LOGCONFIG("vban_rx", "  Playing: %s", playing() ? "YES" : "NO");
    ESP_LOGCONFIG("vban_rx", "  Current stream name: %s", current_stream_name_.c_str());
	ESP_LOGCONFIG("vban_rx", "  Current samplerate: %u", current_samplerate_);
  }
  
  bool playing() const { return audioOut && audioOut->isRunning(); }

 protected:
 
  static void socketTask_(void *pvParams)
  {
	VBANReceiver *inst = static_cast<VBANReceiver*>(pvParams);
	inst->socketTask();
  }
  void socketTask()
  {	  
	sockBuff_.assign(2000, 0);
	current_stream_name_.reserve(16);

	struct sockaddr_in from;
    socklen_t fromlen = sizeof(from);

    for (;;) {
      int n = ::recvfrom(sock_, sockBuff_.data(), sockBuff_.size(), 0, (struct sockaddr *)&from, &fromlen);
						
	  if (n >= 0) {
		raw_packets_received_++;		
		VBanPacket packet(sockBuff_.data(), n);
		if (packet.checkValid())
		  handle_packet_(packet);
		else
		  log_format_warning_("header", 0);
	  }
	  else {
	    if (errno == EINPROGRESS || errno == EAGAIN || errno == EWOULDBLOCK) {
          // not an error
        }
		else {
		  ESP_LOGD("vban_rx", "Socket error, errno: %d", errno);
		}
		if (playing() && (millis() - last_packet_ms_) > idle_timeout_ms_) {
		  stop_playback_();
		}
	  
		vTaskDelay(1);
      }
    }
	vTaskDelete(0);
  }
 
  void handle_packet_(const VBanPacket &packet) {
	if (packet.getProtocol() != VBanPacket::VBAN_PROTOCOL_AUDIO) {
	  log_format_warning_("protocol", (unsigned)packet.getProtocol());
	  return;
	}
	if (packet.getBitrate() != VBanPacket::VBAN_BITFMT_16_INT) {
	  log_format_warning_("format", (unsigned)packet.getBitrate());
	  return;
	}
	if (packet.getNumChannels() != 2) {
	  log_format_warning_("channels", packet.getNumChannels());
	  return;
	}
	if (!stream_name_.empty() && !packet.checkStreamName(stream_name_.c_str())) {
	  log_format_warning_("stream", 0);
	  return;
	}
	if (!packet.checkStreamName(current_stream_name_.c_str()))
	  packet.getStreamName(current_stream_name_);

	unsigned sr = packet.getSampleRate();
	if (playing()) {
	  if (sr != audioOut->getRate()) {
		audioOut->setRate(sr);
		current_samplerate_ = sr;
	  }
	}
	else
	  start_playback_(sr);

    uint32_t frame = packet.getFrameNum();
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

	int vbanNbr;
	const int16_t *pcmSamples = packet.getSamplesData(&vbanNbr);
	if ((vbanNbr & 1) == 0) {
	  for (; vbanNbr > 0; ) {
		if (audioOut->consumeSample(pcmSamples)) {
		  vbanNbr -= 2;
		  pcmSamples += 2;
		}
		else {
		  data_overflows_++;
		  break;
		}
	  }
	}	
  }


  void log_format_warning_(const char *field, uint8_t value) {
    uint32_t now = millis();
    if (now - last_format_warning_ms_ < 1000) 
	  return;
    last_format_warning_ms_ = now;

    ESP_LOGD("vban_rx", "Rejecting packet: unsupported %s=%u (expected mono PCM16 @ 16kHz)", field, (unsigned) value);
  }

  void start_playback_(unsigned sampleRate) {
	if (!audioOut) {
		std::unique_ptr<AudioOutputI2S> i2s = std::make_unique<AudioOutputI2S>(dout_pin_, mclk_pin_, bclk_pin_, lrclk_pin_);
		i2s->setBuffers(16, 2048);
		audioOut = std::move(i2s);
	}
	audioOut->setRate(sampleRate);
	current_samplerate_ = sampleRate;
	audioOut->begin();

    ESP_LOGD("vban_rx", "Stream '%s' started", current_stream_name_.c_str());
  }

  void stop_playback_() {
	audioOut->stop();
	current_samplerate_ = 0;
	current_stream_name_.resize(0);
    ESP_LOGD("vban_rx", "Stream idle");
  }

  std::unique_ptr<AudioOutput> audioOut;
  int sock_{-1};
  std::vector<uint8_t> sockBuff_;
  uint16_t listen_port_{6980};
  std::string stream_name_;
  std::string current_stream_name_;
  uint32_t idle_timeout_ms_{1500};
  uint32_t last_packet_ms_{0};
  uint32_t last_format_warning_ms_{0};
  uint32_t raw_packets_received_{0};
  uint32_t packets_received_{0};
  uint32_t packets_lost_{0};
  uint32_t data_overflows_{0};
  uint32_t packets_out_of_order_{0};
  uint32_t last_frame_counter_{0};
  uint32_t current_samplerate_{0};
  gpio_num_t dout_pin_{(gpio_num_t)-1}, mclk_pin_{(gpio_num_t)-1}, bclk_pin_{(gpio_num_t)-1}, lrclk_pin_{(gpio_num_t)-1};

};

}  // namespace vban_receiver
}  // namespace esphome
