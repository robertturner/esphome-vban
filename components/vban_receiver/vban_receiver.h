#pragma once
#include "esphome.h"
#include <driver/i2s_std.h>

#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

#include <lwip/sockets.h>
#include <arpa/inet.h>
#include <fcntl.h>

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

static constexpr int I2S_DMA_BUF_COUNT_DEFAULT  = 8;
static constexpr int I2S_DMA_BUF_SIZE_DEFAULT   = 256;

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

static constexpr int SPDIF_DMA_BUF_COUNT_DEFAULT = 8;
static constexpr int SPDIF_DMA_BUF_SIZE_DEFAULT = 256;

// Constexpr BMC encoder for compile-time LUT generation.
// Encodes with start phase=true (HIGH). The complement property allows phase=false
// via XOR: bmc_encode(v, N, false) == bmc_encode(v, N, true) ^ mask
static constexpr uint16_t bmc_lut_encode(uint32_t data, uint8_t num_bits) 
{
	uint16_t bmc = 0;
	bool phase = true;
	for (uint8_t i = 0; i < num_bits; i++) 
	{
		bool bit = (data >> i) & 1;
		uint8_t bmc_pair = phase ? (bit ? 0b01 : 0b00) : (bit ? 0b10 : 0b11);
		bmc |= static_cast<uint16_t>(bmc_pair) << ((num_bits - 1 - i) * 2);
		if (!bit)
			phase = !phase;
	}
	return bmc;
}

static constexpr auto BMC_LUT_4 = [] 
{
	std::array<uint8_t, 16> t{};
	for (uint32_t i = 0; i < 16; i++)
		t[i] = static_cast<uint8_t>(bmc_lut_encode(i, 4));
	return t;
}();
// 8-bit BMC lookup table: 256 entries (512 bytes in flash)
// Index: 8-bit data value (0-255), always phase=true start
static constexpr auto BMC_LUT_8 = [] 
{
	std::array<uint16_t, 256> t{};
	for (uint32_t i = 0; i < 256; i++)
		t[i] = bmc_lut_encode(i, 8);
	return t;
}();


class AudioOutputSPDIF : public AudioOutput
{
public:
    AudioOutputSPDIF(int dout_pin) : hertz(48000),
									i2sOn(false),
									doutPin(dout_pin),
									frame_num(0)
	{ }
    
    virtual ~AudioOutputSPDIF() { stop(); }
    
    bool setBuffers(int dmaBufferCount = SPDIF_DMA_BUF_COUNT_DEFAULT, int dmaBufferBytes = SPDIF_DMA_BUF_SIZE_DEFAULT * 4) {
		if (i2sOn || (dmaBufferCount < 3) || (dmaBufferBytes & 3))
			return false;
		_buffers = dmaBufferCount;
		_bufferWords = dmaBufferBytes / 4;
		return true;
	}

    bool setRate(int hz) override {
		if (hz < 32000)
			return false;
		if (hz == hertz)
			return true;
		hertz = hz;
		if (i2sOn)
		{
			i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG((uint32_t)AdjustI2SRate(hz));
	#if SOC_CLK_APLL_SUPPORTED
			clk_cfg.clk_src = i2s_clock_src_t::I2S_CLK_SRC_APLL;
	#endif
			i2s_channel_disable(_tx_handle);
			i2s_channel_reconfig_std_clock(_tx_handle, &clk_cfg);
			buildChannelStatusBits(hz);
			frame_num = 0;
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
			.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG((uint32_t)AdjustI2SRate(hertz)),
			.slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
			.gpio_cfg = {
				.mclk = I2S_GPIO_UNUSED,
				.bclk = I2S_GPIO_UNUSED,
				.ws = I2S_GPIO_UNUSED,
				.dout = (gpio_num_t)doutPin,
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
		assert(ESP_OK == i2s_channel_init_std_mode(_tx_handle, &std_cfg));
		buildChannelStatusBits(hertz);
		if (initCallback)
			initCallback(_tx_handle);
		assert(ESP_OK == i2s_channel_enable(_tx_handle));

		i2sOn = true;
		frame_num = 0;
		return true;
	}

    bool consumeSample(const int16_t sample[2]) override {
		if (!i2sOn) 
			return true;    // Sink the data

		// S/PDIF encoding:
		//   http://www.hardwarebook.info/S/PDIF
		// Original sources: Teensy Audio Library
		//   https://github.com/PaulStoffregen/Audio/blob/master/output_spdif2.cpp

		uint32_t buf[4];
		uint8_t startingFrameNum = frame_num;
		encodeSample(&buf[0], sample[0], true, frame_num);
		//encodeSample(&buf[2], (channels > 1 ) ? sample[1] : sample[0], false, frame_num);
		encodeSample(&buf[2], sample[1], false, frame_num);
		size_t bytes_written;
		esp_err_t ret = i2s_channel_write(_tx_handle, (const char*)&buf, 8 * 2, &bytes_written, 10);
		// If we didn't write all bytes, return false early and do not increment frame_num
		if ((ret != ESP_OK) || (bytes_written != (8 * 2))) 
		{
			frame_num = startingFrameNum;
			return false;
		}   
		return true;
}

    bool stop() override {
		if (i2sOn) {
			i2sOn = false;
			i2s_channel_disable(_tx_handle);
			i2s_del_channel(_tx_handle);
			_tx_handle = 0;
			frame_num = 0;
		}
		return true;
	}

protected:
    const uint32_t VUCP_PREAMBLE_B = 0xCCE80000; // 11001100 11101000
    const uint32_t VUCP_PREAMBLE_M = 0xCCE20000; // 11001100 11100010
    const uint32_t VUCP_PREAMBLE_W = 0xCCE40000; // 11001100 11100100

    inline int AdjustI2SRate(int hz) { return 2 * hz; }
    void buildChannelStatusBits(int sampleRate) {
		memset(channel_status_, 0, sizeof(channel_status_));
		uint8_t freq_code;
		switch (sampleRate) 
		{
		case 44100:
			freq_code = 0x0;  // 0000
			break;
		case 48000:
			freq_code = 0x2;  // 0010
			break;
		default:
			// Other values are possible but they're not supported by ESPHome
			freq_code = 0x1;  // 0001 = not indicated
			break;
		}
		// Byte 3: freq_code in bits 0-3, clock accuracy (00) in bits 4-5
		channel_status_[3] = freq_code;  // Clock accuracy bits 4-5 are already 0
	}
    inline bool getChannelStatusBit(uint8_t frame) const { return (channel_status_[frame >> 3] >> (frame & 7)) & 1; }
	
	void encodeSample(uint32_t dest[2], int16_t sample, bool isLeftChannel, uint8_t &frameInBlock) const
	{
		const uint8_t *pcm_sample = (uint8_t*)&sample;
		//uint32_t raw_subframe = ((uint32_t)((uint16_t)sample)) << 12;
		uint32_t raw_subframe = (static_cast<uint32_t>(pcm_sample[1]) << 20) | (static_cast<uint32_t>(pcm_sample[0]) << 12);

		bool c_bit = getChannelStatusBit(frameInBlock);
		if (c_bit)
			raw_subframe |= (1U << 30);

		uint32_t bits_4_30 = (raw_subframe >> 4) & 0x07FFFFFF;  // 27 bits (4-30)
		uint32_t ones_count = __builtin_popcount(bits_4_30);
		uint32_t parity = ones_count & 1;  // 1 if odd count, 0 if even
		raw_subframe |= parity << 31;      // Set P bit to make total even

		static constexpr uint8_t PREAMBLE_B = 0x17;  // Block start (left channel, frame 0)
		static constexpr uint8_t PREAMBLE_M = 0x1d;  // Left channel (not block start)
		static constexpr uint8_t PREAMBLE_W = 0x1b;  // Right channel
		// ============================================================================
		// Select preamble based on position in block and channel
		// ============================================================================
		// B = block start (left channel, frame 0 of 192-frame block)
		// M = left channel (frames 1-191)
		// W = right channel (all frames)
		uint8_t preamble;
		if (isLeftChannel) 
			preamble = (frameInBlock == 0) ? PREAMBLE_B : PREAMBLE_M;
		else 
			preamble = PREAMBLE_W;
		
		// ============================================================================
		// BMC encode the data portion (bits 4-31) using lookup tables
		// ============================================================================
		// The I2S uses 16-bit halfword swap: bits 16-31 transmit before bits 0-15.
		// This applies to BOTH word[0] and word[1].
		//
		// word[0] transmission order: [16-23] → [24-31] → [0-7] → [8-15]
		// For correct S/PDIF subframe order (preamble → aux → audio):
		//   - bits 16-23: preamble (8 BMC bits)
		//   - bits 24-31: BMC(subframe bits 4-7) - first aux nibble
		//   - bits 0-7:   BMC(subframe bits 8-11) - second aux nibble
		//   - bits 8-15:  BMC(subframe bits 12-15) - audio low nibble
		//
		// word[1] transmission order: [16-31] → [0-15]
		// For correct S/PDIF subframe order:
		//   - bits 16-31: BMC(subframe bits 16-23) - audio mid byte
		//   - bits 0-15:  BMC(subframe bits 24-31) - audio high nibble + VUCP
		// ============================================================================

		// All preambles end at phase HIGH. Bits 4-11 are always zero for 16-bit audio;
		// two zero nibbles flip phase 8 times total → back to HIGH.
		// So bits 12-15 always start encoding at phase=true.

		// Bits 12-15: 4-bit LUT lookup (always phase=true start)
		uint32_t nibble = (raw_subframe >> 12) & 0xF;
		uint32_t bmc_12_15 = BMC_LUT_4[nibble];

		// Phase tracking via branchless XOR mask:
		// - 0x0000 means phase=true (use LUT value directly)
		// - 0xFFFF means phase=false (complement LUT value)
		// End phase = start XOR (popcount & 1) since zero-bits flip phase,
		// and for even bit widths: #zeros parity == popcount parity.
		uint32_t phase_mask = -(__builtin_popcount(nibble) & 1u) & 0xFFFF;

		// Bits 16-23: 8-bit LUT lookup with phase correction
		uint32_t byte_mid = (raw_subframe >> 16) & 0xFF;
		uint32_t bmc_16_23 = BMC_LUT_8[byte_mid] ^ phase_mask;
		phase_mask ^= -(__builtin_popcount(byte_mid) & 1u) & 0xFFFF;

		// Bits 24-31: 8-bit LUT lookup with phase correction
		uint32_t byte_hi = (raw_subframe >> 24) & 0xFF;
		uint32_t bmc_24_31 = BMC_LUT_8[byte_hi] ^ phase_mask;

		// ============================================================================
		// Combine with correct positioning for I2S transmission
		// ============================================================================
		// I2S with halfword swap: transmits bits 16-31, then bits 0-15.
		// Within each halfword, MSB (highest bit) is transmitted first.
		//
		// For upper halfword (bits 16-31): bit 31 → bit 16
		// For lower halfword (bits 0-15):  bit 15 → bit 0
		//
		// Desired S/PDIF order: preamble → bmc_4_7 → bmc_8_11 → bmc_12_15
		//
		// word[0] layout for correct transmission:
		//   bits 24-31: preamble        (transmitted 1st, as MSB of upper halfword)
		//   bits 16-23: BMC_ZERO_NIBBLE (transmitted 2nd, aux bits 4-7)
		//   bits 8-15:  BMC_ZERO_NIBBLE (transmitted 3rd, aux bits 8-11)
		//   bits 0-7:   bmc_12_15       (transmitted 4th, audio low nibble)
		//
		// word[1] layout:
		//   bits 16-31: bmc_16_23 (transmitted 5th)
		//   bits 0-15:  bmc_24_31 (transmitted 6th)

		// BMC encoding of 4 zero bits starting at phase HIGH: 00_11_00_11 = 0x33
		// Since both aux nibbles (bits 4-7, 8-11) are zero for 16-bit audio and phase is preserved, both are 0x33.
		static constexpr uint32_t BMC_ZERO_NIBBLE = 0x33;    
		static constexpr uint16_t SPDIF_BLOCK_SAMPLES = 192;

		dest[0] = bmc_12_15 | (BMC_ZERO_NIBBLE << 8) | (BMC_ZERO_NIBBLE << 16) | (static_cast<uint32_t>(preamble) << 24);      
		dest[1] = bmc_24_31 | (bmc_16_23 << 16);

		// ============================================================================
		// Update position tracking
		// ============================================================================
		if (!isLeftChannel) 
		{
			// Completed a stereo frame, advance frame counter
			if (++frameInBlock >= SPDIF_BLOCK_SAMPLES) 
				frameInBlock = 0;
		}
	}

    uint16_t hertz;
    bool i2sOn;
    int8_t doutPin;
    uint8_t frame_num;
    size_t _buffers;
    size_t _bufferWords;
    uint8_t channel_status_[24];
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
  void set_stream_name(const std::string name) { stream_name_ = std::move(name); }
  void set_src_ip(const std::string src_ip) { src_ip_ = network::IPAddress(src_ip); }
  void set_idle_timeout_ms(uint32_t ms) { idle_timeout_ms_ = ms; }
  void set_spdif_mode(bool spdif_mode) { spdif_mode_ = spdif_mode; }
  void set_dout_pin(int dout_pin) { dout_pin_ = (gpio_num_t)dout_pin; }
  void set_mclk_pin(int mclk_pin) { mclk_pin_ = (gpio_num_t)mclk_pin; }
  void set_bclk_pin(int bclk_pin) { bclk_pin_ = (gpio_num_t)bclk_pin; }
  void set_lrclk_pin(int lrclk_pin) { lrclk_pin_ = (gpio_num_t)lrclk_pin; }
#ifdef USE_TEXT_SENSOR
  void set_streamname_sensor(text_sensor::TextSensor *streamname_info) { streamname_info_ = streamname_info; }
  void set_src_ip_sensor(text_sensor::TextSensor *src_ip_info) { src_ip_info_ = src_ip_info; }
#endif  // USE_TEXT_SENSOR
#ifdef USE_SENSOR
  void set_samplerate_sensor(sensor::Sensor *samplerate_info) { samplerate_info_ = samplerate_info; }
#endif  // USE_SENSOR


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
		network::IPAddress from_ip((ip_addr_t*)&from.sin_addr.s_addr);

		if (!src_ip_.is_set() || src_ip_ == from_ip) {
		  VBanPacket packet(sockBuff_.data(), n);
		  if (packet.checkValid()) {
		    if (handle_packet_(packet)) {
			  if (from_ip != current_src_ip_)
				update_current_src_ip(std::move(from_ip));
			}
		  }
		  else
		    log_format_warning_("header", 0);
		}
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
 
  bool handle_packet_(const VBanPacket &packet) {
	if (packet.getProtocol() != VBanPacket::VBAN_PROTOCOL_AUDIO) {
	  log_format_warning_("protocol", (unsigned)packet.getProtocol());
	  return false;
	}
	if (packet.getBitrate() != VBanPacket::VBAN_BITFMT_16_INT) {
	  log_format_warning_("format", (unsigned)packet.getBitrate());
	  return false;
	}
	if (packet.getNumChannels() != 2) {
	  log_format_warning_("channels", packet.getNumChannels());
	  return false;
	}
	if (!stream_name_.empty() && !packet.checkStreamName(stream_name_.c_str())) {
	  log_format_warning_("stream", 0);
	  return false;
	}
	if (!packet.checkStreamName(current_stream_name_.c_str())) {
	  packet.getStreamName(current_stream_name_);
	  if (streamname_info_)
		streamname_info_->publish_state(current_stream_name_);
	}
	

	unsigned sr = packet.getSampleRate();
	if (playing()) {
	  if (sr != audioOut->getRate()) {
		audioOut->setRate(sr);
		update_current_samplerate(sr);
	  }
	}
	else {
	  start_playback_(sr);
	  update_current_samplerate(sr);
	}

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
	return true;
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
		if (spdif_mode_) {
			std::unique_ptr<AudioOutputSPDIF> spdif = std::make_unique<AudioOutputSPDIF>(dout_pin_);
			spdif->setBuffers(16, 2*2048);
			audioOut = std::move(spdif);
		}
		else {
			std::unique_ptr<AudioOutputI2S> i2s = std::make_unique<AudioOutputI2S>(dout_pin_, mclk_pin_, bclk_pin_, lrclk_pin_);
			i2s->setBuffers(16, 2048);
			audioOut = std::move(i2s);
		}
	}
	audioOut->setRate(sampleRate);
	audioOut->begin();

    ESP_LOGD("vban_rx", "Stream '%s' started", current_stream_name_.c_str());
  }

  void stop_playback_() {
	audioOut->stop();
	update_current_src_ip(network::IPAddress{});
	current_stream_name_.resize(0);
	if (streamname_info_)
	  streamname_info_->publish_state(current_stream_name_);
	update_current_samplerate(0);
    ESP_LOGD("vban_rx", "Stream idle");
  }
  
  void update_current_samplerate(uint32_t sr) {
	current_samplerate_ = sr;
	if (samplerate_info_)
	  samplerate_info_->publish_state(current_samplerate_);
  }
  
  void update_current_src_ip(network::IPAddress current_src_ip) {
	  current_src_ip_ = std::move(current_src_ip);
	  if (src_ip_info_) {
		char buf[network::IP_ADDRESS_BUFFER_SIZE];
		current_src_ip_.str_to(buf);
		src_ip_info_->publish_state(buf);
	  }
  }

  std::unique_ptr<AudioOutput> audioOut;
  int sock_{-1};
  std::vector<uint8_t> sockBuff_;
  uint16_t listen_port_{6980};
  std::string stream_name_;
  std::string current_stream_name_;
  network::IPAddress src_ip_;
  network::IPAddress current_src_ip_;
  bool spdif_mode_;
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

#ifdef USE_TEXT_SENSOR
  text_sensor::TextSensor *streamname_info_{nullptr};
  text_sensor::TextSensor *src_ip_info_{nullptr};
#endif // USE_TEXT_SENSOR  
#ifdef USE_SENSOR
  sensor::Sensor *samplerate_info_{nullptr};
#endif // USE_SENSOR  
};

}  // namespace vban_receiver
}  // namespace esphome
