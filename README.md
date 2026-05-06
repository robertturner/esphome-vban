# esphome-vban

ESPHome component for receiving VBAN (UDP audio) on an ESP32. `vban_receiver` (network → speaker).

## Installation

```yaml
external_components:
  - source: github://robertturner/esphome-vban
    components: [vban_receiver]
    refresh: 0s
```

## `vban_receiver`

Listens to a UDP port, filters by stream name, and pushes the received PCM to custom low-latency I2S driver.

```yaml
vban_receiver:
  listen_port: 6980				# optional, defaults to 6980
  stream_name: "TTS1"           # 16 chars max. Optional. If empty or not defined then accepts any incoming stream. If non-empty then only accepts that stream name
  idle_timeout_ms: 1500         # Disables output after timeout
  i2s_dout_pin: GPIO5			
  i2s_lrclk_pin: GPIO6			
  i2s_bclk_pin: GPIO7			
  i2s_mclk_pin: GPIO8			
```

| Parameter          | Required | Défaut | Description 								|
|--------------------|----------|--------|------------------------------------------|
| `listen_port`      | ❌       | `6980` | Port UDP listening 						|
| `stream_name`      | ❌       | —      | Stream to accept, or any (16 chars max)	|
| `idle_timeout_ms`  | ❌       | `1500` | Duration without packets before `stop()` |
| `i2s_dout_pin`     | ✅       | —      | I2S DOUT pin 							|
| `i2s_lrclk_pin`    | ✅       | —      | I2S LRCLK pin 							|
| `i2s_bclk_pin`     | ✅       | —      | I2S BCLK pin 							|
| `i2s_mclk_pin`     | ❌       | —      | I2S MCLK pin 							|

## Format audio supported

I2S sample rate automatically updated according to incoming VBAN stream :

- Sample rate : **16000 to 192kHz**
- Format     : **PCM 16-bit signed**
- Channels   : **2 (stero)**

