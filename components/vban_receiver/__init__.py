from esphome import pins
import esphome.codegen as cg

from esphome.components.esp32 import (
    add_idf_sdkconfig_option,
    get_esp32_variant,
    include_builtin_idf_component,
)

import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = ["@robertturner"]
DEPENDENCIES = ["network", "esp32"]

AUTO_LOAD = ["socket"]

CONF_VBANRECEIVER_ID = "vban_receiver_id"
vban_receiver_ns = cg.esphome_ns.namespace("vban_receiver")
VBANReceiver = vban_receiver_ns.class_("VBANReceiver", cg.Component)

#CONF_SPEAKER = "speaker"
CONF_LISTEN_PORT = "listen_port"
CONF_STREAM_NAME = "stream_name"
CONF_IDLE_TIMEOUT_MS = "idle_timeout_ms"

CONF_I2S_DOUT_PIN = "i2s_dout_pin"
CONF_I2S_MCLK_PIN = "i2s_mclk_pin"
CONF_I2S_BCLK_PIN = "i2s_bclk_pin"
CONF_I2S_LRCLK_PIN = "i2s_lrclk_pin"

def _consume_sockets(config):
    """Register socket needs for this component."""
    from esphome.components import socket
    socket.consume_sockets(1, "vban_receiver")(config)
    return config

CONFIG_SCHEMA = cv.All(
    cv.Schema({
        cv.GenerateID(): cv.declare_id(VBANReceiver),
        #cv.Required(CONF_SPEAKER): cv.use_id(speaker.Speaker),
        cv.Optional(CONF_LISTEN_PORT, default=6980): cv.port,
        cv.Optional(CONF_STREAM_NAME): cv.All(cv.string, cv.Length(max=16)),
        cv.Optional(CONF_IDLE_TIMEOUT_MS, default=1500): cv.positive_int,
        cv.Required(CONF_I2S_DOUT_PIN): pins.internal_gpio_output_pin_number,
        cv.Optional(CONF_I2S_MCLK_PIN): pins.internal_gpio_output_pin_number,
        cv.Required(CONF_I2S_BCLK_PIN): pins.internal_gpio_output_pin_number,
        cv.Required(CONF_I2S_LRCLK_PIN): pins.internal_gpio_output_pin_number,
    }).extend(cv.COMPONENT_SCHEMA),
    _consume_sockets
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    include_builtin_idf_component("esp_driver_i2s")
    
    add_idf_sdkconfig_option("CONFIG_I2S_ISR_IRAM_SAFE", True)

    cg.add(var.set_listen_port(config[CONF_LISTEN_PORT]))
    if CONF_STREAM_NAME in config:
        cg.add(var.set_stream_name(config[CONF_STREAM_NAME]))
    cg.add(var.set_idle_timeout_ms(config[CONF_IDLE_TIMEOUT_MS]))
    cg.add(var.set_dout_pin(config[CONF_I2S_DOUT_PIN]))
    if CONF_I2S_MCLK_PIN in config:
        cg.add(var.set_mclk_pin(config[CONF_I2S_MCLK_PIN]))
    cg.add(var.set_bclk_pin(config[CONF_I2S_BCLK_PIN]))
    cg.add(var.set_lrclk_pin(config[CONF_I2S_LRCLK_PIN]))
    
    
    
