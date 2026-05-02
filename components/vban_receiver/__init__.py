import esphome.codegen as cg

from esphome.components.esp32 import (
    add_idf_sdkconfig_option,
    get_esp32_variant,
    include_builtin_idf_component,
)

import esphome.config_validation as cv
#from esphome.components import speaker
from esphome.const import CONF_ID

CODEOWNERS = ["@powange"]
#DEPENDENCIES = ["network", "speaker"]
DEPENDENCIES = ["network", "esp32"]

vban_receiver_ns = cg.esphome_ns.namespace("vban_receiver")
VBANReceiver = vban_receiver_ns.class_("VBANReceiver", cg.Component)

#CONF_SPEAKER = "speaker"
CONF_LISTEN_PORT = "listen_port"
CONF_STREAM_NAME = "stream_name"
CONF_IDLE_TIMEOUT_MS = "idle_timeout_ms"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(VBANReceiver),
    #cv.Required(CONF_SPEAKER): cv.use_id(speaker.Speaker),
    cv.Optional(CONF_LISTEN_PORT, default=6980): cv.port,
    cv.Required(CONF_STREAM_NAME): cv.All(cv.string, cv.Length(max=16)),
    cv.Optional(CONF_IDLE_TIMEOUT_MS, default=1500): cv.positive_int,
}).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    include_builtin_idf_component("esp_driver_i2s")
    
    add_idf_sdkconfig_option("CONFIG_I2S_ISR_IRAM_SAFE", True)

    #spk = await cg.get_variable(config[CONF_SPEAKER])
    #cg.add(var.set_speaker(spk))

    cg.add(var.set_listen_port(config[CONF_LISTEN_PORT]))
    cg.add(var.set_stream_name(config[CONF_STREAM_NAME]))
    cg.add(var.set_idle_timeout_ms(config[CONF_IDLE_TIMEOUT_MS]))
