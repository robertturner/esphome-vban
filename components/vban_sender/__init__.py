import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import microphone
from esphome.const import CONF_ID, CONF_MICROPHONE

CODEOWNERS = ["@powange"]
DEPENDENCIES = ["wifi", "network", "microphone"]
AUTO_LOAD = []

vban_sender_ns = cg.esphome_ns.namespace("vban_sender")
VBANSender = vban_sender_ns.class_("VBANSender", cg.Component)

CONF_TARGET_IP = "target_ip"
CONF_TARGET_PORT = "target_port"
CONF_STREAM_NAME = "stream_name"
CONF_GAIN = "gain"

vban_stream_name = cv.All(cv.string, cv.Length(max=16))

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(VBANSender),
    cv.Required(CONF_MICROPHONE): cv.use_id(microphone.Microphone),
    cv.Required(CONF_TARGET_IP): cv.string,
    cv.Optional(CONF_TARGET_PORT, default=6980): cv.port,
    cv.Optional(CONF_STREAM_NAME, default="AtomEcho"): vban_stream_name,
    cv.Optional(CONF_GAIN, default=1.0): cv.float_range(min=0.0),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    mic = await cg.get_variable(config[CONF_MICROPHONE])
    cg.add(var.set_microphone(mic))

    cg.add(var.set_target_ip(config[CONF_TARGET_IP]))
    cg.add(var.set_target_port(config[CONF_TARGET_PORT]))
    cg.add(var.set_stream_name(config[CONF_STREAM_NAME]))
    cg.add(var.set_gain(config[CONF_GAIN]))
