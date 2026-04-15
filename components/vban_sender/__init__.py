import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = ["@powange"]
DEPENDENCIES = ["wifi", "network"]
AUTO_LOAD = []

vban_sender_ns = cg.esphome_ns.namespace("vban_sender")
VBANSender = vban_sender_ns.class_("VBANSender", cg.Component)

CONF_TARGET_IP = "target_ip"
CONF_TARGET_PORT = "target_port"
CONF_STREAM_NAME = "stream_name"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(VBANSender),
    cv.Required(CONF_TARGET_IP): cv.string,
    cv.Optional(CONF_TARGET_PORT, default=6980): cv.port,
    cv.Optional(CONF_STREAM_NAME, default="AtomEcho"): cv.string,
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_target_ip(config[CONF_TARGET_IP]))
    cg.add(var.set_target_port(config[CONF_TARGET_PORT]))
    cg.add(var.set_stream_name(config[CONF_STREAM_NAME]))
