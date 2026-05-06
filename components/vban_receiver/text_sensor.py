import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv

from . import (  # noqa: F401  pylint: disable=unused-import
    CONF_VBANRECEIVER_ID,
    VBANReceiver,
)

DEPENDENCIES = ["vban_receiver"]

CONF_STREAM_NAME = "stream_name"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_VBANRECEIVER_ID): cv.use_id(VBANReceiver),
        cv.Optional(CONF_STREAM_NAME): text_sensor.text_sensor_schema(
            icon="mdi:radio_tower",
            entity_category="diagnostic",
        ),
    }
)


async def to_code(config):
    vban_receiver_component = await cg.get_variable(config[CONF_VBANRECEIVER_ID])

    if CONF_STREAM_NAME in config:
        sens = await text_sensor.new_text_sensor(config[CONF_STREAM_NAME])
        cg.add(vban_receiver_component.set_streamname_sensor(sens))

