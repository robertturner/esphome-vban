import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    UNIT_HERTZ,
)

from . import (  # noqa: F401  pylint: disable=unused-import
    CONF_VBANRECEIVER_ID,
    VBANReceiver,
)


CONF_SAMPLERATE = "sample_rate"

DEPENDENCIES = ["vban_receiver"]

CONFIG_SCHEMA = {
    cv.GenerateID(CONF_VBANRECEIVER_ID): cv.use_id(VBANReceiver),
    cv.Optional(CONF_SAMPLERATE): sensor.sensor_schema(
        unit_of_measurement=UNIT_HERTZ,
        icon="mdi:speedometer",
        accuracy_decimals=0,
        entity_category="diagnostic",
    ),
}


async def to_code(config):
    vban_receiver_component = await cg.get_variable(config[CONF_VBANRECEIVER_ID])

    if samplerate_conf := config.get(CONF_SAMPLERATE):
        sens = await sensor.new_sensor(samplerate_conf)
        cg.add(vban_receiver_component.set_samplerate_sensor(sens))

