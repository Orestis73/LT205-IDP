from machine import I2C, Pin
from libs.VNCL4010.VNCL4010 import VCNL4010

# Distance sensor module class for VCNL4010 proximity sensor over I2C
class ProximityVCNL4010:

    #configure I2C and initialize the VCNL4010 sensor
    def __init__(self, i2c_id: int, scl_pin: int, sda_pin: int, freq_hz: int = 100_000):
        self.i2c = I2C(i2c_id, scl=Pin(scl_pin), sda=Pin(sda_pin), freq=freq_hz)
        self.sensor = VCNL4010(self.i2c)
    
    #read proximity value from the sensor
    def read(self) -> int:
        return self.sensor.read_proximity()

