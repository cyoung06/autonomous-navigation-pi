# this is to be saved in the local folder under the name "mpu9250_i2c.py"
# it will be used as the I2C controller and function harbor for the project
# refer to datasheet and register map for full explanation

class Sensors:
    def __init__(self, bus, id2):
        self.bus = bus
        self.id2 = id2

    def readUltra(self):
        front = self.bus.read_word_data(self.id2, 0)
        right = self.bus.read_word_data(self.id2, 0)
        left = self.bus.read_word_data(self.id2, 0)
        return {
            "front": front,
            "left": left,
            "right": right
        }
