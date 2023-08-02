# this is to be saved in the local folder under the name "mpu9250_i2c.py"
# it will be used as the I2C controller and function harbor for the project
# refer to datasheet and register map for full explanation

class Sensors:
    def __init__(self, bus, id):
        self.bus = bus
        self.id = id

    def readUltra(self):
        front = self.bus.read_word_data(id, 0)
        right = self.bus.read_word_data(id, 0)
        left = self.bus.read_word_data(id, 0)
        return {
            "front": front,
            "left": left,
            "right": right
        }
