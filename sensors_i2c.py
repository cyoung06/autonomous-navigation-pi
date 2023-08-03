from smbus2 import i2c_msg

class Sensors:
    def __init__(self, bus, id2):
        self.bus = bus
        self.id2 = id2

    def readUltra(self):
        msg = i2c_msg.read(self.id2, 6)

        front = msg.buf[0] * 255 + msg.buf[1]
        left = msg.buf[2] * 255 + msg.buf[3]
        right = msg.buf[4] * 255 + msg.buf[5]
        return {
            "front": front / 100.0,
            "left": left / 100.0,
            "right": right / 100.0
        }
