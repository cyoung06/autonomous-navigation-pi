from smbus2 import i2c_msg

class Sensors:
    def __init__(self, bus, id2):
        self.bus = bus
        self.id2 = id2

    def readUltra(self):
        msg = i2c_msg.read(self.id2, 8)
        self.bus.i2c_rdwr(msg)
        data = list(msg)
        front = data[0] * 256 + data[1]
        right = data[2] * 256 + data[3]
        left = data[4] * 256 + data[5]
        forward = data[6] * 256 + data[7]
        return {
            "front": front / 100.0,
            "left": left / 100.0,
            "right": right / 100.0,
            "forward": forward / 100.0
        }
