from smbus2 import i2c_msg

class Sensors:
    def __init__(self, bus, id2):
        self.bus = bus
        self.id2 = id2

    def readUltra(self):
        msg = i2c_msg.read(self.id2, 6)
        self.bus.i2c_rdwr(msg)

        front = msg.buf[0] * 255 + msg.buf[1]
        right = msg.buf[2] * 255 + msg.buf[3]
        left = msg.buf[4] * 255 + msg.buf[5]
        print(front)
        print(left)
        print(right)
        return {
            "front": front / 100.0,
            "left": left / 100.0,
            "right": right / 100.0
        }
