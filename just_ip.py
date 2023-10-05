import time

import smbus2
from peripherals.lcd import LiquidCrystal
import ip_utils

bus = smbus2.SMBus(1)
lcd = LiquidCrystal(bus)

lcd.backlight(1)

while True:
    lcd.lcd_display_string(ip_utils.wifiname()+"          ", 1, 0)
    lcd.lcd_display_string(ip_utils.get_ip()+"          ", 2, 0)
    time.sleep(1)