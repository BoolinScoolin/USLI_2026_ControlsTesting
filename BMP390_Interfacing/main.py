import time
import board
import adafruit_bmp3xx
i2c = board.I2C()
bmp = adafruit_bmp3xx.BMP3XX_I2C(i2c)


while True:

    print("Pressure: {:6.1f}".format(bmp.pressure))
    print("Temperature: {:5.2f}".format(bmp.temperature))

    time.sleep(0.2)