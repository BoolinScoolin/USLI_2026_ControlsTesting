import time
print('1')

import board
print('2')

import adafruit_bmp3xx
print('3')

i2c = board.I2C()
print('4')

bmp = adafruit_bmp3xx.BMP3XX_I2C(i2c)
print('5')


# while True:

print("Pressure: {:6.1f}".format(bmp.pressure))
print("Temperature: {:5.2f}".format(bmp.temperature))

#    time.sleep(0.2)

