import time
import board, busio
import adafruit_lsm6ds.lsm6ds3trc as LSM6DS

i2c = busio.I2C(board.SCL, board.SDA)
sensor = LSM6DS.LSM6DS3TRC(i2c)

try:
    while True:
        # Gyro values in radians/sec
        gx, gy, gz = sensor.gyro
        print(f"Gyro: x={gx:.3f}, y={gy:.3f}, z={gz:.3f}")
        time.sleep(0.2)  # adjust speed (0.2s = 5 Hz)
except KeyboardInterrupt:
    print("\nStopped by user.")