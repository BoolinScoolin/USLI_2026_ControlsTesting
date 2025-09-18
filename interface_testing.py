import socket  # used for live streaming through TCP
import struct
import time  # used for delays
import board, busio  # circuitpython
import adafruit_lsm6ds.lsm6ds3trc as LSM6DS  # adafruit
import numpy as np


# setup i2c bus
i2c = busio.I2C(board.SCL, board.SDA)

# IMU sensor object
sensor = LSM6DS.LSM6DS3TRC(i2c)

HOST = "192.168.1.1"   # laptop's ethernet static ip address
PORT = 5000  # idk just an arbitary port number I believe

# Create socket object to speak over TCP
# (AF_INET -> AddressFamily IPv4, SOCK_STREAM -> TCP Stream)
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))

# Initialize time
last_time = time.time()

# AEROSPACE STUFF

# Constants
d2r = np.pi/180
r2d = 1/d2r

# Initialize Orientation w/ Euler Angles
phi_rad = 180*d2r
theta_rad = 0*d2r
psi_rad = 0*d2r

# Initialize quaternion
q0 = ( np.cos(psi_rad/2)*np.cos(theta_rad/2)*np.cos(phi_rad/2)
        + np.sin(psi_rad/2)*np.sin(theta_rad/2)*np.sin(phi_rad/2))
q1 = ( np.cos(psi_rad/2)*np.cos(theta_rad/2)*np.sin(phi_rad/2)
        - np.sin(psi_rad/2)*np.sin(theta_rad/2)*np.cos(phi_rad/2))
q2 = ( np.cos(psi_rad/2)*np.sin(theta_rad/2)*np.cos(phi_rad/2)
        + np.sin(psi_rad/2)*np.cos(theta_rad/2)*np.sin(phi_rad/2))
q3 = ( np.sin(psi_rad/2)*np.cos(theta_rad/2)*np.cos(phi_rad/2)
        - np.cos(psi_rad/2)*np.sin(theta_rad/2)*np.sin(phi_rad/2))

q = np.array([q0, q1, q2, q3])

try:
    while True:

        # Time math
        now = time.time()
        dt = now - last_time
        last_time = now
        
        # Extract Gyro Data
        p_rps, q_rps, r_rps = sensor.gyro

        print(p_rps, q_rps, r_rps)

        # Form Quaternion Kinematic Evolution Matrix
        omega_matrix = np.array([
                                [0,      -p_rps,  -q_rps,  -r_rps],
                                [p_rps,   0,       r_rps,  -q_rps],
                                [q_rps,  -r_rps,   0,       p_rps],
                                [r_rps,   q_rps,  -p_rps,   0]
                                ])

        # Lambda Correction Factor (Modelling and Simulation, P. Zipfel)
        lamb = 1 - (q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2)
        k = 1/dt

        # Compute State Derivative
        qdot = 0.5 * omega_matrix @ q

        # Update Quaternion
        q = q + qdot*dt
        q = q / np.linalg.norm(q)

        # Format Output
        packet = struct.pack("ffff", q[0], q[1], q[2], q[3])
        sock.sendall(packet)

        # Delay
        time.sleep(0.05)
except KeyboardInterrupt:
    sock.close()