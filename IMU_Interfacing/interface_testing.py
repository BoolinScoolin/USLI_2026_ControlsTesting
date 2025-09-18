import socket  # used for live streaming through TCP
import struct
import time  # used for delays
import board, busio  # circuitpython
import adafruit_lsm6ds.lsm6ds3trc as LSM6DS  # adafruit
import numpy as np
from quaternion import *

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
d2r = np.pi/180  # degrees to radians
r2d = 1/d2r  # radians to degrees
q2qstar = np.array([1,-1,-1,-1])  # quaternion transpose

# Initialize Position
r_n_m = np.array([0,0,0])
v_n_mps = np.array([0,0,0])

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

q = Quaternion(q)


# Tare
tare_size = 1000 # Number of measurements in the tare
tare_measurements_gyro = np.zeros([tare_size,3])
tare_measurements_accel = np.zeros([tare_size,3])
for i in range(tare_size):
    p_rps, q_rps, r_rps = sensor.gyro
    ax_mps2, ay_mps2, az_mps2 = sensor.acceleration
    tare_measurements_gyro[i] = [p_rps, q_rps, r_rps]
    tare_measurements_accel[i] = [ax_mps2, ay_mps2, az_mps2]
p_tare_rps, q_tare_rps, r_tare_rps = np.mean(tare_measurements_gyro, axis=0)
ax_mps2_tare, ay_mps2_tare, az_mps2_tare = np.mean(tare_measurements_accel, axis=0)

try:
    while True:

        # Time math
        now = time.time()
        dt = now - last_time
        last_time = now
        
        # Extract Gyro Data
        p_rps, q_rps, r_rps = sensor.gyro

        # Subtract Tare
        p_rps = p_rps - p_tare_rps
        q_rps = q_rps - q_tare_rps
        r_rps = r_rps - r_tare_rps

        # Print readings (debug)
        # print(p_rps, q_rps, r_rps)

        # Form Quaternion Kinematic Evolution Matrix
        omega_matrix = np.array([
                                [0,      -p_rps,  -q_rps,  -r_rps],
                                [p_rps,   0,       r_rps,  -q_rps],
                                [q_rps,  -r_rps,   0,       p_rps],
                                [r_rps,   q_rps,  -p_rps,   0]
                                ])

        # Lambda Correction Factor (Modelling and Simulation, P. Zipfel)
        lamb = 1 - (q.array[0]**2 + q.array[1]**2 + q.array[2]**2 + q.array[3]**2)
        k = 1/dt

        # Compute State Derivative
        qdot = 0.5 * omega_matrix @ q.array

        # Forward Euler Integration
        deltaq = qdot*dt

        # Store in Quaternion object
        deltaq = Quaternion(deltaq)

        # Update Quaternion
        q = q + deltaq

        # Normalize Quaternion
        q = q.normalize()

        # Read Accelerometer Data
        ax_b_mps2, ay_b_mps2, az_b_mps2 = sensor.acceleration

        # Subtract Tare
        ax_b_mps2 = ax_b_mps2 - ax_mps2_tare
        ay_b_mps2 = ay_b_mps2 - ay_mps2_tare
        az_b_mps2 = az_b_mps2 - az_mps2_tare

        # Print Tared Readings (debug)
        print(f"ax={ax_b_mps2:.3f}, ay={ay_b_mps2:.3f}, az={az_b_mps2:.3f}")

        # Store acceleration as quaternion object
        aquat_b_mps2 = Quaternion([0, ax_b_mps2, ay_b_mps2, az_b_mps2])

        # Rotate accelerations to body frame
        temp_quat = quat_mult(q.conjugate(), aquat_b_mps2)
        aquat_n_mps2 = quat_mult(temp_quat,q)

        # Extract acceleration in navigation frame
        a_n_mps2 = aquat_n_mps2.vec

        # Euler integration
        v_n_mps = v_n_mps + a_n_mps2*dt
        r_n_m = r_n_m + v_n_mps*dt

        # Extract r and v
        rx_n_m = r_n_m[0]
        ry_n_m = r_n_m[1]
        rz_n_m = r_n_m[2]
        vx_n_mps = v_n_mps[0]
        vy_n_mps = v_n_mps[1]
        vz_n_mps = v_n_mps[2]

        # Format Output
        packet = struct.pack("ffffffffff", q.array[0], q.array[1], q.array[2], q.array[3],
                             rx_n_m, ry_n_m, rz_n_m, vx_n_mps, vy_n_mps, vz_n_mps)
        sock.sendall(packet)

        # Delay
        time.sleep(0.05)
except KeyboardInterrupt:
    sock.close()