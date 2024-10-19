import numpy as np
from scipy.spatial.transform import Rotation as R
from filterpy.kalman import KalmanFilter

# Constants
DELTA_T = 0.01  # Time step in seconds (depends on sensor sampling rate)
G = 9.81  # Gravity in m/s^2

# Initialize the Kalman Filter
kf = KalmanFilter(dim_x=7, dim_z=6)  # 7 state variables (quaternion and angular velocity), 6 measurements (accel and mag)

# State vector [q0, q1, q2, q3, wx, wy, wz] (quaternion + angular velocity)
state = np.zeros(7)
state[0] = 1  # Start with identity quaternion

# State transition matrix (F)
# The gyroscope updates the quaternion, so this is nonlinear and we'll handle it in the predict step.
kf.F = np.eye(7)

# Process noise covariance matrix (Q)
kf.Q = np.eye(7) * 0.001  # Small process noise

# Measurement matrix (H) - maps the quaternion to accelerometer and magnetometer measurements
# This will also be nonlinear, so we will handle it in the update step.
kf.H = np.zeros((6, 7))

# Measurement noise covariance matrix (R) - accounts for sensor noise
kf.R = np.eye(6) * 0.01  # Adjust based on sensor noise

# Initial state covariance (P)
kf.P = np.eye(7) * 1.0  # Initial uncertainty in state

# Function to update quaternion from gyroscope (predict step)
def quaternion_predict(quaternion, gyro, delta_t):
    """Predict the quaternion update based on angular velocity (gyro)."""
    wx, wy, wz = gyro
    gyro_quat = np.array([0, wx, wy, wz])
    
    # Quaternion derivative: dq/dt = 0.5 * q * omega
    dq_dt = 0.5 * quaternion_mult(quaternion, gyro_quat)
    
    # Update quaternion: q_new = q + dq/dt * delta_t
    quaternion += dq_dt * delta_t
    
    # Normalize quaternion to ensure it remains a valid rotation
    quaternion /= np.linalg.norm(quaternion)
    
    return quaternion

# Function to multiply two quaternions
def quaternion_mult(q1, q2):
    """Multiply two quaternions."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    ])

# Function to compute the expected accelerometer and magnetometer readings based on the current quaternion
def expected_measurements(quaternion):
    """Compute the expected gravity and magnetic field vectors from the current quaternion."""
    rotation_matrix = R.from_quat(quaternion).as_matrix()

    # Gravity vector (aligned with Z-axis in the world frame)
    expected_accel = rotation_matrix.T @ np.array([0, 0, G])

    # Magnetometer vector (assumed aligned with the X-axis in the world frame)
    expected_mag = rotation_matrix.T @ np.array([1, 0, 0])  # Example reference vector

    return np.hstack((expected_accel, expected_mag))

# Kalman filter predict and update functions
def kalman_predict(state, gyro, delta_t):
    """Kalman filter predict step with gyroscope (angular velocity)."""
    quaternion = state[:4]
    angular_velocity = state[4:]

    # Predict new quaternion using gyroscope
    quaternion = quaternion_predict(quaternion, gyro, delta_t)

    # Update state
    state[:4] = quaternion
    state[4:] = gyro  # Keep angular velocity as it is for now
    
    kf.predict()
    
    return state

def kalman_update(state, accel_adc, mag_adc, quaternion):
    """Kalman filter update step with accelerometer and magnetometer."""
    # Get expected measurements based on current quaternion
    expected_meas = expected_measurements(quaternion)
    
    # Create measurement vector from accelerometer and magnetometer readings
    measurement = np.hstack((accel_adc, mag_adc))

    # Update Kalman filter with measurements
    kf.update(measurement)
    
    # Update state (quaternion is adjusted based on correction)
    state[:4] = kf.x[:4]  # Update quaternion
    state[4:] = kf.x[4:]  # Update angular velocity
    
    return state

# Example loop to perform sensor fusion using the Kal
