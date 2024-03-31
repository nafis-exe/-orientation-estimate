import numpy as np
import math

class MadgwickAHRS:
    def __init__(self, beta=0.1):
        self.beta = beta  # Beta parameter for algorithm

        # Quaternion of sensor frame relative to auxiliary frame
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

    def update(self, gyroscope, accelerometer, dt):
        gx, gy, gz = gyroscope
        ax, ay, az = accelerometer

        q1, q2, q3, q4 = self.q

        # Rate of change of quaternion from gyroscope
        qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz)
        qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy)
        qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx)
        qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx)

        # Compute feedback only if accelerometer measurement valid
        if not (ax == 0.0 and ay == 0.0 and az == 0.0):
            # Normalise accelerometer measurement
            recipNorm = 1.0 / math.sqrt(ax * ax + ay * ay + az * az)
            ax *= recipNorm
            ay *= recipNorm
            az *= recipNorm

            # Auxiliary variables to avoid repeated arithmetic
            _2q1 = 2.0 * q1
            _2q2 = 2.0 * q2
            _2q3 = 2.0 * q3
            _2q4 = 2.0 * q4
            _4q1 = 4.0 * q1
            _4q2 = 4.0 * q2
            _4q3 = 4.0 * q3
            _8q2 = 8.0 * q2
            _8q3 = 8.0 * q3
            q1q1 = q1 * q1
            q2q2 = q2 * q2
            q3q3 = q3 * q3
            q4q4 = q4 * q4

            # Gradient decent algorithm corrective step
            s1 = _4q1 * q3q3 + _2q4 * ax + _4q1 * q2q2 - _2q2 * ay
            s2 = _4q2 * q4q4 - _2q1 * ax + _4q2 * q3q3 + _2q4 * ay
            s3 = 4.0 * q3q3 * q1 - _2q2 * ax + 4.0 * q2q2 * q3 - _2q4 * ay
            s4 = 4.0 * q4q4 * q2 + _2q1 * ax + 4.0 * q3q3 * q4 + _2q3 * ay
            recipNorm = 1.0 / math.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)  # normalise step magnitude
            s1 *= recipNorm
            s2 *= recipNorm
            s3 *= recipNorm
            s4 *= recipNorm

            # Apply feedback step
            qDot1 -= self.beta * s1
            qDot2 -= self.beta * s2
            qDot3 -= self.beta * s3
            qDot4 -= self.beta * s4

        # Integrate rate of change of quaternion to yield quaternion
        q1 += qDot1 * dt
        q2 += qDot2 * dt
        q3 += qDot3 * dt
        q4 += qDot4 * dt

        # Normalise quaternion
        recipNorm = 1.0 / math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)
        self.q = np.array([q1 * recipNorm, q2 * recipNorm, q3 * recipNorm, q4 * recipNorm])

    def get_orientation(self):
        roll = math.atan2(2.0 * (self.q[0] * self.q[1] + self.q[2] * self.q[3]),
                          1.0 - 2.0 * (self.q[1] ** 2 + self.q[2] ** 2))
        pitch = math.asin(2.0 * (self.q[0] * self.q[2] - self.q[3] * self.q[1]))
        yaw = math.atan2(2.0 * (self.q[0] * self.q[3] + self.q[1] * self.q[2]),
                         1.0 - 2.0 * (self.q[2] ** 2 + self.q[3] ** 2))
        return roll, pitch, yaw


# Example usage
if __name__ == "__main__":
    # Sample data
    gyroscope = [0.1, 0.2, -0.1]  # Angular velocity in radians per second
    accelerometer = [0.0, 0.0, 9.81]  # Acceleration in m/s^2
    dt = 0.01  # Time step in seconds

    # Initialize AHRS object
    madgwick = MadgwickAHRS()

    # Update orientation estimate
    madgwick.update(gyroscope, accelerometer, dt)

    # Get orientation (roll, pitch, yaw)
    roll, pitch, yaw = madgwick.get_orientation()

    # Print the result
    print("Roll:", math.degrees(roll))
    print("Pitch:", math.degrees(pitch))
    print("Yaw:", math.degrees(yaw))
