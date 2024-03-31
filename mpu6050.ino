#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

#define RAD_TO_DEG 57.2957795131
#define complementaryFilterFactor 0.98 // Adjust this factor for balance between gyroscope and accelerometer data

void setup() {
  Serial.begin(9600);
  
  Wire.begin();
  mpu.initialize();
  
  // Set gyro sensitivity (adjust as necessary)
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
}

void loop() {
  // Read raw accelerometer and gyroscope data
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Calculate accelerometer angles (pitch and roll)
  float accRoll = atan2(ay, az) * RAD_TO_DEG;
  float accPitch = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;

  // Convert gyroscope readings to degrees per second
  float gyroRoll = gx / 131.0; // MPU6050 gyro scaling factor for 250 degrees/sec range
  float gyroPitch = gy / 131.0; // MPU6050 gyro scaling factor for 250 degrees/sec range

  // Apply complementary filter to combine accelerometer and gyroscope data
  float roll = complementaryFilterFactor * (roll + gyroRoll * dt) + (1 - complementaryFilterFactor) * accRoll;
  float pitch = complementaryFilterFactor * (pitch + gyroPitch * dt) + (1 - complementaryFilterFactor) * accPitch;

  // Calculate heading direction (yaw)
  float heading = atan2(-ay, ax) * RAD_TO_DEG;

  // Ensure heading is within [0, 360] degrees
  if (heading < 0) heading += 360.0;

  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print(" Pitch: ");
  Serial.print(pitch);
  Serial.print(" Heading: ");
  Serial.println(heading);

  delay(10); // Adjust delay as needed for sampling rate
}
