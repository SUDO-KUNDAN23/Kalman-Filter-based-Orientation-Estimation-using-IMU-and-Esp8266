#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp;
#define SEA_LEVEL_PRESSURE_HPA 1013.25
float altitude_estimate = 0.0;
float error_estimate = 1.0;
float error_measurement = 1.0; // Tune this based on noise
float kalman_gain;

#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 150  // pulse length for 0°
#define SERVOMAX 600  // pulse length for 180°

#define SERVO1 0  // Front Left
#define SERVO2 1  // Front Right
#define SERVO3 2  // Rear Left
#define SERVO4 3  // Rear Right


// OLED config
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// MPU6050 and HMC5883L
MPU6050 mpu;
#define HMC5883L_ADDR 0x1E 
#define MAG_SCALE 0.92   // HMC5883L I2C address

double magMinX = 0, magMaxX = 0, magMinY = 0, magMaxY = 0, magMinZ = 0, magMaxZ = 0; 

// Constants
double pitch = 0, roll = 0, yaw = 0;  // Orientation angles (degrees)
   

// Kalman filter constants
float Q_angle = 0.001, Q_bias = 0.003, R_measure = 0.03;
float angleX = 0.0, biasX = 0.0;
float angleY = 0.0, biasY = 0.0;
float P_X[2][2] = {{0, 0}, {0, 0}};
float P_Y[2][2] = {{0, 0}, {0, 0}};

unsigned long timer;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50); 
  testServos(); 
 
  while ( !Serial ) delay(100);   // wait for native usb
  // Serial.println(F("BMP280 test"));
  unsigned status;
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = bmp.begin(0x76);
  if (!status) {
    // Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  
  HMC5883L_init();
  calibrate_HMC5883L(); // Calibrate magnetometer

  mpu.initialize();
  if (!mpu.testConnection()) {
    // Serial.println("MPU6050 connection failed!");
    while (1);
  }

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    // Serial.println("OLED failed");
    while (1);
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  delay(1000);
  timer = millis();
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  double mx, my, mz;    
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  read_HMC5883L(mx, my, mz);   
 
  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;

  float gyroX = gx / 131.0;
  float gyroY = gy / 131.0;
  float gyroz = gz / 131.0;
  
  float dt = (millis() - timer) / 1000.0;
  timer = millis();

  float rollAcc  = atan2(accelY, accelZ) * 180.0 / PI;
  float pitchAcc = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;

  float roll  = kalmanFilter(rollAcc, gyroX, dt, &angleX, &biasX, P_X);
  float pitch = kalmanFilter(pitchAcc, gyroY, dt, &angleY, &biasY, P_Y);

  // Magnetometer-based yaw (tilt-compensated)
  yaw = atan2(my, mx); // Calculate yaw from magnetometer readings
  float declinationAngle =0.00436; // Declination in radians for -10° 13'
  yaw += declinationAngle;  

  

// Normalize to 0–360°
  if (yaw < 0) yaw += 2 * PI;
  if (yaw > 2 * PI) yaw -= 2 * PI;
  yaw = yaw * 180.0 / PI;
  controlServos(roll, pitch,yaw);
  float filtered_altitude = kalmanFilterAltitude();
  float alti=0;
  float lo=0;
  transmitTelemetry(accelX, accelY, accelZ, gyroX, gyroY, gyroz,filtered_altitude, pitch, roll, yaw,lo,alti);
  // OLED Display
  display.clearDisplay();
  display.setCursor(0, 10);
  display.print("Roll : ");
  display.print(roll, 1);
  display.print(" deg");

  display.setCursor(0, 25);
  display.print("Pitch: ");
  display.print(pitch, 1);
  display.print(" deg");

  display.setCursor(0, 40);  // ← Fixed from "400"
  display.print("Yaw  : ");
  display.print(yaw, 1);
  display.print(" deg");
 
  display.setTextSize(1);
  display.setCursor(0, 50);
  display.print("Alt: ");
  display.print(filtered_altitude);
  display.println(" m");

  display.display();
  delay(10);
  

  

}

// ---------------- Kalman Filter ------------------
float kalmanFilter(float newAngle, float newRate, float dt, float* angle, float* bias, float P[2][2]) {
  float rate = newRate - *bias;
  *angle += dt * rate;

  P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  float S = P[0][0] + R_measure;
  float K0 = P[0][0] / S;
  float K1 = P[1][0] / S;

  float y = newAngle - *angle;
  *angle += K0 * y;
  *bias += K1 * y;

  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= K0 * P00_temp;
  P[0][1] -= K0 * P01_temp;
  P[1][0] -= K1 * P00_temp;
  P[1][1] -= K1 * P01_temp;

  return *angle;
}

// ---------------- Tilt-Compensated Yaw ------------------
void HMC5883L_init() {
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x00); // Configuration Register A
  Wire.write(0x70); // 8-average, 15 Hz default, normal measurement
  Wire.endTransmission();
 
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x01); // Configuration Register B
  Wire.write(0x20); // Gain = 5 (±1.3 Gauss)
  Wire.endTransmission();
 
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x02); // Mode Register
  Wire.write(0x00); // Continuous measurement mode
  Wire.endTransmission();
}

void calibrate_HMC5883L() {
  // Serial.println("Calibrating HMC5883L... Rotate the sensor in all directions.");
  double mx, my, mz;
  
  magMinX = magMinY = magMinZ = 1e6;
  magMaxX = magMaxY = magMaxZ = -1e6;
 
  unsigned long startTime = millis();
  while (millis() - startTime < 10000) { // 10 seconds for calibration
    read_HMC5883L(mx, my, mz);
 
    if (mx < magMinX) magMinX = mx;
    if (mx > magMaxX) magMaxX = mx;
    if (my < magMinY) magMinY = my;
    if (my > magMaxY) magMaxY = my;
    if (mz < magMinZ) magMinZ = mz;
    if (mz > magMaxZ) magMaxZ = mz;
 
    Serial.print("."); // Print a dot every 100 ms to indicate progress
    delay(100);
  }
  // Serial.println("\nHMC5883L Calibration Complete.");
}
 
void read_HMC5883L(double &mx, double &my, double &mz) {
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x03); // Starting register for magnetometer readings
  Wire.endTransmission(false);
  Wire.requestFrom(HMC5883L_ADDR, 6, true);
 
  int16_t x = Wire.read() << 8 | Wire.read();
  int16_t z = Wire.read() << 8 | Wire.read();
  int16_t y = Wire.read() << 8 | Wire.read();
 
  // Apply calibration offsets and scaling
  mx = (x - (magMinX + magMaxX) / 2) * MAG_SCALE;
  my = (y - (magMinY + magMaxY) / 2) * MAG_SCALE;
  mz = (z - (magMinZ + magMaxZ) / 2) * MAG_SCALE;
}
int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}
void controlServos(float roll, float pitch,float yaw) {
  // Limit roll and pitch to ±30°
  roll = constrain(roll, -30, 30);
  pitch = constrain(pitch, -30, 30);
  float yaw_setpoint = 0;              // Desired heading
  float yaw_error = yaw - yaw_setpoint;
  
  if (yaw_error > 180)  yaw_error -= 360;
  if (yaw_error < -180) yaw_error += 360;

  float yaw_control = constrain(yaw_error, -30, 30);

  // Convert to servo offset (-30 to +30 → 60 to 120)
  int base = 90;

  int servo1 = base + pitch;  // Front Left
  int servo2 = base -pitch ;  // Front Right
  int servo3 = base + roll ;  // Rear Left
  int servo4 = base - roll ; 
  
  servo1 = constrain(servo1, 45, 135);
  servo2 = constrain(servo2, 45, 135);
  servo3 = constrain(servo3, 45, 135);
  servo4 = constrain(servo4, 45, 135);
 

  pwm.setPWM(SERVO1, 0, angleToPulse(servo1));
  pwm.setPWM(SERVO2, 0, angleToPulse(servo2));
  pwm.setPWM(SERVO3, 0, angleToPulse(servo3));
  pwm.setPWM(SERVO4, 0, angleToPulse(servo4));

  
}
float kalmanFilterAltitude() {
  float pressure = bmp.readPressure() / 100.0F; // Convert Pa to hPa
  float raw_altitude = 44330.0 * (1.0 - pow(pressure / SEA_LEVEL_PRESSURE_HPA, 1.0 / 5.255));

  kalman_gain = error_estimate / (error_estimate + error_measurement);
  altitude_estimate = altitude_estimate + kalman_gain * (raw_altitude - altitude_estimate);
  error_estimate = (1.0 - kalman_gain) * error_estimate;

  return altitude_estimate;
}
void testServos() {
  //Serial.println("Testing servos...");

  for (int angle = 60; angle <= 120; angle += 5) {
    pwm.setPWM(SERVO1, 0, angleToPulse(angle));
    pwm.setPWM(SERVO2, 0, angleToPulse(angle));
    pwm.setPWM(SERVO3, 0, angleToPulse(angle));
    pwm.setPWM(SERVO4, 0, angleToPulse(angle));
    delay(100);
  }

  for (int angle = 120; angle >= 60; angle -= 5) {
    pwm.setPWM(SERVO1, 0, angleToPulse(angle));
    pwm.setPWM(SERVO2, 0, angleToPulse(angle));
    pwm.setPWM(SERVO3, 0, angleToPulse(angle));
    pwm.setPWM(SERVO4, 0, angleToPulse(angle));
    delay(100);
  }

  // Reset to neutral
  int neutral = 90;
  pwm.setPWM(SERVO1, 0, angleToPulse(neutral));
  pwm.setPWM(SERVO2, 0, angleToPulse(neutral));
  pwm.setPWM(SERVO3, 0, angleToPulse(neutral));
  pwm.setPWM(SERVO4, 0, angleToPulse(neutral));

  //Serial.println("Servo test complete.");
}
void transmitTelemetry(float ax, float ay, float az,
                       float gx, float gy, float gz,
                       float altitude, float pitch, float roll, float yaw,float longi ,float alt) {
  Serial.print("TELEMETRY,");
  Serial.print(ax); Serial.print(",");
  Serial.print(ay); Serial.print(",");
  Serial.print(az); Serial.print(",");
  Serial.print(gx); Serial.print(",");
  Serial.print(gy); Serial.print(",");
  Serial.print(gz); Serial.print(",");
  Serial.print(altitude); Serial.print(",");
  Serial.print(pitch); Serial.print(",");
  Serial.print(roll); Serial.print(",");
  Serial.print(yaw); Serial.print(",");
  Serial.print(longi); Serial.print(",");
  Serial.println(alt);
}



