#define BLYNK_TEMPLATE_ID   "Your_template_id"
#define BLYNK_TEMPLATE_NAME "Balancingrobot"
#define BLYNK_AUTH_TOKEN    "Your_blynk_auth_token"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <MPU6050_light.h>
#include <Wire.h>

char ssid[] = "Your_Wifi_SSID";
char pass[] = "Your_Wifi_password";

MPU6050 mpu(Wire);

// ----- L298N Motor Pins -----
const int ENA = 25; const int IN1 = 26; const int IN2 = 27;
const int ENB = 14; const int IN3 = 12; const int IN4 = 13;

const int PWM_Frequency = 500;
const int PWM_Resolution = 12; // 0 - 4095

// ----- Global Variables -----
float kP = 250.0, kI = 10.0, kD = 3.0;
float setPoint = 2.0;
bool balanceMode = true;
float trim = 0; // Global variable


float kalmanPitch = 0;
float kalmanPitchUncertainty = 0;
float accelUncertainty = 3;
float gyroUncertainty = 4;

unsigned long timePrev = 0;
float timeDelta; 

// ----- PID Internal Variables -----
float integral = 0;
float previousAngle = 0;

// ----- Blynk Sliders -----
BLYNK_WRITE(V1) { kP = param.asFloat(); Serial.print("kP: "); Serial.println(kP); }
BLYNK_WRITE(V2) { kI = param.asFloat(); Serial.print("kI: "); Serial.println(kI); }
BLYNK_WRITE(V3) { kD = param.asFloat(); Serial.print("kD: "); Serial.println(kD); }
BLYNK_WRITE(V4) { balanceMode = param.asInt(); Serial.print("Mode: "); Serial.println(balanceMode); }

BLYNK_WRITE(V5) { 
  trim = param.asFloat(); // Set slider range -5.0 to 5.0 in Blynk app
}
// Motor functions with Deadband to help BO motors start turning
void Motor_1_Speed(int speed) {
  if (speed == 0) { ledcWrite(0, 0); return; }
  
  // Adding a deadband of 1100 (adjust if motors just hum but don't move)
  int min_pwm = 1100;
  int final_speed = (speed > 0) ? (speed + min_pwm) : (speed - min_pwm);
  final_speed = constrain(final_speed, -4095, 4095);

  digitalWrite(IN1, (final_speed > 0) ? HIGH : LOW);
  digitalWrite(IN2, (final_speed > 0) ? LOW : HIGH);
  ledcWrite(0, abs(final_speed));
}

void Motor_2_Speed(int speed) {
  if (speed == 0) { ledcWrite(1, 0); return; }
  
  int min_pwm = 1100;
  int final_speed = (speed > 0) ? (speed + min_pwm) : (speed - min_pwm);
  final_speed = constrain(final_speed, -4095, 4095);

  digitalWrite(IN3, (final_speed > 0) ? HIGH : LOW);
  digitalWrite(IN4, (final_speed > 0) ? LOW : HIGH);
  ledcWrite(1, abs(final_speed));
}

void kalmanUpdate(float gyroInput, float accelMeasurement, float dt) {
  kalmanPitch += dt * gyroInput;
  kalmanPitchUncertainty += pow(dt * gyroUncertainty, 2);
  float gain = kalmanPitchUncertainty / (kalmanPitchUncertainty + pow(accelUncertainty, 2));
  kalmanPitch += gain * (accelMeasurement - kalmanPitch);
  kalmanPitchUncertainty = (1 - gain) * kalmanPitchUncertainty;
}


void setup() {
  Serial.begin(115200);
  Wire.begin(); 
  
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  
  ledcSetup(0, PWM_Frequency, PWM_Resolution);
  ledcSetup(1, PWM_Frequency, PWM_Resolution);
  ledcAttachPin(ENA, 0);
  ledcAttachPin(ENB, 1);

  Serial.println("\n--- Initializing Hardware ---");
  
  byte status = mpu.begin();
  if (status != 0) { 
    Serial.println("MPU6050 FAILED");
    while (1); 
  }

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  
  Serial.println("Calibrating... DO NOT MOVE");
  // delay(2000);
  // mpu.calcOffsets(true, true);
  mpu.setAccOffsets(-0.92, -0.03, -1.44);  // Example: (X, Y, Z)
  mpu.setGyroOffsets(-2.19, -1.01, -0.76); // Example: (X, Y, Z)
  Serial.println("System Ready!");
  timePrev = millis();
}

void loop() {
  Blynk.run();
  
  unsigned long timeCurrent = millis();
  if (timeCurrent - timePrev >= 10) { 
    timeDelta = (timeCurrent - timePrev) / 1000.0;
    timePrev = timeCurrent;

    mpu.update();
    
    float gyroY = mpu.getGyroY(); 
    float accelPitch = mpu.getAngleY(); 

    kalmanUpdate(gyroY, accelPitch, timeDelta);
    
    if (balanceMode) {
      if (abs(kalmanPitch) > 45) {
        Motor_1_Speed(0); Motor_2_Speed(0);
        integral = 0;
      } else {
        float error = kalmanPitch - (setPoint + trim);
        
        // 1. RESET INTEGRAL ON ZERO CROSSING
        // This stops the robot from "lunging" past the center
        if ((error > 0 && (previousAngle - (setPoint + trim)) < 0) || 
            (error < 0 && (previousAngle - (setPoint + trim)) > 0)) {
            integral = 0; 
        }

        // 2. ONLY ACCUMULATE INTEGRAL NEAR CENTER
        // If error is > 15 degrees, just use P and D to catch the fall
        if (abs(error) < 15) {
            integral = constrain(integral + (kI * error * timeDelta), -1000, 1000);
        } else {
            integral = 0;
        }

        float p_term = kP * error;
        float d_term = kD * (kalmanPitch - previousAngle) / timeDelta;

        int output = (int)constrain(p_term + integral + d_term, -4095, 4095);
        
        Motor_1_Speed(-output); 
        Motor_2_Speed(-output);
        
        previousAngle = kalmanPitch;
      }
    } else {
      Motor_1_Speed(0); Motor_2_Speed(0);
      integral = 0;
    }

    static int count = 0;
    if (count++ > 10) {
      Serial.print("Angle: "); Serial.println(kalmanPitch);
      count = 0;
    }
  }
}
