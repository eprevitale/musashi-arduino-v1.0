#include <Bluepad32.h>
#include <math.h>

// --- Pinout Config ---
#define ENA 25
#define IN1 26
#define IN2 27
#define IN3 14
#define IN4 12
#define ENB 15

#define RPWM 32
#define LPWM 33
#define R_EN 22
#define L_EN 23

// --- Control Variables ---
ControllerPtr myController;
float leftX = 0, leftY = 0;
float throttle = 0;
float brake = 0;

// --- Constants ---
const float EPSILON = 0.1;  // Deadzone threshold
const int PWM_MAX = 255;
const float RAMP_FACTOR = 0.15;  // Smooth acceleration

// --- Current output state ---
float leftMotor = 0;
float rightMotor = 0;
float weaponSpeed = 0;

// --- Utility Functions ---
float smoothChange(float current, float target, float factor) {
  return current + (target - current) * factor;
}

String movementDescriptor(float left, float right) {
  if (abs(left) < EPSILON && abs(right) < EPSILON) return "stopped";
  if (left > 0 && right > 0) return "moving forward";
  if (left < 0 && right < 0) return "moving backward";
  if (left > right) return "turning right";
  if (right > left) return "turning left";
  return "unknown";
}

String weaponDescriptor(float speed) {
  if (fabs(speed) < EPSILON) return "stopped";
  if (speed > 0) return "spinning clockwise";
  return "spinning counterclockwise";
}

// --- Motor Control ---
void setMotor(int in1, int in2, int channel, float speed) {
  int pwm = (int)(fabs(speed) * PWM_MAX);
  digitalWrite(in1, speed > 0);
  digitalWrite(in2, speed < 0);
  analogWrite(channel, pwm);
}

// --- Weapon Control ---
void setWeapon(float speed) {
  int pwm = (int)(fabs(speed) * PWM_MAX);
  // Enable outputs
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
  // Use channel 0 (RPWM) for forward, channel 3 (LPWM) for reverse.
  if (speed > 0) {
    ledcWrite(0, pwm);
    ledcWrite(3, 0);
  } else if (speed < 0) {
    ledcWrite(0, 0);
    ledcWrite(3, pwm);
  } else {
    ledcWrite(0, 0);
    ledcWrite(3, 0);
  }
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // Setup PWM channels for motors
  ledcSetup(0, 5000, 8);
  ledcSetup(1, 5000, 8);
  ledcSetup(2, 5000, 8);
  ledcSetup(3, 5000, 8);

  ledcAttachPin(ENA, 1);
  ledcAttachPin(ENB, 2);
  ledcAttachPin(RPWM, 0);
  ledcAttachPin(LPWM, 3);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);

  Serial.println("ESP32 Combat Robot Controller ready!");
}

void onConnectedController(ControllerPtr ctl) {
  myController = ctl;
  Serial.println("Controller connected!");
}

void onDisconnectedController(ControllerPtr ctl) {
  myController = nullptr;
  Serial.println("Controller disconnected!");
}

// --- Main Loop ---
void loop() {
  BP32.update();

  if (myController && myController->isConnected()) {
    leftX = myController->axisX() / 512.0;
    leftY = -myController->axisY() / 512.0;
    throttle = myController->throttle() / 512.0;
    brake = myController->brake() / 512.0;

    // Weapon target: throttle drives forward, brake drives reverse.
    // If both pressed they cancel: targetWeapon = throttle - brake
    float targetWeapon = constrain(throttle - brake, -1, 1);

    float targetLeft = constrain(leftY + leftX, -1, 1);
    float targetRight = constrain(leftY - leftX, -1, 1);

    leftMotor = smoothChange(leftMotor, targetLeft, RAMP_FACTOR);
    rightMotor = smoothChange(rightMotor, targetRight, RAMP_FACTOR);
    weaponSpeed = smoothChange(weaponSpeed, targetWeapon, RAMP_FACTOR);

    // Apply outputs
    // Note: ledcWrite expects a channel number (we attached ENA to channel 1 and ENB to channel 2)
    setMotor(IN1, IN2, ENA, leftMotor);
    setMotor(IN3, IN4, ENB, rightMotor);
    setWeapon(weaponSpeed);

    // Telemetry
    Serial.print("LX:"); Serial.print(leftX, 2);
    Serial.print("  LY:"); Serial.print(leftY, 2);
  Serial.print("  Throttle:"); Serial.print(throttle, 2);
  Serial.print("  Brake:"); Serial.print(brake, 2);
  Serial.print("  Movement: "); Serial.print(movementDescriptor(leftMotor, rightMotor));
  Serial.print("  Weapon: "); Serial.print(weaponDescriptor(weaponSpeed));
  Serial.print("  Wspd:"); Serial.println(weaponSpeed, 2);
  }
  delay(50);
}
