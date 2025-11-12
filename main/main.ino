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
float rightX = 0, leftY = 0;
float throttle = 0;
float brake = 0;
// Previous A button state for edge detection
bool prevAState = false;

// --- Constants ---
const float EPSILON = 0.1;  // Deadzone threshold
const int PWM_MAX = 255;
const float RAMP_FACTOR = 0.15;  // Smooth acceleration

// ESP32 + buzzer passivo: GǣbipsGǥ e chirps R2G��D2
const int BUZZER_PIN = 18; // chosen to avoid 32,33,25,13,15 and other used pins
const int BUZZER_CH = 4;   // use a dedicated LEDC channel for tones

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

// --- Buzzer functions ---
void chirp(int f0, int f1, int steps, int hold_ms) {
  int df = (f1 - f0) / steps;
  for (int i = 0; i <= steps; i++) {
    // ledcWriteTone takes the channel number and frequency
    ledcWriteTone(BUZZER_CH, f0 + i * df);
    delay(hold_ms);
  }
  ledcWriteTone(BUZZER_CH, 0); // silence
}

void beep(int f, int ms) {
  ledcWriteTone(BUZZER_CH, f);
  delay(ms);
  ledcWriteTone(BUZZER_CH, 0);
  delay(40);
}

void speak() {
  // Short R2-like sequence provided by the user
  beep(2200, 90); beep(3000, 70); chirp(1200, 3200, 12, 15);
  beep(2600, 80); chirp(3400, 1800, 10, 18); beep(2800, 60);
  delay(400);
}

void whistle(int f0, int f1, int tempo_ms) {
  int steps = 40;
  int hold = tempo_ms / steps;
  int df = (f1 - f0) / steps;
  for (int i = 0; i <= steps; i++) {
    ledcWriteTone(BUZZER_PIN, f0 + i*df);
    delay(hold);
  }
  ledcWriteTone(BUZZER_PIN, 0);
  delay(30);
}

// “scream” (agudo descendente, efeito alarme)
void scream(int f0, int f1, int tempo_ms) {
  int steps = 25;
  int hold = tempo_ms / steps;
  int df = (f1 - f0) / steps;
  for (int i = 0; i <= steps; i++) {
    ledcWriteTone(BUZZER_PIN, f0 - i*df);
    delay(hold);
  }
  ledcWriteTone(BUZZER_PIN, 0);
  delay(80);
}

// “trill” (efeito de alternância rápida entre dois tons)
void trill(int fA, int fB, int ciclos, int dt_ms) {
  for (int i = 0; i < ciclos; i++) {
    ledcWriteTone(BUZZER_PIN, fA);
    delay(dt_ms);
    ledcWriteTone(BUZZER_PIN, fB);
    delay(dt_ms);
  }
  ledcWriteTone(BUZZER_PIN, 0);
  delay(40);
}


void r2d2_gentle_chirp() {
  beep(2100, 60);
  whistle(2100, 3200, 140);
  trill(1700, 2200, 8, 13);
  beep(3200, 70);
}

void r2d2_excited() {
  chirp(1500, 3500, 16, 10);
  trill(3200, 2600, 8, 10);
  chirp(1800, 3500, 10, 15);
  trill(2700, 2400, 7, 9);
}

void r2d2_alarm() {
  scream(4100, 1500, 120);
  trill(1200, 670, 14, 13);
  beep(800, 60);
  beep(2000, 30);
  scream(4000, 1500, 110);
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

  // Setup a dedicated PWM channel for the buzzer (non-conflicting)
  ledcSetup(BUZZER_CH, 2000, 8);

  ledcAttachPin(ENA, 1);
  ledcAttachPin(ENB, 2);
  ledcAttachPin(RPWM, 0);
  ledcAttachPin(LPWM, 3);

  // Attach buzzer pin to its LEDC channel
  ledcAttachPin(BUZZER_PIN, BUZZER_CH);

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
    rightX = myController->axisRX() / 512.0;
    leftY = -myController->axisY() / 512.0;
    throttle = myController->throttle() / 512.0;
    brake = myController->brake() / 512.0;
    
    bool currA = myController->a();

    // Weapon target: throttle drives forward, brake drives reverse.
    // If both pressed they cancel: targetWeapon = throttle - brake
    float targetWeapon = constrain(throttle - brake, -1, 1);

    float targetLeft = constrain(leftY + rightX, -1, 1);
    float targetRight = constrain(leftY - rightX, -1, 1);

    leftMotor = smoothChange(leftMotor, targetLeft, RAMP_FACTOR);
    rightMotor = smoothChange(rightMotor, targetRight, RAMP_FACTOR);
    weaponSpeed = smoothChange(weaponSpeed, targetWeapon, RAMP_FACTOR);

    // Apply outputs
    // Note: ledcWrite expects a channel number (we attached ENA to channel 1 and ENB to channel 2)
    setMotor(IN2, IN1, ENB, leftMotor);
    setMotor(IN4, IN3, ENA, rightMotor);
    setWeapon(weaponSpeed);

    if (currA && !prevAState) {
      speak();
    }
    prevAState = currA;

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

