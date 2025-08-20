// Pin Definitions
const int PWM_PIN_3 = 3;   // Motor speed
const int PWM_PIN_5 = 5;   // Motor speed
const int PIN_7 = 7;       // Turn
const int PIN_6 = 6;       // Turn
const int PIN_8 = 8;       // Direction
const int PIN_9 = 9;       // Direction
const int PIN_10 = 10;     // Direction
const int PIN_11 = 11;     // Direction
const int BRAKE_DIR = 2;   // Brake direction
const int BRAKE_STEP = 5;  // Brake step
const int BRAKE_EN = 8;    // Brake enable
const int SENSOR = 7;      // Brake sensor

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Set pin modes
  pinMode(PWM_PIN_3, OUTPUT);
  pinMode(PWM_PIN_5, OUTPUT);
  pinMode(PIN_7, OUTPUT);
  pinMode(PIN_6, OUTPUT);
  pinMode(PIN_8, OUTPUT);
  pinMode(PIN_9, OUTPUT);
  pinMode(PIN_10, OUTPUT);
  pinMode(PIN_11, OUTPUT);
  pinMode(BRAKE_DIR, OUTPUT);
  pinMode(BRAKE_STEP, OUTPUT);
  pinMode(BRAKE_EN, OUTPUT);
  pinMode(SENSOR, INPUT);
}

void loop() {
  // Example usage
  rotateForward();
  delay(2000);
  rotateReverse();
  delay(2000);
  brakeHome();
  delay(1000);
  brakeMove(1000, HIGH);
  delay(2000);
}

// Set PWM duty cycle (0–255)
void setPWM(int pin, int dutyCycle) {
  analogWrite(pin, dutyCycle);
}

// Motor control
void updateMotor3(int value) {
  setPWM(PWM_PIN_3, value);
}

void updateMotor5(int value) {
  setPWM(PWM_PIN_5, value);
}

void controlPin7(bool state) {
  digitalWrite(PIN_7, state);
}

void controlPin6(bool state) {
  digitalWrite(PIN_6, state);
}

// Direction control
void rotateForward() {
  digitalWrite(PIN_10, HIGH);
  digitalWrite(PIN_11, LOW);
  digitalWrite(PIN_8, HIGH);
  digitalWrite(PIN_9, LOW);
  Serial.println("Rotating forward...");
}

void rotateReverse() {
  digitalWrite(PIN_10, LOW);
  digitalWrite(PIN_11, HIGH);
  digitalWrite(PIN_8, LOW);
  digitalWrite(PIN_9, HIGH);
  Serial.println("Rotating reverse...");
}

// Brake control
void brakeHome() {
  Serial.println("Homing...");
  digitalWrite(BRAKE_EN, LOW);
  digitalWrite(BRAKE_DIR, LOW);
  while (digitalRead(SENSOR) != LOW) {
    digitalWrite(BRAKE_STEP, HIGH);
    delayMicroseconds(50);
    digitalWrite(BRAKE_STEP, LOW);
    delayMicroseconds(50);
  }
  Serial.println("Home position reached.");
}

void brakeMove(long steps, bool direction) {
  Serial.print("Moving ");
  Serial.print(steps);
  Serial.print(" steps in direction ");
  Serial.println(direction == HIGH ? "HIGH" : "LOW");

  digitalWrite(BRAKE_EN, LOW);
  digitalWrite(BRAKE_DIR, direction);
  for (long i = 0; i < steps; i++) {
    digitalWrite(BRAKE_STEP, HIGH);
    delayMicroseconds(50);
    digitalWrite(BRAKE_STEP, LOW);
    delayMicroseconds(50);
  }
}
