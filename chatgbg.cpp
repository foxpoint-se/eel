const int potPin = A6;
const int dirPin = 1;
const int pwmPin = 2;

const int potEmpty = 674;
const int potFull = 194;

const int filterSize = 10;
int potReadings[filterSize];
int potIndex = 0;
long potSum = 0;
int potAverage = 0;

String inputBuffer = "";
int setpoint = 674;
bool targetSet = false; // << NEW FLAG

// PID variables
float Kp = 0.2;
float Ki = 0.05;
float Kd = 0.1;
float previousError = 0;
float integral = 0;
unsigned long lastPIDTime = 0;

void setup() {
  pinMode(dirPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  analogWrite(pwmPin, 0);
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Enter percentage target (0–100):");

  for (int i = 0; i < filterSize; i++) {
    potReadings[i] = analogRead(potPin);
    potSum += potReadings[i];
  }
  potAverage = potSum / filterSize;

  lastPIDTime = millis();
}

void loop() {
  // Filtered potentiometer reading
  int newReading = analogRead(potPin);
  potSum = potSum - potReadings[potIndex] + newReading;
  potReadings[potIndex] = newReading;
  potIndex = (potIndex + 1) % filterSize;
  potAverage = potSum / filterSize;

  int tankPercent = map(potAverage, potEmpty, potFull, 0, 100);
  tankPercent = constrain(tankPercent, 0, 100);
  Serial.print("Pot: ");
  Serial.print(potAverage);
  Serial.print(" => ");
  Serial.print(tankPercent);
  Serial.println("%");

  // Handle input
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      processInput(inputBuffer);
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }

  // Only run PID if target has been set
  if (targetSet && millis() - lastPIDTime >= 20) {
    runPID();
    lastPIDTime = millis();
  } else if (!targetSet) {
    analogWrite(pwmPin, 0); // keep motor stopped
  }

  delay(50);
}

void processInput(String input) {
  input.trim();
  int pct = input.toInt();
  if (pct >= 0 && pct <= 100) {
    setpoint = map(pct, 0, 100, potEmpty, potFull);
    targetSet = true;
    Serial.print("New target: ");
    Serial.print(pct);
    Serial.print("% => Potentiometer target: ");
    Serial.println(setpoint);
  } else {
    Serial.println("Invalid input. Enter 0-100.");
  }
}

void runPID() {
  int error = potAverage - setpoint;

  // Deadband: stop motor if close enough
  if (abs(error) <= 3) {
    analogWrite(pwmPin, 0);
    integral = 0; // reset integral to prevent windup
    previousError = error;
    return;
  }

  // Integrate with clamping
  integral += error;
  const float integralMax = 3000;
  integral = constrain(integral, -integralMax, integralMax);

  float derivative = error - previousError;
  float output = Kp * error + Ki * integral + Kd * derivative;
  previousError = error;

  // Direction and PWM output
  if (output > 0) {
    digitalWrite(dirPin, HIGH); // Forward
  } else {
    digitalWrite(dirPin, LOW);  // Reverse
    output = -output;
  }

  const int maxPWM = 191;

int pwmOut = constrain(int(output), 0, maxPWM);
analogWrite(pwmPin, pwmOut);

}