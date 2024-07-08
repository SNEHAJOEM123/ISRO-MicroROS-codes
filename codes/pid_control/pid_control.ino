#define ENCA1 2
#define ENCB1 3
#define ENCA2 20
#define ENCB2 21
#define DIR1 4
#define PWM1 5
#define DIR2 6
#define PWM2 7

volatile long count1 = 0;
volatile long count2 = 0;
int targetSpeed = -3000; // Target speed in counts per second
int currentSpeed1 = 0; // Current speed of motor 1
int currentSpeed2 = 0; // Current speed of motor 2
unsigned long previousMillis = 0;
const long interval = 100; // Interval for PID control loop (in milliseconds)
double Kp = 0.1; // Proportional gain
double Ki = 0.01; // Integral gain
double Kd = 0.001; // Derivative gain
double errorSum1 = 0; // Error sum for motor 1
double errorSum2 = 0; // Error sum for motor 2
double lastError1 = 0; // Last error for motor 1
double lastError2 = 0; // Last error for motor 2

void EncoderDataA1() {
  if (digitalRead(ENCB1) == LOW)
    count1--;
  else
    count1++;
}

void EncoderDataA2() {
  if (digitalRead(ENCB2) == LOW)
    count2--;
  else
    count2++;
}

void move_forward(int speed1, int speed2) {
  digitalWrite(DIR1, LOW);
  analogWrite(PWM1, speed1);
  digitalWrite(DIR2, LOW);
  analogWrite(PWM2, speed2);
}

void setup() {
  Serial.begin(9600);
  pinMode(ENCA1, INPUT_PULLUP);
  pinMode(ENCB1, INPUT_PULLUP);
  pinMode(ENCA2, INPUT_PULLUP);
  pinMode(ENCB2, INPUT_PULLUP);
  pinMode(PWM1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(DIR2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA1), EncoderDataA1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA2), EncoderDataA2, RISING);
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // Update the previousMillis for the next loop iteration
    previousMillis = currentMillis;
    
    // Calculate the speeds based on the encoder counts
    currentSpeed1 = count1 / (interval / 1000.0); // Counts per second for motor 1
    currentSpeed2 = count2 / (interval / 1000.0); // Counts per second for motor 2
    
    // Compute PID control output for motor 1
    int error1 = targetSpeed - currentSpeed1;
    errorSum1 += error1;
    double dError1 = error1 - lastError1;
    lastError1 = error1;
    double output1 = Kp * error1 + Ki * errorSum1 + Kd * dError1;
    
    // Compute PID control output for motor 2
    int error2 = targetSpeed - currentSpeed2;
    errorSum2 += error2;
    double dError2 = error2 - lastError2;
    lastError2 = error2;
    double output2 = Kp * error2 + Ki * errorSum2 + Kd * dError2;
    
    // Apply the control outputs to adjust motor speeds
    move_forward(output1, output2);
    
    // Reset encoder counts for the next interval
    count1 = 0;
    count2 = 0;
    
    // Print debugging information
    Serial.print("Count1:");
    Serial.print(count1);
    Serial.print("Target Speed: ");
    Serial.print(targetSpeed);
    Serial.print(" Current Speed1: ");
    Serial.print(currentSpeed1);
    Serial.print(" Output1: ");
    Serial.print(output1);
    Serial.print(" Current Speed2: ");
    Serial.print(currentSpeed2);
    Serial.print(" Output2: ");
    Serial.println(output2);
  }
}
