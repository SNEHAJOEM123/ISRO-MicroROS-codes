#define ENCA1 19
#define ENCB1 21
#define ENCA2 22
#define ENCB2 23
#define DIR1 25
#define PWM1 26
#define DIR2 27
#define PWM2 32

const int mChannel1 = 0;
const int mChannel2=1;
const int frequency = 5000;
const int resolution = 8;

volatile long count1 = 0;
volatile long count2 = 0;
unsigned long previousMillis = 0;
unsigned long startmillis=0;
const long interval = 100; // Interval to print encoder data (in milliseconds)

void IRAM_ATTR EncoderDataA1() {
  if (digitalRead(ENCB1) == LOW)
    count1--;
  else
    count1++;
}

void IRAM_ATTR EncoderDataA2() {
  if (digitalRead(ENCB2) == LOW)
    count2--;
  else
    count2++;
}

void move_forward() {
  Serial.println("Moving forward");
  digitalWrite(DIR1, LOW);
  ledcWrite(mChannel1,80);
  digitalWrite(DIR2, LOW);
  ledcWrite(mChannel2,80);
}

void move_backward() {
  digitalWrite(DIR1, HIGH);
  ledcWrite(mChannel1,80);
  digitalWrite(DIR2, HIGH);
  ledcWrite(mChannel2,80);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(ENCA1, INPUT_PULLUP);
  pinMode(ENCB1, INPUT_PULLUP);
  pinMode(ENCA2, INPUT_PULLUP);
  pinMode(ENCB2, INPUT_PULLUP);
  pinMode(PWM1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(DIR2, OUTPUT);
  ledcSetup(mChannel1, frequency, resolution);
  ledcAttachPin(PWM1, mChannel1);
  ledcSetup(mChannel2, frequency, resolution);
  ledcAttachPin(PWM2, mChannel2);
  attachInterrupt(digitalPinToInterrupt(ENCA1), EncoderDataA1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA2), EncoderDataA2, RISING);
  move_forward();
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // Save the last time the encoder data was printed
    previousMillis = currentMillis;
    Serial.println(currentMillis);
    Serial.println("Encoder data:");
    Serial.print("Count1:");
    Serial.println(count1);
    Serial.print("Count2:");
    Serial.println(count2);
  }

  // Check if you need to stop moving forward (e.g., after 5 seconds)
  // if(currentMillis<10000){
  //   move_forward();
  // }
  if(currentMillis>25000){
    // Stop moving forward
    digitalWrite(DIR1, LOW);
    digitalWrite(DIR2, LOW);
    ledcWrite(mChannel1,0);
    ledcWrite(mChannel2,0);
    // Perform other actions as needed (e.g., move backward)
    // move_backward();
  }
}
