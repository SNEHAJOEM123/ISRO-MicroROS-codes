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

void EncoderDataA1() 
{
  if(digitalRead(ENCB1) == LOW) 
  count1--;
  else
  count1++; 
}

void EncoderDataA2() 
{
  if(digitalRead(ENCB2) == LOW) 
  count2--;
  else
  count2++; 
}

void move_forward(){
    // Serial.println("Moving forward");
    digitalWrite(DIR1,LOW);
    analogWrite(PWM1,75);
    digitalWrite(DIR2,LOW);
    analogWrite(PWM2,80);
    delay(5000);
    
}

void move_backward(){
    digitalWrite(DIR1,LOW);
    analogWrite(PWM1,75);
    digitalWrite(DIR2,LOW);
    analogWrite(PWM2,80);
    delay(1000);
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
  attachInterrupt(digitalPinToInterrupt(ENCA1), EncoderDataA1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA2), EncoderDataA2, RISING);
  move_forward();
}

void loop() {
  // put your main code here, to run repeatedly:
  // move_forward();
  // move_backward();
  // Serial.println("Stop");
  digitalWrite(DIR1,HIGH);
  digitalWrite(DIR2,HIGH);
  analogWrite(PWM1,0);
  analogWrite(PWM2,0);

  Serial.println("Encoder data:");
  Serial.print("Count1:");
  Serial.println(count1);
  Serial.print("Count2:");
  Serial.println(count2);
}