#define ENCA1 2
#define ENCB1 3
#define ENCA2 20
#define ENCB2 21

int PWM_Value= 100;
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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(ENCA1, INPUT_PULLUP); 
  pinMode(ENCB1, INPUT_PULLUP);
  pinMode(ENCA2, INPUT_PULLUP); 
  pinMode(ENCB2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA1), EncoderDataA1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA2), EncoderDataA2, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Encoder data:");
  Serial.print("Count1:");
  Serial.println(count1);
  Serial.print("Count2:");
  Serial.println(count2);
}