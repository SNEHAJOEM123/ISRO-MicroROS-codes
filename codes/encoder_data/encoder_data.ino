#define ENCA D6
#define ENCB D5
#define ENCZ D7
#define LS1 D4

#define PWM D2
#define DIR D1


int PWM_Value= 100;
volatile long count = 0;
ICACHE_RAM_ATTR void EncoderDataA() 
{
  if(digitalRead(ENCB) == LOW) 
  count--;
  else
  count++; 
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(ENCA, INPUT_PULLUP); 
  pinMode(ENCB, INPUT_PULLUP);
  pinMode(PWM, OUTPUT);
  pinMode(DIR, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), EncoderDataA, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(count);
}
