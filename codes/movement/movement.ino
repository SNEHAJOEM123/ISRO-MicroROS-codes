#define DIR1 4
#define PWM1 5
#define DIR2 6
#define PWM2 7

int PWM_Value= 100;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(PWM1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(DIR2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Move forward");
  digitalWrite(DIR1,HIGH);
  digitalWrite(DIR2,HIGH);
  analogWrite(PWM1,PWM_Value);
  analogWrite(PWM2,PWM_Value);
  delay(100);

  Serial.println("Move backward");
  digitalWrite(DIR1,LOW);
  digitalWrite(DIR2,LOW);
  analogWrite(PWM1,PWM_Value);
  analogWrite(PWM2,PWM_Value);
  delay(100);

}