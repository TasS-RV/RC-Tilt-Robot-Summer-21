#define motorR1 9
#define motorR2 12
#define pwmL 5
#define pwmR 6
#define motorL1 8
#define motorL2 10


void setup() {
  pinMode(motorR1, OUTPUT);
  pinMode(motorR2, OUTPUT);
  pinMode(pwmR,OUTPUT);
  pinMode(motorL1, OUTPUT);
  pinMode(motorL2, OUTPUT);
  pinMode(pwmL, OUTPUT);
  
}

void loop() {
  analogWrite(pwmR,255);
  analogWrite(pwmL, 255);
  digitalWrite(motorR1, LOW);
  digitalWrite(motorR2, HIGH);
  digitalWrite(motorL1, LOW);
  digitalWrite(motorL2, HIGH);

}
