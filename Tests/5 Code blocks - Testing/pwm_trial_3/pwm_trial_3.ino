#define motorR1 9
#define motorR2 12
#define pwmL 5
#define pwmR 6
#define motorL1 8
#define motorL2 10

int P; int pwmR_value; int pwmL_value;

void setup() {
  pinMode(motorR1, OUTPUT);
  pinMode(motorR2, OUTPUT);
  pinMode(pwmR,OUTPUT);
  pinMode(motorL1, OUTPUT);
  pinMode(motorL2, OUTPUT);
  pinMode(pwmL, OUTPUT);
  
}

void loop() {


if (abs(P)< 15){
  P = 0;}
else if (abs(P)>70){
  P = 70;}
else {P = P;} float P_ = P;

if (P != 0){pwmR_value = 120+((abs(P_)/70)*140);}
else if (P = 0){pwmR_value = 0;}
pwmR_value = round((int) pwmR_value);

//Driving motor control
analogWrite(pwmR, pwmR_value);
analogWrite(pwmL, pwmR_value);
if (P>0){
  digitalWrite(motorL1, LOW);
  digitalWrite(motorL2, HIGH);
  digitalWrite(motorR1, LOW);
  digitalWrite(motorR2, HIGH); 
}
else if (P<0){
  digitalWrite(motorL1, HIGH);
  digitalWrite(motorL2, LOW);
  digitalWrite(motorR1, HIGH);
  digitalWrite(motorR2, LOW); 
}
else if (P = 0){
  digitalWrite(motorL1, LOW);
  digitalWrite(motorL2, LOW);
  digitalWrite(motorR1, LOW);
  digitalWrite(motorR2, LOW); 
}

/* 
 if (receiver_test_n <= 100){
  custom_isr();
  }

//After a sequence of Radio messages, arduino will sleep (to avoid overflow of Serial port and power use)

 else {
  digitalWrite(13,HIGH);
  sleep_enable();
 }
*/
}
