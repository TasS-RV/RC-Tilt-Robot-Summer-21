//Note: on our model of the L293D, the pwm pin in pin 1 (top-left, when notch is at the top)

int motorA1 = 8;
int motorA2 = 9;
int Apwm = 5;
float sig; float intensity;

void setup() {
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(Apwm, OUTPUT);
  pinMode(A0, INPUT);
  Serial.begin(9600);
}

void loop() {
  intensity = analogRead(A0); 
 // Serial.println("Analog signal:");
  
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2,HIGH);
  sig = (int)((intensity/1023.0)*255.0);
  //Serial.print(round(sig));
  analogWrite(Apwm, sig);
  
}
