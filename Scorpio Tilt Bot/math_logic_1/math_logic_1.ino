int pwmR_value;
int P;

void setup() {
Serial.begin(9600);
}

void loop() {
P = -56;
Serial.println(P);
////////////////////////////////////////  
if (abs(P)< 15){
  P = 0;}
else if (abs(P)>70){
  P = 70;}
else {P = P;} float P_ = P;

if (P != 0){pwmR_value = 120+((abs(P_)/70)*140);}
else if (P = 0){pwmR_value = 0;}


pwmR_value = round((int) pwmR_value);
Serial.println(pwmR_value);

///////////////////////

delay(300);



}
