
#include <Stepper.h>
Stepper Armstepper(2048, A0, A1, A2, A3); 
Stepper Basestepper(2038,2,3, A4,A5);

#include <ServoTimer2.h>
#define Hpin 5; 
#define Npin 6;
#define Cpin 13;

int Hpos = 800; int Cpos = 1400; int Npos = 1000;
ServoTimer2 Hservo;
ServoTimer2 Nservo;
ServoTimer2 Cservo;


//Time keeping for stepper motors
long current_time = millis();
unsigned long previous_time;


//Time keeping for all Servo motors
long current_timeH = millis();
unsigned long previous_timeH; 


//Receiver connected to Pin 9
#include <avr/sleep.h>
#include <RH_ASK.h>
#include <SPI.h> // Not actualy used but needed to compile
const int rc_period = 50;

RH_ASK driver(2000,9,10,0);
String rec; unsigned long previousmillis = 0; 

//Interrupt pin definitions
int receiver_test_n = 0;
#define wake_pin 2

//Variables for decomposition of the R/C packet
String Pitch; String Roll; 

#define motorR1 4
#define motorR2 7
#define pwmL 3
#define pwmR 11
#define motorL1 12
#define motorL2 8

//Variables for Pitch and Roll
int P; int R; int pwmR_value; int pwmL_value; bool packetstate; 

//Variables for Claw and Arm control:
int T_; int H_; int B_; int N_; int claw; String Clawvalue;

////////////////////////////////////////////////////////
void setup()
{
    Serial.begin(9600);	// Debugging only
    if (!driver.init())
         Serial.println("init failed");


  pinMode(motorR1, OUTPUT);
  pinMode(motorR2, OUTPUT);
  pinMode(pwmR,OUTPUT);
  pinMode(motorL1, OUTPUT);
  pinMode(motorL2, OUTPUT);
  pinMode(pwmL, OUTPUT);

  pinMode(5,OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(13,OUTPUT);

//Servo and Stepper Motor pins setup:
  Nservo.attach(6);
 // Cservo.attach(13);
 Hservo.attach(5);
  pinMode(A0, OUTPUT);  
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);

  pinMode(2,OUTPUT);  
  pinMode(3,OUTPUT);  
  pinMode(A4,OUTPUT);  
  pinMode(A5,OUTPUT);
  
  
//Default Servo positions:
Hservo.write(Hpos); //This is the main arm servo
Nservo.write(Npos);
Cservo.write(Cpos);

Armstepper.setSpeed(9);
Basestepper.setSpeed(8);


}





////////////////////////////////////////
void reception()
{
  uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
    uint8_t buflen = sizeof(buf);

    if (driver.recv(buf, &buflen)) // Non-blocking
    {

	// Message with a good checksum received, dump it.
	/////driver.printBuffer("Got:", buf, buflen);

 for (int i = 0; i < buflen; i++) {
      rec += (char)buf[i];}
    }
    return rec;}

//Can be called anytime to check the R/C packet data

void custom_isr(){
  unsigned long currentmillis = millis();
  if (currentmillis - previousmillis >= rc_period) {
    
    //Calls the radio receiver
    reception();

    previousmillis = currentmillis;
    Serial.println(rec); //Serial.println(receiver_test_n);//When running the actual code, remove any serial prints as they are interrupts which will interfere with pwm signals
    receiver_test_n += 1;   
  }

if ((int)rec.length()>=20){
    //Data packet decomposition:
Pitch = rec.substring(rec.indexOf("V")+4, rec.indexOf("T"));
Roll =  rec.substring(rec.indexOf("V")+1, rec.indexOf("V")+4);

int P_print = Pitch.toInt(); int R_print = Roll.toInt();
P = P_print; R = R_print;
//Serial.print("Pitch:");Serial.println(P_print);
//Serial.print("Roll:");Serial.println(R_print);


if (abs(R)< 20){ //Roll code
  R = 1;}
else if (R > 70){
  R = 70;}
else if (R < -70){
  R = -70;}
else {R = R;} float R_ = R;

if (R != 1){pwmL_value = 100.0+abs((R_/70.0)*160.0);}
else if (R = 1){pwmL_value = 0;}
pwmL_value = round((int) pwmL_value);

if (abs(P)< 15){ //Pitch code
  P = 1;}
else if (P > 70){
  P = 70;}
else if (P < -70){
  P = -70;}
else {P = P;} float P_ = P;

if (P != 1){pwmR_value = 100.0+abs((P_/70.0)*160.0);}
else if (P = 1){
  pwmR_value = 0;}
pwmR_value = round((int) pwmR_value);

//Processing data from the Joysticks:
String T = rec.substring(rec.indexOf("T")+1, rec.indexOf("P"));
String H = rec.substring((rec).indexOf("P")+1, rec.indexOf("A"));
T_ =  T.toInt(); H_ = H.toInt();

String B = rec.substring(rec.indexOf("A")+1, rec.indexOf("H"));
String N = rec.substring(rec.indexOf("H")+1, rec.indexOf("C"));
N_ = N.toInt(); B_ = B.toInt();

Clawvalue = rec.substring(rec.indexOf("C")+1); claw = Clawvalue.toInt();
//Outputs and updating packet 
    rec = "";return receiver_test_n; return P; return R; return pwmR_value; return pwmL_value; return packetstate = true;
    return H_;return T_;  return N_; return B_; return claw; 
}
  
else{
  return packetstate = false;}
}
///////////////////////////////////////////

//Main Arm stepper motor:
void stepperA(String dirA) {
  
current_time = millis();
if (current_time - previous_time > 60 ){
   previous_time = current_time;

if (dirA == "R"){
Armstepper.step(30);
}

if (dirA == "L"){
Armstepper.step(-30);
}}
}

//Rotating Base stepper motor:
void stepperB(String dirB) {
  
current_time = millis();
if (current_time - previous_time > 60 ){
   previous_time = current_time;

if (dirB == "R"){
Basestepper.step(20);
}

if (dirB == "L"){
Basestepper.step(-20);
}}

}





//Main motor driver loop
void loop(){
custom_isr();
//Freezing PWM issue fixed: 1000uF cap. accross battery terminals: and arduino powered with separate power supply

if ((P>-10&&P<10) && (R>-10&&R<10)){
  digitalWrite(motorL1, LOW);
  digitalWrite(motorL2, LOW);
  digitalWrite(motorR1, LOW);
  digitalWrite(motorR2, LOW);} 

  
else if ((P>-10&&P<10) && abs(R)>10){
  //Steering motor control
if (R >=-10 && R <=10){
 // Serial.println("Neutral");

  analogWrite(pwmR, pwmL_value);
  analogWrite(pwmL, pwmL_value); 
  digitalWrite(motorL1, LOW);
  digitalWrite(motorL2, LOW);
  digitalWrite(motorR1, LOW);
  digitalWrite(motorR2, LOW); 
}


if (R>10){
 // Serial.println("Forward");
  analogWrite(pwmR, pwmL_value);
 analogWrite(pwmL, pwmL_value);
  digitalWrite(motorL1, LOW);
  digitalWrite(motorL2, HIGH);
  digitalWrite(motorR1, LOW);
  digitalWrite(motorR2, LOW);
}
if (R<-10){
 // Serial.println("Backwards");
  analogWrite(pwmR, pwmL_value);
  analogWrite(pwmL, pwmL_value);
  digitalWrite(motorL1, LOW);
  digitalWrite(motorL2, LOW);
  digitalWrite(motorR1, LOW);
  digitalWrite(motorR2, HIGH); 
}} 

  
else if (abs(P)>10 && (R>-10&&R<10)){
  if (P >=-10 && P <=10){
 // Serial.println("Neutral");

  analogWrite(pwmR, pwmR_value);
  analogWrite(pwmL, pwmR_value); 
  digitalWrite(motorL1, LOW);
  digitalWrite(motorL2, LOW);
  digitalWrite(motorR1, LOW);
  digitalWrite(motorR2, LOW); 
}


if (P>10){
 // Serial.println("Forward");
  analogWrite(pwmR, pwmR_value);
 analogWrite(pwmL, pwmR_value);
  digitalWrite(motorL1, LOW);
  digitalWrite(motorL2, HIGH);
  digitalWrite(motorR1, LOW);
  digitalWrite(motorR2, HIGH);
}

if (P<-10){
 // Serial.println("Backwards");
 analogWrite(pwmR, pwmR_value);
 analogWrite(pwmL, pwmR_value);
  digitalWrite(motorL1, HIGH);
  digitalWrite(motorL2, LOW);
  digitalWrite(motorR1, HIGH);
  digitalWrite(motorR2, LOW); 
} }

/*
/////////////////////////////////////
//Checker for RF receiver - if correctly obtaining values
if (packetstate = true){
Serial.println(claw);
}

//################  !!!!!!!!!!!!!! Delay here which could cause issues: currently mainly power and current draw problems
if (claw < 1){
  Cservo.attach(13); Cservo.write(1450);}
else{ Cservo.write(900); delay(30); Cservo.detach();}


//Arm base stepper motor:
if (T_ < 100){
  digitalWrite(10, HIGH);
    stepperA("L");}
else if (T_>700){
  digitalWrite(10, HIGH);
   stepperA("R");}

//Rotating Base stepper motor:
if (B_ < 100){
  digitalWrite(10, HIGH);
    stepperB("R");}
else if (B_>700){
  digitalWrite(10, HIGH);
   stepperB("L");}

//Stepper Motors use power to maintain position: we disconnect the power-line when not in use to save battery
if ((100 < B_ < 700) && (100 <T_<700)){
  digitalWrite(10,LOW);} //MOSFET signal


//Neck Servo control 
current_timeH = millis();
if (current_timeH - previous_timeH >30){  //Maybe use 35 
  previous_timeH = current_timeH;
   
   if (N_ < 100){
    Hservo.detach();Nservo.attach(6);
    Npos+=25;
   if (Npos > 2200){Npos = 2200;}
   Nservo.write(Npos);
}

else if (N_>700){
  Hservo.detach(); Nservo.attach(6);
   Npos-=25;
   if (Npos < 750){Npos = 750;}
   Nservo.write(Npos);

}

if (H_ < 100){
    Hservo.attach(5);
    Hpos+=35;
   if (Hpos > 1900){Hpos = 1900;}
   Hservo.write(Hpos);
}

else if (H_>700){
   Hservo.attach(5);
   Hpos-=35;
   if (Hpos < 750){Hpos = 750;}
   Hservo.write(Hpos);
}}

*/
}
