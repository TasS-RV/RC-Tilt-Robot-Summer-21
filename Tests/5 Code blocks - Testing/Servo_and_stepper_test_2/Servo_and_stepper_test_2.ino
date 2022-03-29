#include <ServoTimer2.h>


#include <Stepper.h>
Stepper mystepper(2038, A2,A0,A1,A3);
//A0 A2 A1 A3 is the correct working order for the Stepper motor - make a note of this for the rotating base


#define Hpin 5; 
#define Npin 6;
#define Cpin 13;

int Hpos = 800; int Cpos = 1400; int Npos = 1000;
ServoTimer2 Hservo;
ServoTimer2 Nservo;
ServoTimer2 Cservo;

/*
//Roatating Base stepper motor
#define stepB1 2
#define stepB2 3
#define stepB3 A4
#define stepB4 A5
int stepnumB = 0;

//Arm-link rotating stepper
#define stepA A0
#define stepB A1
#define stepC A2
#define stepD A3
*/
int stepnumA = 0; //Time keeping for both Stepper motors
long current_time = millis();
unsigned long previous_time;
int step_; 

//Time keeping for all Servo motors
long current_timeH = millis();
unsigned long previous_timeH; 


int T_; int H_;


void setup() {
 mystepper.setSpeed(9);

  Serial.begin(9600);
  pinMode(5,OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(13,OUTPUT);
  
  Nservo.attach(6);
  Cservo.attach(13);
  Hservo.attach(5);
  pinMode(A0, OUTPUT);  
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);

  pinMode(2,OUTPUT);  
  pinMode(3,OUTPUT);  
 // pinMode(A4,OUTPUT);  
 // pinMode(A5,OUTPUT);
  
  
//Default Servo positions:
pinMode(A5, INPUT);
pinMode(A4, INPUT);

Hservo.write(Hpos); //This is the main arm servo
Nservo.write(Npos);
Cservo.write(Cpos);
  }


/*
// Arm-link stepper motor
void stepperB(String dirB) {
   
current_time = millis();
if (current_time - previous_time >7 ){
   previous_time = current_time;

   if (dirB == "R"){
    switch(stepnumB){
      case 0:
      digitalWrite(stepB1,HIGH);
       digitalWrite(stepB2,LOW);
       analogWrite(stepB3,0);
       analogWrite(stepB4,0);
      break;
      case 1:
      digitalWrite(stepB1, LOW);
       digitalWrite(stepB2,HIGH);
       analogWrite(stepB3,0);
       analogWrite(stepB4,0);
      break;
      case 2:
      digitalWrite(stepB1,LOW);
       digitalWrite(stepB2,LOW);
       analogWrite(stepB3,1023);
       analogWrite(stepB4,0);
      break;
      case 3:
       digitalWrite(stepB1,LOW);
       digitalWrite(stepB2,LOW);
       analogWrite(stepB3,0);
       analogWrite(stepB4,1023);
      break;
      }}
   if (dirB == "L"){
    switch(stepnumB){
      case 0:
      digitalWrite(stepB1,LOW);
       digitalWrite(stepB2,LOW);
       analogWrite(stepB3,0);
       analogWrite(stepB4,1023);
      break;
      case 1:
      digitalWrite(stepB1,LOW);
       digitalWrite(stepB2,LOW);
       analogWrite(stepB3,1023);
       analogWrite(stepB4,0);
      break;
      case 2:
      digitalWrite(stepB1,LOW);
       digitalWrite(stepB2,HIGH);
       analogWrite(stepB3,0);
       analogWrite(stepB4,0);
      break;
      case 3:
      digitalWrite(stepB1,HIGH);
       digitalWrite(stepB2,LOW);
       analogWrite(stepB3,0);
       analogWrite(stepB4,0);
      break;
      }}
    else if (dirB == "N"){
      digitalWrite(stepB1,LOW);
       digitalWrite(stepB2,LOW);
       analogWrite(stepB3,0);
       analogWrite(stepB4,0);}

        stepnumB ++;
      if (stepnumB >3){
        stepnumB = 0;}
  }
}
*/




void stepperA(String dirA) {
  
current_time = millis();
if (current_time - previous_time > 70 ){
   previous_time = current_time;

if (dirA == "R"){
mystepper.step(30);
}

if (dirA == "L"){
mystepper.step(-30);
}

}}





   

/*
//  Rotating Base Stepper Motor
void stepperA(String dirA) {
   
current_time = millis();
if (current_time - previous_time >6 ){
   previous_time = current_time;

   if (dirA == "R"){
    switch(stepnumA){
      case 0:
      analogWrite(stepA,1023);
       analogWrite(stepB,0);
       analogWrite(stepC,0);
       analogWrite(stepD,0);
      break;
      case 1:
      analogWrite(stepA,0);
       analogWrite(stepB,1023);
       analogWrite(stepC,0);
       analogWrite(stepD,0);
      break;
      case 2:
      analogWrite(stepA,0);
       analogWrite(stepB,0);
       analogWrite(stepC,1023);
       analogWrite(stepD,0);
      break;
      case 3:
       analogWrite(stepA,0);
       analogWrite(stepB,0);
       analogWrite(stepC,0);
       analogWrite(stepD,1023);
      break;
      }}
   if (dirA == "L"){
    switch(stepnumA){
      case 0:
      analogWrite(stepA,0);
       analogWrite(stepB,0);
       analogWrite(stepC,0);
       analogWrite(stepD,1023);
      break;
      case 1:
      analogWrite(stepA,0);
       analogWrite(stepB,0);
       analogWrite(stepC,1023);
       analogWrite(stepD,0);
      break;
      case 2:
      analogWrite(stepA,0);
       analogWrite(stepB,1023);
       analogWrite(stepC,0);
       analogWrite(stepD,0);
      break;
      case 3:
      analogWrite(stepA,1023);
       analogWrite(stepB,0);
       analogWrite(stepC,0);
       analogWrite(stepD,0);
      break;
      }}
    else if (dirA == "N"){
      analogWrite(stepA,0);
       analogWrite(stepB,0);
       analogWrite(stepC,0);
       analogWrite(stepD,0);}

        stepnumA ++;
      if (stepnumA >3){
        stepnumA = 0;}
  }
}
*/

void loop() {

int val = analogRead(A5);
int val0 = analogRead(A4);
Serial.println(val);

//Arm base stepper motor
if (val0 < 100){
    stepperA("R");}
else if (val0>700){
   stepperA("L");}

/*
// Tail-end Servo attach to arm

current_timeH = millis();
if (current_timeH - previous_timeH >35){  //Maybe use 35 
  previous_timeH = current_timeH;
   
   if (val < 100){
    Hpos+=50;
   if (Hpos > 1900){Hpos = 1900;}
   Hservo.write(Hpos);
}

else if (val>700){
   Hpos-=50;
   if (Hpos < 750){Hpos = 760;}
   Hservo.write(Hpos);
}}

String T_ = rec.substring(rec.indexOf("T"), rec.indexOf("P"));
String H = rec.substring((rec).indexOf("P"), rec.indexOf("A"));
T_ =  T.toInt(); H_ = H.toInt();
return H_, return H_;
*/




//Neck Servo control 
current_timeH = millis();
if (current_timeH - previous_timeH >30){  //Maybe use 35 
  previous_timeH = current_timeH;
   
   if (val < 100){
    Npos+=25;
   if (Npos > 2200){Npos = 2200;}
   Nservo.write(Npos);
}

else if (val>700){
   Npos-=25;
   if (Npos < 750){Npos = 750;}
   Nservo.write(Npos);
}}


//Cservo.write(800); //FULLY OPEN
//Cservo.write(1500); //FU*LLY CLOSED
}

//Connecting the Servos: Claw Servo to separate 5V power rail on other side
//Arm Servo connected to Arduino Power, and Neck Servo connected to same 5V power rail as Stepper motors
//This minimises interference of the pwm signals











/*
int val = analogRead(A5);
if (val < 100){
  Hservo_move(1);}
else if (val > 750){
  Hservo_move(2);}



 
Serial.println(analogRead(A5));  
//Hservo_move(1);

    
//stepperA("R");
*/

 




/*
#include <avr/sleep.h>
#include <RH_ASK.h>
#include <SPI.h> // Not actualy used but needed to compile
const int rc_period = 50;

RH_ASK driver(2000,9,10,0);
String rec; unsigned long previousmillis = 0; 

//Interrupt pin definitions
#include <TimerOne.h>
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


////////////////////////////////////////////////////////
void setup()
{
    Serial.begin(9600); // Debugging only
    if (!driver.init())
         Serial.println("init failed");

   // attachInterrupt(wake_pin, wake_, RISING);

  pinMode(motorR1, OUTPUT);
  pinMode(motorR2, OUTPUT);
  pinMode(pwmR,OUTPUT);
  pinMode(motorL1, OUTPUT);
  pinMode(motorL2, OUTPUT);
  pinMode(pwmL, OUTPUT);
//Timer1.initialize(71341);
//Timer1.attachInterrupt(wake_);         
}

void wake_(){
//Will not work, as the Radio signal triggers the arduino to wake up 
  sleep_enable();
  receiver_test_n = 0;
/*  if (S_state = "Off"){
    sleep_enable();
    S_state = "On";digitalWrite(13,HIGH);}
  else if (S_state = "On"){
    sleep_disable();
    S_state = "Off";digitalWrite(13,LOW);}

  sleep_disable(); //digitalWrite(13,LOW);
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


//Outputs and updating packet 
    rec = "";return receiver_test_n; return P; return R; return pwmR_value; return pwmL_value; return packetstate = true; 

    
  }
  
else{
  return packetstate = false;}
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
 // exit(1);
}


if (R>10){
 // Serial.println("Forward");
  analogWrite(pwmR, pwmL_value);
 analogWrite(pwmL, pwmL_value);
  digitalWrite(motorL1, LOW);
  digitalWrite(motorL2, HIGH);
  digitalWrite(motorR1, LOW);
  digitalWrite(motorR2, LOW);

 // exit(1);
}
if (R<-10){
 // Serial.println("Backwards");
  analogWrite(pwmR, pwmL_value);
  analogWrite(pwmL, pwmL_value);
  digitalWrite(motorL1, LOW);
  digitalWrite(motorL2, LOW);
  digitalWrite(motorR1, LOW);
  digitalWrite(motorR2, HIGH); 
 // exit(1);
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
 // exit(1);
}


if (P>10){
 // Serial.println("Forward");
  analogWrite(pwmR, pwmR_value);
 analogWrite(pwmL, pwmR_value);
  digitalWrite(motorL1, LOW);
  digitalWrite(motorL2, HIGH);
  digitalWrite(motorR1, LOW);
  digitalWrite(motorR2, HIGH);

 // exit(1);
}
if (P<-10){
 // Serial.println("Backwards");
 analogWrite(pwmR, pwmR_value);
 analogWrite(pwmL, pwmR_value);
  digitalWrite(motorL1, HIGH);
  digitalWrite(motorL2, LOW);
  digitalWrite(motorR1, HIGH);
  digitalWrite(motorR2, LOW); 
 // exit(1);
} }

//Checker for RF receiver - if correctly obtaining values
if (packetstate = true){
Serial.println(R);
Serial.println(pwmL_value);
}}    
 */
