/*Program written by Tasin Sayed. Date: 05 August 2021 for final draft.
 * Any Serial prints are for diagnostic purposes: to ensure correct values unpacked from 433MHz RF module.
 * Servo pins must be detached when in drive mode: this intefered with the PWM pins for L293D motor driver.
 * Analog pins of stepper motors used as digital pin outputs - they are capable of digitalRead/Write functions.
 */

#include <avr/wdt.h>
//Watch-dog timer: resets arduino when switching back to 'D' mode. Servo pins 5, 6, and 13 

#include <Stepper.h>
Stepper Armstepper(2048, A0, A1, A2, A3); 
Stepper Basestepper(2038,2,3, A4,A5);

#include <ServoTimer2.h> //Timer 0 used for PWM on L293D - timer1 (pin 9) used by 433 MHz recceiver, timer 2 for Servo control via PWm
#define Hpin 5;  //Rear-hinge servo
#define Npin 6;  //Neck servo
#define Cpin 13; //Claw servo

int Hpos = 800; int Cpos = 1400; int Npos = 1000;//Initialising Servo positions - REDUNDANT VARIABLES (kept for safe default positions)

ServoTimer2 Hservo;
ServoTimer2 Nservo;
ServoTimer2 Cservo;

//Time keeping for Stepper motors
long current_time = millis();
unsigned long previous_time;

//Time keeping for all Servo motors
long current_timeH = millis();
unsigned long previous_timeH; 

//Decomposing RC packet - claw and arm control data
int T_; int H_; int B_; int N_; int claw; String Clawvalue; 
String mode;

//Receiver connected to Pin 10 ################
#include <RH_ASK.h>
#include <SPI.h> // Not actualy used but needed to compile
const int rc_period = 7;

RH_ASK driver(2000,9,10,0);
String rec; unsigned long previousmillis = 0; 


//Decomposing RC packet - motion control data 
String Pitch; String Roll; 

#define motorR1 4
#define motorR2 7
#define pwmL 3
#define pwmR 11
#define motorL1 12
#define motorL2 8

//Variables for Pitch and Roll
int P; int R; int pwmR_value; int pwmL_value; bool packetstate; 


//Watch-dog timer: 0b00001000 setup for board reset
void WDTsetup(byte WDTmode){
  WDTCSR |= 0b00011000;
  WDTCSR = WDTmode | WDTO_250MS; //Setting up the timer for 250ms refresh period
  wdt_reset();
  }

////////////////////////////////////////////////////////
void setup()
{ //RF module crash test:
    Serial.begin(9600);	// Debugging only
    if (!driver.init())
         Serial.println("init failed");

  pinMode(motorR1, OUTPUT);
  pinMode(motorR2, OUTPUT);
  pinMode(pwmR,OUTPUT);
  pinMode(motorL1, OUTPUT);
  pinMode(motorL2, OUTPUT);
  pinMode(pwmL, OUTPUT);

//Sets up stepper motor speeds:
Armstepper.setSpeed(9);
Basestepper.setSpeed(8);

WDTsetup(0b00001000); wdt_disable(); //MUST disable: or program gets stuck in infinite reset loop (WDT is tripped before reset)
}


bool state; //Variable helps with loop errors/ board being frozen

//Program fetches packet received by RF module
void reception()
{
  uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
    uint8_t buflen = sizeof(buf);

//Packet passthrough loop: !very important, as this loop is NOT always satisfied when the reception() is called-
//This can prevent the 'rec' packet data from being updated, if the  receiver freezes
//Arduino calls Receiver to fetch data, but it may be frozen so packet not updated ('blocked')
    
state = false; 

if (driver.recv(buf, &buflen)) // State = false indicates inadequate packet received: prevents processes that could freeze the receiver from 
//being stuck in an inifinite loop
    {
	// Message with a good checksum received, dump it.
	//driver.printBuffer("Got:", buf, buflen);

 state = true; //If not satisfied, the packet state will not be true, so will terminate any ongoing loops
   
 for (int i = 0; i < buflen; i++) {
      rec += (char)buf[i];}
    }    
    return rec;}

//Fetches the data packet each program cycle: 
void custom_isr(){
  unsigned long currentmillis = millis();
  if (currentmillis - previousmillis >= rc_period) { //Refreshed every 7ms - period setup at start
    
    //Calls the radio receiver to fetch packet
    reception();

    previousmillis = currentmillis;
   // Serial.println(rec);  Serial.println(state);
  }            

//  !!  Serial prints are used for debugging and to check state of RF readings - if corresponding to motion from motors and servos. 
//  !! These should be commented out when running the actual code - to prevent Interrupts intefering  with the runtime. 



if ((int)rec.length()>=20){  //Receiver does not pcik up every packet sent: so, all States are retained until correct data packet is received

// Pitch - forwards and backwards motion, Roll - left and right rotation
//Angle of pitch and roll from 15-70 affects the % of PWM: +-15 deg. deadzone for the Sensor.

Pitch = rec.substring(rec.indexOf("V")+4, rec.indexOf("T"));
Roll =  rec.substring(rec.indexOf("V")+1, rec.indexOf("V")+4);

int P_print = Pitch.toInt(); int R_print = Roll.toInt();
P = P_print; R = R_print;

//Serial.print("Pitch:");Serial.println(P_print);
//Serial.print("Roll:");Serial.println(R_print);


if (abs(R)< 15){ //Roll code
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
//T value controls main arm stepper. H value controls angle of rear Hinge servo. 

String B = rec.substring(rec.indexOf("A")+1, rec.indexOf("H"));
String N = rec.substring(rec.indexOf("H")+1, rec.indexOf("C"));
N_ = N.toInt(); B_ = B.toInt();
//N value controls angle of Neck servo. B value controls Rotation of the Base stepper.

Clawvalue = rec.substring(rec.indexOf("C")+1, rec.indexOf("C")+2); claw = Clawvalue.toInt();

//Mode: deactivates motor pins if 'Z' - arm and claw control mode. 'D' is Drive mode, deactivates all servo and stepper pins.
mode = rec.substring(rec.indexOf("C")+2);


//Outputs from decomposition of data packet 'rec': also clears the variable onto which packet is unloaded
    rec = ""; return P; return R; return pwmR_value; return pwmL_value; return packetstate = true; 
return H_;return T_;  return N_; return B_; return claw; return mode;
}
  
else{
  return packetstate = false;} //Does not update values if correct packet size not received - retaines previous states
}


//Main Arm stepper motor:

void stepperA(String dirA) {
Hservo.detach();
  
current_time = millis();
if (current_time - previous_time > 23 ){
   previous_time = current_time;

if (dirA == "R"){
Armstepper.step(36);
}

if (dirA == "L"){
Armstepper.step(-36);
}}
}

//Rotating Base stepper motor:

void stepperB(String dirB) {
Hservo.detach();
  
current_time = millis();
if (current_time - previous_time > 19 ){
   previous_time = current_time;

if (dirB == "R"){
Basestepper.step(27);
}

if (dirB == "L"){
Basestepper.step(-27);
}}
}

//When mode 'D' is initiated: drive mode (L293D)
void drive_mode(){
  //Freezing PWM issue fixed: 1000uF cap. accross battery terminals: and arduino powered with separate power supply
Hservo.detach(); Nservo.detach(); Cservo.detach();

pinMode(A0, INPUT);  
  pinMode(A1,INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);

  pinMode(2,INPUT);  
 // pinMode(3,INPUT); Coincides with pwmL pin  
  pinMode(A4,INPUT);  
  pinMode(A5,INPUT);

  pinMode(5, INPUT); pinMode(13, INPUT); pinMode(6, INPUT);
  

//L293 logic based on receiver input:

if ((P>-10&&P<10) && (R>-10&&R<10)){ //If values 0: does not drive motors
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


if (R<-10){
 // Serial.println("Forward");
  analogWrite(pwmR, pwmL_value);
 analogWrite(pwmL, pwmL_value);
  digitalWrite(motorL1, LOW);
  digitalWrite(motorL2, HIGH);
  digitalWrite(motorR1, LOW);
  digitalWrite(motorR2, LOW);
}

if (R>10){
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


if (P<-10){
 // Serial.println("Forward");
  analogWrite(pwmR, pwmR_value);
int adjust_pwm; 
if (pwmR_value >= 150){ adjust_pwm = (pwmR_value - 35); 
analogWrite(pwmL, adjust_pwm);}// Due to partial curve when driving forward or back - imbalance of R and L torques
 else{analogWrite(pwmL, pwmR_value);}
  digitalWrite(motorL1, LOW);
  digitalWrite(motorL2, HIGH);
  digitalWrite(motorR1, LOW);
  digitalWrite(motorR2, HIGH);
}

if (P>10){
 // Serial.println("Backwards");
 analogWrite(pwmR, pwmR_value);
 int adjust_pwm;
 if (pwmR_value >= 150){ adjust_pwm = (pwmR_value - 35);
 analogWrite(pwmL, adjust_pwm);}// Due to partial curve when driving forward or back - imbalance of R and L torques
 
 else{analogWrite(pwmL, pwmR_value);}
 digitalWrite(motorL1, HIGH);
  digitalWrite(motorL2, LOW);
  digitalWrite(motorR1, HIGH);
  digitalWrite(motorR2, LOW); 
} }
}


//When mode 'Z' is activated - claw and arm control:
void control_mode(){
  
pinMode(motorR1,INPUT);
  pinMode(motorR2, INPUT);
  pinMode(pwmR, INPUT);
  pinMode(motorL1, INPUT);
  pinMode(motorL2, INPUT);
  //pinMode(pwmL, INPUT); Coincides with one of the Stepper motor pins



if (state == true){  //Claw servo and Stepper motors seem to freeze Receiver - unless packet updated, prevents signal sent when 'rec' cannot be updated
  
//Claw servo - pin 13, connected to 5v external power supply, as current draw interferes with RF receiver.
if (claw < 1){
  Cservo.attach(13); Cservo.write(1520);}
else{ Cservo.write(900); delay(30); Cservo.detach();}


//Arm base stepper motor:
if (T_ < 100){
  digitalWrite(10, HIGH);
 //Serial.println("A1");
    stepperA("L");}
else if (T_>700){
 // Serial.println("A2");
  digitalWrite(10, HIGH);
   stepperA("R");}

//Rotating Base stepper motor:
if (B_ < 100){
  digitalWrite(10, HIGH);
 // Serial.println("B1");
    stepperB("R");}
else if (B_>700){
  //Serial.println("B2");
  digitalWrite(10, HIGH);
   stepperB("L");}

//Stepper Motors use power to maintain position: we disconnect the power-line when not in use to save battery.
//Pin 10 controls signal controls the state of the MOSFET

if ((100 < B_ < 700) && (100 <T_<700)){
  digitalWrite(10,LOW);} //MOSFET signal
}




//Neck Servo control 
current_timeH = millis();
if (current_timeH - previous_timeH >30){  //Maybe use 35 
  previous_timeH = current_timeH;
  
   if (N_ > 700){
    Hservo.detach();Nservo.attach(6);
    Npos+=25;
   if (Npos > 2200){Npos = 2200;}
   Nservo.write(Npos);
}

else if (N_< 100){
  Hservo.detach(); Nservo.attach(6);
   Npos-=25;
   if (Npos < 750){Npos = 750;}
   Nservo.write(Npos);
}

//Rear-hinge Servo control:
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
}

int switch_state = 1; //Keeps track of what mode Arduino switches from: Z(arm-claw) to D (drive) mode.
int secondary_switch = 0; //Changes to 1 after when switching from D(Drive) to Z (Claw) mode.


//Main loop: performs packet fetching, logic decision on mode: for motor, stepper and servo pin control. And Watch-dog timer state.
void loop(){

//Serial.println("Main loop");

//Receives radio packets and decomposes the String in the data packet
custom_isr(); 

if (mode == "D"){
wdt_disable(); //Prevents unnecessary resets for low probability events

 if (switch_state == 0){
    switch_state = 1; Serial.println("Resetting"); 
    wdt_enable(WDTO_250MS); //WDT required to reset all pin states: as activating servo pins disables PWM for L293D control pins
    while(1){}; //Trips WDT after 250ms - resets board
    }

  
  // Activating motor pins:  
pinMode(motorR1, OUTPUT);
  pinMode(motorR2, OUTPUT);
  pinMode(pwmR,OUTPUT);
  pinMode(motorL1, OUTPUT);
  pinMode(motorL2, OUTPUT);
  pinMode(pwmL, OUTPUT);
  
drive_mode();
}

//Initialises mode to control the Claw and Arm:
else if (mode == "Z"){
  
  wdt_disable(); //Prevents unnecessary resets for low probability events
 switch_state = 0; //Serial.println("Claw mode");

  if (secondary_switch == 1){
 Serial.println("Claw Resetting");
 wdt_enable(WDTO_250MS);
 while(1);}
  
  pinMode(5,OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(13,OUTPUT);

//Servo^ and Stepper Motor pins setup:

  pinMode(A0, OUTPUT);  
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);

  pinMode(2,OUTPUT);  
  pinMode(3,OUTPUT);  
  pinMode(A4,OUTPUT);  
  pinMode(A5,OUTPUT);
  
  control_mode();}


//Checker for RF receiver - Serial prints for diagnostic purposes
if (packetstate = true){

//Serial.println(rec);
//Serial.println(claw);
//Serial.println(mode);
}}
