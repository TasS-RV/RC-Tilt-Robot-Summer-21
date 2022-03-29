//Receiver connected to Pin 3, button to interrupt pin 2, and LED indicator to pin 13

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

//Variables for Claw and Arm control:
int T_; int H_;

////////////////////////////////////////////////////////
void setup()
{
    Serial.begin(9600);	// Debugging only
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
*/
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

//Processing data form the Joysticks:
String T = rec.substring(rec.indexOf("T")+1, rec.indexOf("P"));
String H = rec.substring((rec).indexOf("P")+1, rec.indexOf("A"));
T_ =  T.toInt(); H_ = H.toInt();



//Outputs and updating packet 
    rec = "";return receiver_test_n; return P; return R; return pwmR_value; return pwmL_value; return packetstate = true;
    return H_;return T_; 

    
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
Serial.println(T_);
Serial.println(H_);
}}    
