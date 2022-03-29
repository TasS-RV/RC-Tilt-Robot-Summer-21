//Receiver connected to Pin 3, button to interrupt pin 2, and LED indicator to pin 13

#include <avr/sleep.h>
#include <RH_ASK.h>
#include <SPI.h> // Not actualy used but needed to compile
const int rc_period =40;

RH_ASK driver(2000,3,10,0);
String rec; unsigned long previousmillis = 0; 

//Interrupt pin definitions
int receiver_test_n = 0;
#define wake_pin 2

//Variables for decomposition of the R/C packet
String Pitch; String Roll; 

#define motorR1 9
#define motorR2 12
#define pwmL 5
#define pwmR 6
#define motorL1 8
#define motorL2 10

int P; int pwmR_value; int pwmL_value; bool packetstate;


////////////////////////////////////////////////////////
void setup()
{
    Serial.begin(9600);	// Debugging only
    if (!driver.init())
         Serial.println("init failed");

    attachInterrupt(wake_pin, wake_, RISING);

  pinMode(motorR1, OUTPUT);
  pinMode(motorR2, OUTPUT);
  pinMode(pwmR,OUTPUT);
  pinMode(motorL1, OUTPUT);
  pinMode(motorL2, OUTPUT);
  pinMode(pwmL, OUTPUT);
  
         
}

void wake_(){
//Will not work, as the Radio signal triggers the arduino to wake up 
  
/*  if (S_state = "Off"){
    sleep_enable();
    S_state = "On";digitalWrite(13,HIGH);}
  else if (S_state = "On"){
    sleep_disable();
    S_state = "Off";digitalWrite(13,LOW);}
*/
  sleep_disable(); digitalWrite(13,LOW);
  receiver_test_n = 0;
  digitalWrite(2,LOW);
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
   // Serial.println(rec); //Serial.println(receiver_test_n);//When running the actual code, remove any serial prints as they are interrupts which will interfere with pwm signals
    receiver_test_n += 1;   
  }

if ((int)rec.length()>=20){
    //Data packet decomposition:
Pitch = rec.substring(rec.indexOf("V")+4, rec.indexOf("T"));
Roll =  rec.substring(rec.indexOf("V")+1, rec.indexOf("V")+4);

int P_print = Pitch.toInt(); int R_print = Roll.toInt();
P = P_print;
//Serial.print("Pitch:");Serial.println(P_print);
//Serial.print("Roll:");Serial.println(R_print);


if (abs(P)< 15){
  P = 1;}
else if (abs(P)>70){
  P = 70;}
else {P = P;} float P_ = P;

if (P != 1){pwmR_value = 105.0+abs((P_/70.0)*155.0);}
else if (P = 1){pwmR_value = 0;}
pwmR_value = round((int) pwmR_value);

//Outputs and updating packet 
    rec = "";return receiver_test_n; return P; return pwmR_value; return packetstate = true; 

    
  }
else{
  return packetstate = false;}
}
//Mian motor driver loop
void loop(){
custom_isr();

if (packetstate = true){
Serial.println(P);
Serial.println(pwmR_value);
}    
///////////////////////////////////

//!!!Value retention has been achieved - flyback voltage issue still remains (diodes placed in circuit board)!!!

//Driving motor control
if (P >=-10 && P <=10){
 // Serial.println("Neutral");

  analogWrite(pwmR, pwmR_value);
  analogWrite(pwmL, pwmR_value); 
  digitalWrite(motorL1, LOW);
  digitalWrite(motorL2, LOW);
  digitalWrite(motorR1, LOW);
  digitalWrite(motorR2, LOW); 
  digitalWrite(2, HIGH);
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
  digitalWrite(2, HIGH);
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
  digitalWrite(2, HIGH);
 // exit(1);
}
//digitalWrite(2, HIGH);
}

 
  
  
  
