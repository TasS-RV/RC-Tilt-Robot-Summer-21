//Receiver connected to Pin 3, button to interrupt pin 2, and LED indicator to pin 13
//THIS WORKLS: Don't touch it


#include <avr/sleep.h>
#include <RH_ASK.h>
#include <SPI.h> // Not actualy used but needed to compile

RH_ASK driver(2000,9,10,0);
String rec; unsigned long previousmillis = 0; 

int receiver_test_n = 0;
#define wake_pin 2

void setup()
{
    Serial.begin(9600);	// Debugging only
    if (!driver.init())
         Serial.println("init failed");

    attachInterrupt(wake_pin, wake_, CHANGE);
         
}

void wake_(){
  sleep_disable();
  digitalWrite(13,LOW);
  receiver_test_n = 0;
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

    
void custom_isr(){
  unsigned long currentmillis = millis();
  if (currentmillis - previousmillis >= 200) {
    reception();

    previousmillis = currentmillis;
    Serial.println(rec); Serial.println(receiver_test_n);//When running the actual code, remove any serial prints as they are interrupts which will interfere with pwm signals
    receiver_test_n += 1;   
    
    rec = ""; return receiver_test_n;
    }
}

void loop(){
 if (receiver_test_n <= 100){
  custom_isr();}

 else {
  digitalWrite(13,HIGH);
  sleep_enable();
 }
}
 
  
  
  
