//Set servo pins to INPUT when running the motors - check if that fixes
//Intialise a watchdog timer - which is kicked when the arduino enters an infinite while loop on chnagin back to drive mode
//If not: disconnect one of the stepper pins, connect to pwmR pin and use digital write on thw arduino to reset pin - this
//causes a manual hard reset.


#include <avr/wdt.h>
int n = 0;

void setWDT(byte WDT_setting){
  WDTCSR |= 0b00011000;
  WDTCSR =  WDT_setting | WDTO_1S;
  wdt_reset();
  }

void setup() {
  pinMode(13, HIGH);
  pinMode(A5,INPUT); 
  
wdt_disable();
//wdt_enable(WDTO_1S);
//setWDT(0b00001000); //<--- Arduino reset mode. Note: 0b01000000 can be used for Interrupt mode
}

void loop() {

for (n; n<6; n+=1){digitalWrite(LED_BUILTIN,HIGH); wdt_reset(); delay(800);
digitalWrite(LED_BUILTIN, LOW); wdt_reset(); delay(800);}



delay(1500);
pinMode(A5, OUTPUT); //Must initialise pin as input - high impedance mode, then connect to reset pin - simply altering to output mode is sufficient to trigger the reset pin 

//while(1){}
}
