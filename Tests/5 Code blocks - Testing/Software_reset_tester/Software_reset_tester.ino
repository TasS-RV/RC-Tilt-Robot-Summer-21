#include <avr/wdt.h>
int n = 0;
void setup() {
  Serial.begin(9600);
pinMode(13, OUTPUT);
}


void (*resetFunc) (void) = 0;



void loop() {
 
for (n; n <= 6; n++){
  digitalWrite(LED_BUILTIN, HIGH);
 delay(800);
 digitalWrite(LED_BUILTIN, LOW);
 delay(800);
  }
//resetFunc();
  
} 
 
