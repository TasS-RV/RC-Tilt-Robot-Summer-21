#include <RH_ASK.h>
#ifdef RH_HAVE_HARDWARE_SPI
#include <SPI.h> // Not actually used but needed to compile
#endif
//Pins: Rec,Trans, Push
RH_ASK driver(2000,9, 10, 0); 

#include <MPU6050_tockn.h>
#include <Wire.h>
MPU6050 mpu6050(Wire);

#include <LiquidCrystal_I2C.h>
#include <Wire.h>
// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);
long t_offset;
long timer;
long lcd_time; //LCD will update independently from timing of the sensors

int y1 = A0;
int x1 = A1;
int y2 = A2;
int x2 = A3;
float compl_a;

int claw = 8; int mode  = 9; int switchstate = 1;
float roll;
float pitch;
String roll_;
String pitch_;

int buttonnew = 1; int buttonold = 1; long current_time; long prev_time;
int new_switch = 1; String mode_ = "D"; long moment;
 

//Momentarily displays present mode - when changing to update information to the user
void mode_display(){
if (mode_ == "D"){
  lcd.setCursor(5,0);lcd.print("Drive");
  lcd.setCursor(6,1);lcd.print("Mode");delay(1800);} //Period for which message remains on screen}

  
 else if (mode_ == "Z"){
 lcd.setCursor(2,0); lcd.print("Claw Control");
 lcd.setCursor(6,1); lcd.print("Mode");delay(1800);} //Period for which message remains on screen}
lcd.clear(); //Removes update at end - feeds back data from sensors
}


void setup()
{
  Wire.begin();
 lcd.begin();   
  lcd.backlight();
pinMode(claw, INPUT);

#ifdef RH_HAVE_SERIAL
    Serial.begin(9600);    // Debugging only
#endif
    if (!driver.init())
#ifdef RH_HAVE_SERIAL
         Serial.println("init failed");
#else
#endif
pinMode(x1,INPUT);
pinMode(x2,INPUT);
pinMode(y1,INPUT);
pinMode(y2,INPUT);

mpu6050.begin();
lcd.setCursor(0,0);
lcd.print("Calibrating...");
//Progran takes approx 10 seconds to initialise
mpu6050.calcGyroOffsets(true);

lcd.clear();
//Time offset required to maintain timing
t_offset = millis();

pinMode(mode,INPUT);
mode_display(); //Initialises with info about current mode
}

String mode_print; String state_; int pwm_value;

//Main loops: updates MPU for angle readings, updates lcd on sensor and button data and transmits data packet
void loop()
{mpu6050.update();

//lcd and mpu will update every transmission cycle
  
int x_val1 = analogRead(x1);
if (x_val1 > 999){
  x_val1 = 999;}
else{
  x_val1 = x_val1;}
//Restricting char array size to length of 3 bytes
int y_val1 = analogRead(y1);
if (y_val1 > 999){
  y_val1 = 999;}
else{
  y_val1 = y_val1;}
int x_val2 = analogRead(x2);
if (x_val2 > 999){
  x_val2 = 999;}
else{
  x_val2 = x_val2;}
int y_val2 = analogRead(y2);
if (y_val2 > 999){
  y_val2 = 999;}
else{
  y_val2 = y_val2;}

//Accelerometer angles update every 0.1 seconds
if (millis()-timer > 5){
  
long t_elapsed = millis()-t_offset;
float acc_a = atan(mpu6050.getAccY()/mpu6050.getAccZ());
/*
int gyr_a = mpu6050.getGyroAngleX();
compl_a = round(0.92*(gyr_a+(compl_a/t_elapsed))+0.08*(acc_a));
float acc_deg = ((acc_a/(2*PI))*360); //atan() returns values in radians so requires deg conversion 
Serial.print(gyr_a);Serial.print(',');
Serial.println(compl_a);
//Complementary filter not in use now*/

roll = ((acc_a/(2*PI))*360);
pitch = atan(mpu6050.getAccX()/mpu6050.getAccZ());
pitch = ((pitch/(2*PI))*360);
}
roll - (int) roll; pitch = (int) pitch;
roll_; pitch_;


if (roll<0){
  roll_ = round(roll);}
else{
  roll_ = "+"+String(round(roll));}
if (pitch<0){
  pitch_ = round(pitch);}
else{
  pitch_ = "+"+String(round(pitch));}    
//  Serial.print("Roll:");Serial.print(roll_);
  //Serial.print("Pitch:");Serial.println(pitch_);

if (roll_.length()<6){
  roll_ = roll_.substring(0,1)+"0"+roll_.substring(1,roll_.indexOf("."));} //Index of can be used to locate, literally the index of a particular
  //character of a string. 
else if (roll_.length() == 6){
  roll_ = roll_.substring(0,roll_.indexOf("."));}
else{
  roll_ = "00";}
//for data transmission, similar to bearings the angle must be a specific number of digits
if (pitch_.length()<6){
  pitch_ = pitch_.substring(0,1)+"0"+pitch_.substring(1,pitch_.indexOf("."));}
else if (pitch_.length() == 6){
  pitch_ = pitch_.substring(0,pitch_.indexOf("."));}
else{
  pitch_ = "00";}
//Note! Cannot use Pulse in for detecting input due to Interrupts affecting the Rc transmitter






//   Toggle switch for claw and control modes:   //

buttonnew = digitalRead(claw);

if (buttonnew == 0 && buttonold == 1){
  buttonold = buttonnew;  //Button press
//  Serial.println("Pressed");
  prev_time = millis();
}
if(buttonnew == 1 && buttonold == 0){
  //Button depress - end timing and alter switch state
  buttonold = buttonnew; 

  //Serial.println("Removed");
  current_time  = millis(); moment  = current_time - prev_time;
  //Switchstate logic:
  if (moment <1000){
    if (new_switch == 1){
    new_switch = 0;
    }
  else if (new_switch == 0){
    new_switch = 1;}}
          
  if (moment>= 1900){ //Long press will toggle the control modes
    if (mode_ == "D"){
      mode_ = "Z";
      lcd.clear(); mode_display();}
    else if (mode_ == "Z"){
      mode_ = "D";
      lcd.clear(); mode_display();}  
  }}
int claw_read = new_switch; //Reads state of the switch - opens or closes claw depending on toggle state



String pack = "V"+roll_+pitch_+"T"+String(x_val1)+"P"+String(y_val1)+"A"+String(y_val2)+"H"+String(x_val2)+"C"+String(claw_read)+String(mode_);
//Specific data transmission packet which is decoded at other end by robot
  
//  String pack = "V+02+00T527P524A511H501C1D";  Ending is D or Z - depending on drive or arm control mode for the vehicle

int str_len = pack.length()+1;
char data[str_len];
pack.toCharArray(data,str_len);

Serial.println(data);

//Transmission code for RC transmitter
const char *packet = "testing the message 49458023+- length";
driver.send((uint8_t *)data, strlen(data));
    //driver.waitPacketSent();
    delay(2); //Not waiting for packet transmission does increase rate, but causes additional delays
/*
 * DO NOT implement the timing fixation - code delays update mpu6050
if (millis()-timer>150){
  timer = millis();}
else{
  timer = timer;}
 */
String xval1; xval1 = String(x_val1);
String yval1; yval1 = String(y_val1);
String(xval2); xval2 = String(x_val2);
String(yval2); yval2 = String(y_val2);


//This block of code will display the 2 separate lines on LCD:


if (lcd_time>193){  //Refreshes LCD approximately at 5Hz 
lcd.clear(); //Pixel overwriting due to cursor repositioning can leave copies of characters - poor formatting!

//Z has minimal meaning but was needed to not intefere with the "C" in the sent packet - "C" refers to 'Claw' due to link to initials
if (mode_ == "D"){mode_print = "D";} else if (mode_ == "Z"){mode_print = "C";}

//Pitch, Roll and pwm data to each wheel shown:
if (mode_ == "D"){
lcd.setCursor(12,1); lcd.print("M:"+mode_print);
//Print pitch and roll data
lcd.setCursor(1,1);lcd.print("P");
lcd.print(pitch_);
lcd.setCursor(6,1);lcd.print("R");
lcd.print(roll_);


//Any tilt above 70% is considered full throttle - 100% pwm value: requires conversion of pitch and roll 
int P = pitch_.toInt(); int R = roll_.toInt();
 
if (abs(P)<=15){P = 0;} else if (P>= 70){P = 70;} else if (P<=-70){P = -70;} else {P = P;}
if (abs(R)<=15){R = 0;} else if (R>= 70){R = 70;} else if (R<=-70){R = -70;} else {R = R;}


//Roll overrides the pitch value:

if(P <= 20 && abs(R)>=14){ float R_ = R; 
pwm_value = abs(round((R_/70.0)*100.0));
if (R<0){lcd.setCursor(1,0); lcd.print("L:" + String(pwm_value) + "%"); lcd.setCursor(8,0); lcd.print("R: 0%");}
else if (R>0){lcd.setCursor(1,0); lcd.print("L: 0%"); lcd.setCursor(8,0); lcd.print("R:" + String(pwm_value) +"%");}
else {lcd.setCursor(1,0); lcd.print("L: 0%"); lcd.setCursor(8,0); lcd.print("R: 0%");}
}

else if (abs(P)> 0){ float P_ = P;
pwm_value =  abs(round((P_/70.0)*100.0));
lcd.setCursor(1,0);lcd.print("L:" + String(pwm_value) + "%"); lcd.setCursor(8,0);lcd.print("R:" + String(pwm_value) + "%"); 
}

//By default prints forward and backwards PWM values:
else if (P == 0 && R == 0){  
lcd.setCursor(1,0);lcd.print("L: 0%"); lcd.setCursor(8,0);lcd.print("R: 0%"); 
}}



//Joystick values claw state if in claw/arm control mode:

else if (mode_ == "Z"){

lcd.setCursor(12,1); lcd.print("M:"+mode_print);
  //Updating LCD to print data 
lcd.setCursor(0,0);lcd.print("1X");
//Maintaining the character size of 3 bytes - prevents too rapid update of lcd
if (xval1.length()==2){
  xval1 = "0"+xval1;}

else if (xval1.length() == 3){
  xval1 = xval1;}
else {xval1 = "010";}
///////////////////////
if (yval1.length()==2){
  yval1 = "0"+yval1;}
else if (yval1.length() == 3){
  yval1 = yval1;}
else {yval1 = "010";}
///////////////////////
if (xval2.length()==2){
  xval2 = "0"+xval2;}
else if (xval2.length() == 3){
  xval2 = xval2;}
else {xval2 = "010";}
//////////////////////
if (yval2.length()==2){
  yval2 = "0"+y_val2;}
else if (yval2.length() == 3){
  yval2 = yval2;}
else {yval2 = "010";}  
///////////////////////  
lcd.print(xval1);
lcd.setCursor(6,0);lcd.print("Y");
lcd.print(yval1);
lcd.setCursor(0,1);lcd.print("2X");
lcd.print(xval2);
lcd.setCursor(6,1);lcd.print("Y");
lcd.print(yval2);

lcd.setCursor(12,0);lcd.print("C:"+String(claw_read)); //1 indicates claw is closed and active - 0 indicates open claw
  }}

//Rather than using a delay, we can simply check the time elapsed between updates (preventing program lag)
if (millis()-lcd_time>20){
  lcd_time = millis();}
else{
  lcd_time = lcd_time;}
}
//End of Program
