#include <RH_ASK.h>
#ifdef RH_HAVE_HARDWARE_SPI
#include <SPI.h> // Not actually used but needed to compile
#endif
//Pins: Rec,Trans, Push
RH_ASK driver(2000,11, 10, 0); 

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

int claw = 8;
float roll;
float pitch;
String roll_;
String pitch_;

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
}




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
  Serial.print("Roll:");Serial.print(roll_);
  Serial.print("Pitch:");Serial.println(pitch_);

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


//Claw open and close toggle state:
int claw_read = digitalRead(claw);


String pack = "V"+roll_+pitch_+"T"+String(x_val1)+"P"+String(y_val1)+"A"+String(y_val2)+"H"+String(x_val2)+"C"+String(claw_read);
//Specific data transmission packet which is decoded at other end by robot
  
//  String pack = "V+02+00T527P524A511H501C1";

int str_len = pack.length()+1;
char data[str_len];
pack.toCharArray(data,str_len);

Serial.println(data);

//Transmission code for RC transmitter
const char *packet = "testing the message 49458023+- length";
driver.send((uint8_t *)data, strlen(data));
    //driver.waitPacketSent();
    delay(5); //Not waiting for packet transmission does increase rate, but causes additional delays
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

if (lcd_time>190){ 
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
lcd.setCursor(12,0);lcd.print("P");
lcd.print(pitch_);
lcd.setCursor(12,1);lcd.print("R");
lcd.print(claw_read);
//lcd.print(roll_);}


//Rather than using a delay, we can simply check the time elapsed between updates (preventing program lag)
if (millis()-lcd_time>20){
  lcd_time = millis();}
else{
  lcd_time = lcd_time;}
}
