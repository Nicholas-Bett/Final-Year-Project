#include <Wire.h>
//#include <LiquidCrystal.h>
#include "DFRobot_RGBLCD1602.h"
//#include "DFRobot_LCD.h"
#include<SoftwareSerial.h>
#include "EmonLib.h"           
#define VOLT_CAL 592

DFRobot_RGBLCD1602 lcd(16,2);
SoftwareSerial SIM900(0, 1);
#define Motion_Sensor A3 //Button pin, on the other pin it's wired with GND
#define TempSensorPin A0
bool Sensor_State; //Sensor_State

const int colorR = 255;
const int colorG = 0;
const int colorB = 0;
const int trigPin = 8; // Trigger Pin of Ultrasonic Sensor
const int echoPin = 7; // Echo Pin of Ultrasonic Sensor
float duration;
int OilLevel;
const int currentPin = A1;
//float sensitivity = 66;
float adcValue= 0;
float offsetVoltage = 2.5;
double adcVoltage = 0;
double LoadCurrent = 0;
int delayTime = 300;
float vout;
const int voltageSensor = A2;
float vOUT = 0.0;
float vIN = 0.0;
float v = 0.0;
float max_v = 0;
int value = 0;
float c = 0;
//float max_i = 0;
float sensitivity = 0.185;
int sensorValue = 0;
float temp; 
float current = 0;
float supplyVoltage;
void setup() {
  // put your setup code here, to run once:
pinMode(TempSensorPin,INPUT);
pinMode(trigPin, OUTPUT);
pinMode(echoPin, INPUT);
 Serial.begin(19200);
 SIM900.begin(19200);
 delay(2000);
//Serial.println(" GSM-BASED TRANSFORMER MONITORING");  
lcd.init();
lcd.setRGB(colorR, colorG, colorB);
lcd.autoscroll();
lcd.print("PROJECT:GSM-BASED");
lcd.setCursor(0,0);
lcd.print("TRANSFORMER");
//lcd.setCursor(0,2);
lcd.print("MONITORING");
//lcd.setCursor(0,3);
lcd.print("SYSTEM");
delay (500);
lcd.noAutoscroll();
lcd.clear();
}
uint16_t get_maxv() {
  uint16_t max_v = 0;
  for(uint8_t i = 0; i < 100; i++) {
    uint16_t value = analogRead(voltageSensor);  // read from analog channel 3 (A0) 
    v = value;
    if(max_v < v) max_v = v;
    delayMicroseconds(40);
  }
  return max_v;
}
uint16_t get_maxi() {
  uint16_t  max_i = 0;
  for(uint8_t i = 0; i < 100; i++) {
    uint16_t adcValue = analogRead(currentPin);
     adcVoltage = (adcValue *( 5/1023.0));
     c = ((adcVoltage - offsetVoltage)/sensitivity);
     current = c;
    if(max_i < current) max_i = current;
    delayMicroseconds(40);
  }
  return max_i;
}


void loop() {

//temperature//
vout = analogRead(TempSensorPin);
vout = (vout * 500) / 1023;
temp = vout/30;

//LOAD CURRENT//

uint32_t current = get_maxi();
c = current;
c/=sqrt(2);
LoadCurrent = c;
// SENSING OIL LEVEL//
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
duration = pulseIn(echoPin, HIGH);
OilLevel = duration * 0.034 / 2;

//LOAD VOLTAGE
uint32_t v = get_maxv();
  v /= sqrt(2);
vIN = v;
//IR SENSOR
Sensor_State = digitalRead(Motion_Sensor);   //We are constantly reading the button State
//HALL EFFECT SENSOR
//sensorValue = analogRead(A2);
 //LCD DISPLAY
 
lcd.setCursor(0,0);
lcd.print("Temp:");
lcd.setCursor(6,0);
lcd.print(temp);
lcd.setCursor(11,0);
lcd.print("C");

lcd.setCursor(0,1);
lcd.print("Oil L:");
lcd.setCursor(6,1);
lcd.print(OilLevel);
lcd.setCursor(8,1);
lcd.print("CM");
delay(5000);
lcd.clear();

lcd.setCursor(0, 0);
lcd.print("IL:");
lcd.setCursor(3, 0);
lcd.print(LoadCurrent);
lcd.setCursor(7, 2);
lcd.print("A");

lcd.setCursor(9, 0);
lcd.print("VL:");
lcd.setCursor(12, 0);
lcd.print(vIN);
lcd.setCursor(15, 0);
lcd.print("V");

lcd.setCursor(0, 1);
lcd.print("IR:");
lcd.setCursor(5, 1);
if (Sensor_State == HIGH){
lcd.print("High");
}
else {
lcd.print("Low");  
}
delay(5000);
lcd.clear();

condition();

}


void sms1()

{
SIM900.print("AT+CMGF=1\r"); 
SIM900.println("AT + CMGS = \"+254716602827\"");// recipient's mobile number
Serial.println("AT + CMGS = \"+254716602827\"");// recipient's mobile number
SIM900.println("HIGH TRANSFORMER TEMPERATURE"); // message to send
Serial.println("HIGH TRANSFORMER TEMPERATURE");
SIM900.println((char)26); // End AT command with a ^Z, ASCII code 26
Serial.println((char)26);
SIM900.println();
}


void sms2(){
SIM900.print("AT+CMGF=1\r"); 
SIM900.println("AT + CMGS = \"+254716602827\"");// recipient's mobile number
Serial.println("AT + CMGS = \"+254716602827\"");// recipient's mobile number
SIM900.println("LOW OIL LEVEL"); // message to send
Serial.println("LOW OIL LEVEL");
SIM900.println((char)26); // End AT command with a ^Z, ASCII code 26
Serial.println((char)26);
SIM900.println();
}

void sms3(){
SIM900.print("AT+CMGF=1\r"); 
SIM900.println("AT + CMGS = \"+254716602827\"");// recipient's mobile number
Serial.println("AT + CMGS = \"+254716602827\"");// recipient's mobile number
SIM900.println("OVERLOAD CURRENT"); // message to send
Serial.println("OVERLOAD CURRENT");
SIM900.println((char)26); // End AT command with a ^Z, ASCII code 26
Serial.println((char)26);
SIM900.println();

}
void sms4()
{
SIM900.print("AT+CMGF=1\r"); 
SIM900.println("AT + CMGS = \"+254716602827\"");// recipient's mobile number
Serial.println("AT + CMGS = \"+254716602827\"");// recipient's mobile number
SIM900.println("TRANSFORMER OVERVOLTEGE"); // message to send
Serial.println("TRANSFORMER OVERVOLTEGE");
SIM900.println((char)26); // End AT command with a ^Z, ASCII code 26
Serial.println((char)26);
SIM900.println();
}

void sms5()
{
SIM900.print("AT+CMGF=1\r"); 
SIM900.println("AT + CMGS = \"+254716602827\"");// recipient's mobile number
Serial.println("AT + CMGS = \"+254716602827\"");// recipient's mobile number
SIM900.println("TRANSFORMER TAMPERING DETECTED"); // message to send
Serial.println("TRANSFORMER TAMPERING DETECTED");
SIM900.println((char)26); // End AT command with a ^Z, ASCII code 26
Serial.println((char)26);
SIM900.println();
}
//CONDITIONS
void condition()
{
  if (temp > 75)
{
lcd_tempPrint();
sms1();
 delay(300);
  
  }
if (OilLevel < 300)
{lcd_oilLevelLOWPrint();
 sms2();
 delay(1000);
  }
if (LoadCurrent > 300)
{
  lcd_currentPrint();
 sms3();
 delay(1000);
  }
 if (vIN > 415)
{
  lcd_vINPrint();
 sms4();
 delay(1000);
  }
  if (Sensor_State == HIGH) {            //And if it's pressed
    lcd_tamperingPrint();
    sms5();                          //And this function is called
delay(1000);
 }
 if (sensorValue < 300)
{
lcd_tamperingPrint();
sms5();
 delay(1000);
 }
}


// LCD PRINTING CONDITION//

void lcd_currentPrint()
{
lcd.clear();
lcd.setCursor(0,0);                                //if condition temp for it to print.
lcd.print("OVERLOAD CURRENT");
delay(500);
lcd.clear();  
}

void lcd_oilLevelLOWPrint()
{
lcd.clear();
lcd.setCursor(0,1);                                 //if condition temp for it to print.
lcd.print("OIL LEVEL LOW");  
delay(500);
lcd.clear();
}


void lcd_tempPrint()
{
lcd.clear();
lcd.setCursor(0,0);                                //if condition temp for it to print.
lcd.print("HIGH TEMPERATURE");
delay(500);
lcd.clear();    
}
void lcd_vINPrint()
{
lcd.clear();
lcd.setCursor(0,1);                               //if condition temp for it to print.
lcd.print("OVERVOLTAGE");  
delay(500);
lcd.clear(); 
}
void lcd_tamperingPrint()
{
lcd.clear();
lcd.setCursor(0,0);                              //if condition temp for it to print.
lcd.print("TAMPERING DETECTED");  
delay(500);
lcd.clear(); 
}
