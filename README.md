// lcd,Potentiometer and Motor programming//
#include<LiquidCrystal_I2C.h>        // library for lcd interfacing
#include<Wire.h>
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);// create lcd instance
#define RPWM 5
#define LPWM 6
#define REN 8
#define LEN 9


int pot;
int out1;
int out2;
//==============sensor
unsigned int magSensor = 3; 
volatile unsigned int revCount = 0;
volatile unsigned int seconds=0;
volatile unsigned int counter = 0; 
bool secondFlag = 0; 
bool magSensorFlag = 0; 
volatile unsigned int rpm=0; 
bool delayFlag=0;
//===========================
void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);
  pinMode(RPWM,OUTPUT);
  pinMode(LPWM,OUTPUT);
  pinMode(LEN,OUTPUT);
  pinMode(REN,OUTPUT);
  digitalWrite(REN,HIGH);
  digitalWrite(LEN,HIGH);
//====================
lcd.setCursor(0, 0);
lcd.print("Auto Coffee ");
lcd.setCursor(0, 1);
lcd.print("Peeling Machine");
delay(4000);                          
lcd.clear();
//===============================Sensor
 pinMode(magSensor,INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(magSensor), magSensorISR, FALLING);

   cli();//stop interrupts

//set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
//  OCR1A = 7812;// = (8*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();//allow interrupts
  seconds=0;
}
//=========================Nested Function
void magSensorISR()
{
  magSensorFlag=1; 
  counter++;
  if(counter>=1)// number of slated magnets for now we have one magnet
  {
   revCount++;
   counter=0;
  }
  
}

ISR(TIMER1_COMPA_vect){//timer1 interrupt 1Hz
  
 seconds++;

 if(seconds==60)
 {
  secondFlag=1;
  seconds=0;
 }
 
}

 //========================================
 
void loop() {
  
  pot=analogRead(A3);

    out1=map(pot,0,1023,255,0);
    analogWrite(RPWM,out1);
    analogWrite(LPWM,0);
//=========================
 if(magSensorFlag==1)
  {

    lcd.setCursor(0,1);
    lcd.print("rev=");
    lcd.print(revCount);
    
    magSensorFlag=0;
  }

  if(secondFlag==1)
  {

    if(revCount>0)
    {
      rpm = revCount;
      ///rpm = (revCount*6);
      //lcd.clear(); 
      revCount=0;
    }
    else {rpm=0;}

    lcd.clear();  
    lcd.setCursor(0,0);
    lcd.print("Sensor");
    lcd.setCursor(0,1);
    lcd.print("RPM=");
    lcd.print(rpm);
//    lcd.setCursor(0,1);
//    lcd.print("rev=");
//    lcd.print(revCount);
    secondFlag=0;

    delay(5000);
    delay(5000);
    seconds=0;
    rpm=0;
    revCount=0;
    
    lcd.clear();  
    lcd.setCursor(0,0);
    lcd.print("Sensor");
//    lcd.setCursor(8,1);
//    lcd.print("RPM=");
//    lcd.print(rpm);
    lcd.setCursor(0,1);
    lcd.print("rev=");
    lcd.print(revCount);
    secondFlag=0;

  }
//===============================

}
