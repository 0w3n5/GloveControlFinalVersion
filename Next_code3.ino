
//did not include wire.h as it should already be available for use
#include <Adafruit_Sensor.h>             //loads library
#include "Adafruit_TMP006.h"
#include <SoftwareSerial.h>

//connect VCC to +3V (quieter than the 5V supply)
// Gnd to Gnd
//SCL to I2C clock pin (may be SCL or labled A5)
//SDA to I2C data pin (either SDA or if not available use A4)


Adafruit_TMP006 tmp006 =(0x47);           //default I2C address (0x41)

  
  const int trigPin = 10;   //trig pin of distance sensor connected to pin 10
  const int echoPin = 11;    //echo pin of distance sensor to pin 11
  const int Motor1 = 9;      //Motor 1 connected to pin 9       
  const int Motor2 = 8;     //Motor 2 is assigned pin 8

  long objt; //object temperature
  long duration;    //defines variables
  int distance;     //defines variables

  SoftwareSerial mySerial(A4,A5);


void setup() 
{
 pinMode(trigPin, OUTPUT);  //trigPin set as output
 pinMode(echoPin, INPUT);   //echoPin set as input
 pinMode(Motor1, OUTPUT);   //Motor assigned to distance sensor
 Serial.begin(9600); //Starts Serial
 while (!Serial){
  ; //wait for serial port to be connected
 }
 
 Serial.println("Distance:");

 mySerial.begin(9600);
 mySerial.println("Temperatures");
 unsigned long previousMillis = 0;
 const long interval = 1000;
}

void loop() 
{
  digitalWrite(trigPin, LOW);   //clears trig pin
  delay(2);                      //delay of 2ms
  digitalWrite(trigPin, HIGH);  //trigPin high for 10ms
  delay(10);
  digitalWrite(trigPin, LOW);   //resets trig pin
  duration = pulseIn(echoPin,HIGH);  //Duration soundwave traveled for in ms
  distance = duration*0.034/2;   //distance soundwave traveled/2   
//speed = 34000cm/s
//duration in sx10-6
//speed*duration = 34000*10-6 times duration
     
 if (mySerial.available())
 Serial.write(mySerial.read());
 if (Serial.available())
 mySerial.write(Serial.read());
 
 // Check for sleep/wake command.
  if (distance < 30){
    float objt = tmp006.readObjTempC();
    Serial.print("Object Temperature: "); Serial.print(objt); Serial.println("*C");
    float diet = tmp006.readDieTempC();
    Serial.print("Die Temperature: "); Serial.print(diet); Serial.println("*C");
  }
  
  if ((objt > 45))                                            //if object temperature is under the warning temperature the code for the ultrasonic sensor is ignored and the
  {analogWrite(Motor2, 255);                             //heat warning is played
  delay(500);
  analogWrite(Motor2, 0);
  delay(10);
  } //if object temperature is under warning temperature the code for the ultrasonic sensor is ran on
  
  Serial.println(distance);
  if ((distance < 10) && (objt < 45))
  {
   analogWrite(Motor1, 255);    //moderates motor speed
   delay(200);                    
   analogWrite(Motor1, 0);
   delay(100);
  }
  else if ((distance < 20) && (distance >= 10) && (objt<45))
  {
   analogWrite(Motor1, 127);
   delay(200);
   analogWrite(Motor1, 0);
   delay(200);
  }
  else if ((distance >= 20) && (distance <= 30) && (objt<45))
  {
   analogWrite(Motor1, 63);
   delay(200);
   analogWrite(Motor1, 0);
   delay(400);
  }
  else if ((distance < 10) && (objt<45))
  {
    digitalWrite(Motor1, LOW);
    delay(300);
  }
  }
