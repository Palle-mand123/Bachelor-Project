#include <Servo.h>

Servo myServo1; // Servo.h architecture
Servo myServo2; //
Servo myServo3; // 
bool extended1 = LOW; //Bool to rectract servo.
bool extended2 = LOW;
bool extended3 = LOW;
int Servo1_pin = 2; //Pins to activate servo
int Servo2_pin = 3;
int Servo3_pin = 4;
int Distance_pin_1 = 5; //Pins to actiave Distance_sensor
int Distance_pin_2 = 6;
int Distance_out_1 = 7; //Output pins for distance sensor.
int Distance_out_2 = 8;
bool First_run = LOW; //Bool to ensure servos drive back at startup.
int Distance_sensor_1 = A0; //Input from the actual distance sensor
int Distance_sensor_2 = A1; //input from the acutal distance sensor
double threshold = 20; //found to be a good enough threshold, could be chagned
double threshold_close =  490-threshold; //490 is when box is infront.
double threshold_long = 232+threshold;  //232 is when there is nothing.
double sensorValue_1; 
double sensorValue_2;




void setup() {
  //Serial.begin(115200); //For serial communcation if needed
  pinMode(Servo1_pin, INPUT);
  pinMode(Servo2_pin, INPUT);
  pinMode(Servo3_pin, INPUT);
  pinMode(Distance_sensor_1, INPUT);
  pinMode(Distance_sensor_2, INPUT);
  pinMode(Distance_out_1,OUTPUT);
  pinMode(Distance_out_2,OUTPUT);
  pinMode(Distance_pin_1,INPUT);
  pinMode(Distance_pin_2,INPUT);
  digitalWrite(Distance_out_2,HIGH);
  digitalWrite(Distance_out_1,HIGH);
}

void Drive_servo_1() //Drive function, is the same for every servo.
{ 
  myServo1.attach(A2);
  myServo1.write(160); // Set to 160° 
  delay(1000);
  extended1 = HIGH;
}

void retract_servo_1() //Rectract function, is the same for every servo.
{
  myServo1.write(0); // Set to 0°
  delay(1000);
  extended1 = LOW;
  myServo1.detach();

}

void Drive_servo_2()
{
  myServo2.attach(A3);
  myServo2.write(160); // Set to 160°
  delay(1000);
  extended2 = HIGH;
}

void retract_servo_2()
{
  myServo2.write(0); // Set to 0° 
  delay(1000);
  extended2 = LOW;
  myServo2.detach();

}

void Drive_servo_3()
{
  myServo3.attach(A4);
  myServo3.write(160); // Set to 160°
  delay(1000);
  extended3 = HIGH;
}

void retract_servo_3()
{
  myServo3.write(0); // Set to 0°
  delay(1000);
  extended3 = LOW;
  myServo3.detach();
}

void Distance_sensor_1_read() //read and write function. Delays could be change if needed.
{
  sensorValue_1 = analogRead(Distance_sensor_1);
  Serial.println(sensorValue_1);
  if (sensorValue_1 > threshold_close) 
  {
    Serial.println("OBJECT CLOSE DIST 1");
    digitalWrite(Distance_out_1,LOW);
  }
  if (sensorValue_1 < threshold_long) 
  {
    Serial.println("OBJECT FAR DIST 1");
    digitalWrite(Distance_out_1,HIGH);
  }
  delay(20);
}

void Distance_sensor_2_read() //read and write function. Delays could be change if needed.
{
  sensorValue_2 = analogRead(Distance_sensor_2);
  Serial.println(sensorValue_2);
  if (sensorValue_2 > threshold_close) 
  {
    Serial.println("OBJECT CLOSE DIST 2");
    digitalWrite(Distance_out_2,LOW);
  }
  if (sensorValue_2 < threshold_long) 
  {
    Serial.println("OBJECT FAR DIST 2");
    digitalWrite(Distance_out_2,HIGH);
  }
  delay(20);
}
void loop() {
  if (First_run == LOW) //Could prehabs be put in setup.
  {
    retract_servo_1();
    delay(500);
    retract_servo_2();
    delay(500);
    retract_servo_3();
    delay(500);
    First_run = HIGH;
  }
  
  if (digitalRead(Distance_pin_1)==HIGH)
  {
  Distance_sensor_1_read();
  }
  if(digitalRead(Distance_pin_2)==HIGH)
  {
  Distance_sensor_2_read();
  }
  if (digitalRead(Servo1_pin) == HIGH)
  {
    Drive_servo_1();
  }
  if (digitalRead(Servo2_pin)==HIGH)
  {
    Drive_servo_2();
  }
  if (digitalRead(Servo3_pin)==HIGH)
  {
    Drive_servo_3();
  }

  if (extended1 == HIGH)
  {
    delay(500);
    retract_servo_1();
  }
  if(extended2 == HIGH)
  {
    delay(500);
    retract_servo_2();
  }
  if(extended3 == HIGH)
  {
    delay(500);
    retract_servo_3();
  }
  delay(25);
}


