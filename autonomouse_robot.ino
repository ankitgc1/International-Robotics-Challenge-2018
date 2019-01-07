#include <NewPing.h>
#include <LiquidCrystal.h>

#define TRIGGER_PINL  A3  // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PINL     A0  // Arduino pin tied to echo pin on ping sensor.

#define MAX_DISTANCE 500 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

#define TRIGGER_PINF  A4  // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PINF     A1  // Arduino pin tied to echo pin on ping sensor.

#define TRIGGER_PINR  A5  // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PINR     A2  // Arduino pin tied to echo pin on ping sensor.

int wallL = 0, wallF = 0, wallR = 0, ser = 0; 

int en1 = 2 ;
int en2 = 3 ;

int en3 = 4 ;
int en4 = 5 ;

int enA = 10 ;
int enB = 11 ;

int bs = 150, ps = 120;
int red = 0, blue = 0, green = 0, CR1 = 0, CR2 = 0, CR3 = 0;

//for Arduino Screen
const int rs = 12, en = 13, d4 = 9, d5 = 8, d6 = 7, d7 = 6;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//for UltraSonic Sensor
NewPing sonarLeft(TRIGGER_PINL, ECHO_PINL, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonarRight(TRIGGER_PINR, ECHO_PINR, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonarFront(TRIGGER_PINF, ECHO_PINF, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

unsigned int pingSpeed = 30; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
unsigned long pingTimer;     // Holds the next ping time.


float oldLeftSensor, oldRightSensor, leftSensor, rightSensor, frontSensor, oldFrontSensor, lSensor, rSensor, fSensor;
int dist = 20;

float error, preverror = 0, PID = 0, P = 0, I = 0, D = 0, elasped, prev, time;
float Kp = 2;
float Kv = 5;
float Kd = 0.001;

void setup() {
  // difining all the connection
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
  lcd.begin(16, 2);     //define lcd connection
  //define all motors connection
  pinMode(en1, OUTPUT);  
  pinMode(en2, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(en3, OUTPUT);
  pinMode(en4, OUTPUT);
  pinMode(enB, OUTPUT);
  lcd.print("Start");
  delay(1000);
}

void loop() {
  //========================================START========================================//

  lcd.clear();
  ReadSensors();

  // read sensors & print the result to the serial monitor 
  Serial.print("Left : ");
  Serial.print(leftSensor);
  Serial.print(" cm ");
  Serial.print(" Right : ");
  Serial.print(rightSensor);
  Serial.print(" cm ");
  Serial.print(" Front : ");
  Serial.print(frontSensor);
  Serial.println(" cm ");

  wallL = 0, wallF = 0, wallR = 0;
 //========================================WHEN BLOCK COME TO PATH========================================//
  if (Serial.available()) {
    ser = Serial.read();
    Serial.println(ser);
    if (ser == '1') {
      CR1 = 1;
      red = 1;
      lcd.print("Box = Red");
      if (frontSensor < dist) {
      digitalWrite(en1, LOW);
      digitalWrite(en2, LOW);
      analogWrite(enA, bs);
      digitalWrite(en3, LOW);
      digitalWrite(en4, LOW);
      analogWrite(enA, bs);
      delay(2000);
    }
    }
    else if (ser == '2') {
      CR2 = 1;
      blue = 1;
      lcd.print("Box = Blue");
      if (frontSensor < dist) {
      digitalWrite(en1, LOW);
      digitalWrite(en2, LOW);
      analogWrite(enA, bs);
      digitalWrite(en3, LOW);
      digitalWrite(en4, LOW);
      analogWrite(enA, bs);
      delay(2000);
      }
    }
    else if (ser == '3'){
      CR3 = 1;
      green = 1;
      lcd.print("Box = Green");
      if (frontSensor < dist) {
      digitalWrite(en1, LOW);
      digitalWrite(en2, LOW);
      analogWrite(enA, bs);
      digitalWrite(en3, LOW);
      digitalWrite(en4, LOW);
      analogWrite(enA, bs);
      delay(2000);
    }
    }
    else if (ser == '4' && (CR1 + CR2 + CR3) >= 2){
      lcd.print("QR read");
      if (frontSensor < dist) {
        digitalWrite(en1, LOW);
        digitalWrite(en2, LOW);
        analogWrite(enA, bs);
        digitalWrite(en3, LOW);
        digitalWrite(en4, LOW);
        analogWrite(enA, bs);
        delay(2000);
      }
    }
  }


  
  //========================================WHEN THERE IS NO BLOCKS========================================//
  else {
    if(leftSensor<10){
      wallL = 1;
      }
    if(frontSensor<15){
      wallF = 1;
      }
    if(rightSensor<10){
      wallR = 1;
      }
    //RFl algorithm
    //forward
    if (wallR == 1 && wallF != 1) {

      digitalWrite(en1, LOW);   
      digitalWrite(en2, HIGH);
      analogWrite(enA, bs);
      digitalWrite(en3, HIGH);  
      digitalWrite(en4, LOW);
      analogWrite(enB, bs);
      Serial.print("forward");
    }
    //left trun
    else if (wallR == 1 && wallF == 1 && wallL != 1) {
      digitalWrite(en1, LOW);   
      digitalWrite(en2, HIGH);
      analogWrite(enA, bs -50);
      digitalWrite(en3, HIGH);  
      digitalWrite(en4, LOW);
      analogWrite(enB, bs + 50);
      Serial.print("left turn");
    }
    //right trun
    else if (wallR != 1) {
      digitalWrite(en1, LOW);  
      digitalWrite(en2, HIGH);
      analogWrite(enA, bs + 50);
      digitalWrite(en3, HIGH);  
      digitalWrite(en4, LOW);
      analogWrite(enA, bs - 50);
      Serial.print("right turn");
    }
    else if (wallL == 1 && wallF == 1 && wallR == 1) {
      digitalWrite(en1, LOW);   
      digitalWrite(en2, HIGH);
      analogWrite(enA, bs);
      digitalWrite(en3, LOW);  
      digitalWrite(en4, HIGH);
      analogWrite(enA, bs);
      Serial.print("backward");
      delay(1000);
    }
  }
}
  //========================================ULTRASONIC SENSORS READING DATA========================================//
void ReadSensors() {

  //leftSensor = sonarLeft.ping_median(TestNUM);     //accurate but slow
  //rightSensor = sonarRight.ping_median(TestNUM);     //accurate but slow
  //frontSensor = sonarFront.ping_median(TestNUM);     //accurate but slow

  //leftSensor = sonarLeft.convert_cm(leftSensor);
  //rightSensor = sonarRight.convert_cm(rightSensor);
  //frontSensor = sonarFront.convert_cm(frontSensor);

  lSensor = sonarLeft.ping_cm(); //ping in cm
  rSensor = sonarRight.ping_cm();
  fSensor = sonarFront.ping_cm();


  leftSensor = (lSensor + oldLeftSensor) / 2; //average distance between old & new readings to make the change smoother
  rightSensor = (rSensor + oldRightSensor) / 2;
  frontSensor = (fSensor + oldFrontSensor) / 2;


  oldLeftSensor = leftSensor; // save old readings for movment
  oldRightSensor = rightSensor;
  oldFrontSensor = frontSensor;

  //PID algorithum
  prev = time;
  time = millis();
  elasped = time - prev;
  error = leftSensor - rightSensor;
  if (error > 30) error = 30;
  if (error < -30) error = -30;
  P = Kp * error;
  I += Kd * error;
  D = Kv * (error - preverror) / elasped;
  preverror = error;
  PID = P + I + D;
  if (PID > 30) PID = 30;
  else if (PID < -30) PID = -30;

}

  //========================================END========================================//
