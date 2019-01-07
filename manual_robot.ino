#include <EnableInterrupt.h>
#include <Servo.h>

#define SERIAL_PORT_SPEED 57600
#define RC_NUM_CHANNELS  6

//define RC plane transmitter channels
#define RC_CH1  0
#define RC_CH2  1
#define RC_CH3  2
#define RC_CH4  3
#define RC_CH5  4
#define RC_CH6  5

//define RC plane receiver connection to arduino mega
#define RC_CH1_INPUT  A8
#define RC_CH2_INPUT  A9
#define RC_CH3_INPUT  A10
#define RC_CH4_INPUT  A11
#define RC_CH5_INPUT  A12
#define RC_CH6_INPUT  A13

Servo servo1;
Servo servo2;
Servo servo3;
float pos=0, pos1=0;

//for motion dc motor connection(base motors for motion)
int en1 = 24;
int en2 = 26;

int en3 = 28;
int en4 = 30;   

int en5 = 25;
int en6 = 27;

int en7 = 29;
int en8 = 31;

//if you want to use dc motor for upper part rotation uncomment below part
//for upper part ratation(dc motor connection)
int en9 = 32;
int en10 = 33;
int enB = 4;

//if you want to use stepper motor for upper part rotation uncomment below part
//for upper part rotation(stepper motor connection)
/*const int enA = 5;
const int dir = 3;
const int stepPin  = 4;*/


//for rc receiver 
uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];

  //========================================READING RC PLANE TRASNMITTER'S VALUE========================================//

void rc_read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}

void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
  }
}

void calc_ch1() { calc_input(RC_CH1, RC_CH1_INPUT); }
void calc_ch2() { calc_input(RC_CH2, RC_CH2_INPUT); }
void calc_ch3() { calc_input(RC_CH3, RC_CH3_INPUT); }
void calc_ch4() { calc_input(RC_CH4, RC_CH4_INPUT); }
void calc_ch5() { calc_input(RC_CH5, RC_CH5_INPUT); }
void calc_ch6() { calc_input(RC_CH6, RC_CH6_INPUT); }


//========================================START========================================//


void setup() {
  Serial.begin(SERIAL_PORT_SPEED);
  //if you want to use stepper motor for upper part rotation uncomment below part
  //pinMode(enA, OUTPUT);      //Enable 6
  //pinMode(stepPin, OUTPUT);  //Step 5
  //pinMode(dir, OUTPUT);      //Direction 4
  //digitalWrite(enA,LOW);
  
  //servos connection
  servo1.attach(9);
  servo2.attach(10);
  servo3.attach(11);
  
  //dc motors connection for base
  pinMode(en1, OUTPUT);
  pinMode(en2, OUTPUT);
  pinMode(en3, OUTPUT);
  pinMode(en4, OUTPUT);
  pinMode(en5, OUTPUT);
  pinMode(en6, OUTPUT);
  pinMode(en7, OUTPUT);
  pinMode(en8, OUTPUT);
  
  //if you want to use dc motor for upper part rotation uncomment below part
  pinMode(en9, OUTPUT);
  pinMode(en10, OUTPUT);
  pinMode(enB, OUTPUT);
  
  //RC plane receiver connection
  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH3_INPUT, INPUT);
  pinMode(RC_CH4_INPUT, INPUT);
  pinMode(RC_CH5_INPUT, INPUT);
  pinMode(RC_CH6_INPUT, INPUT);
  
  enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
  enableInterrupt(RC_CH3_INPUT, calc_ch3, CHANGE);
  enableInterrupt(RC_CH4_INPUT, calc_ch4, CHANGE);
  enableInterrupt(RC_CH5_INPUT, calc_ch5, CHANGE);
  enableInterrupt(RC_CH6_INPUT, calc_ch6, CHANGE);
}

void loop() {
  rc_read_values();
  // read RC transmitter's values print the result to the serial monitor
  Serial.print("CH1:"); Serial.print(rc_values[RC_CH1]); Serial.print("\t");
  Serial.print("CH2:"); Serial.print(rc_values[RC_CH2]); Serial.print("\t");
  Serial.print("CH3:"); Serial.print(rc_values[RC_CH3]); Serial.print("\t");
  Serial.print("CH4:"); Serial.print(rc_values[RC_CH4]); Serial.print("\t");
  Serial.print("CH5:"); Serial.print(rc_values[RC_CH5]); Serial.print("\t");
  Serial.print("CH6:"); Serial.println(rc_values[RC_CH6]);


  //setting motion to channel 6 value
  if(rc_values[RC_CH5]<1500){
    //Forward
    if(rc_values[RC_CH2]>1700){
      digitalWrite(en1, LOW);  
      digitalWrite(en2, HIGH);
      digitalWrite(en3, LOW);  
      digitalWrite(en4, HIGH);
      digitalWrite(en5, LOW);   
      digitalWrite(en6, HIGH);
      digitalWrite(en7, HIGH);   
      digitalWrite(en8, LOW);
      Serial.println("forward");
      }
    //Backward
    else if(rc_values[RC_CH2]<1300){
      digitalWrite(en1, HIGH);   
      digitalWrite(en2, LOW);
      digitalWrite(en3, HIGH); 
      digitalWrite(en4, LOW);
      digitalWrite(en5, HIGH);  
      digitalWrite(en6, LOW);
      digitalWrite(en7, LOW);   
      digitalWrite(en8, HIGH);
      Serial.println("backward");
      }
    //Left turn
    else if(rc_values[RC_CH1]<1300){
      digitalWrite(en1, HIGH);   
      digitalWrite(en2, LOW);
      digitalWrite(en3, LOW);  
      digitalWrite(en4, HIGH);
      digitalWrite(en5, HIGH);   
      digitalWrite(en6, LOW);
      digitalWrite(en7, HIGH);   
      digitalWrite(en8, LOW);
      Serial.println("left");
      }
    //Right turn
    else if(rc_values[RC_CH1]>1700){
      digitalWrite(en1, LOW);   
      digitalWrite(en2, HIGH);
      digitalWrite(en3, HIGH);  
      digitalWrite(en4, LOW);
      digitalWrite(en5, LOW);  
      digitalWrite(en6, HIGH);
      digitalWrite(en7, LOW);  
      digitalWrite(en8, HIGH);
      }
    //Stop
    else if((rc_values[RC_CH2]<1700) && (rc_values[RC_CH2]>1300) && (rc_values[RC_CH4]>1300) && (rc_values[RC_CH4]<1700)){
      digitalWrite(en1, LOW);   
      digitalWrite(en2, LOW);
      digitalWrite(en3, LOW);  
      digitalWrite(en4, LOW);
      digitalWrite(en5, LOW);  
      digitalWrite(en6, LOW);
      digitalWrite(en7, LOW);  
      digitalWrite(en8, LOW);
      Serial.println("stop");
      }
    }
    
  //setting robotics to channel 6 value
  else if(rc_values[RC_CH5]>1500){

    //if you want to use dc motor for upper part rotation uncomment below part
    //for dc motor control
    if(rc_values[RC_CH1]>1700){
      digitalWrite(en9, HIGH);
      digitalWrite(en10, LOW);
      analogWrite(enB, 150);
      Serial.print("UP forward");
    }
    else if(rc_values[RC_CH1]<1300){
      digitalWrite(en9, LOW);
      digitalWrite(en10, HIGH);
      analogWrite(enB, 150);
      Serial.print("UP backward");
      }
    else if(rc_values[RC_CH1]>1300 && rc_values[RC_CH1]<1700 && rc_values[RC_CH2]>1300 && rc_values[RC_CH2]<1700){
      digitalWrite(en9, LOW);
      digitalWrite(en10, LOW);
      analogWrite(enB, 150);
      Serial.print("UP stop");
      }
    
    //if you want to use stepper motor for upper part rotation uncomment below part        
     //for stepper motor control 
    /*if(rc_values[RC_CH1]>1700){  
      digitalWrite(dir,HIGH);
      Serial.print("anticlockwise");
      for(Index = 0; Index < 20; Index++){
        digitalWrite(stepPin,HIGH);
        delayMicroseconds(500);
        digitalWrite(stepPin,LOW);
        delayMicroseconds(500);
        }
      }
    else if(rc_values[RC_CH1]<1300){
      digitalWrite(dir,LOW);
      Serial.print("clockwise");
      for(Index = 0; Index < 20; Index++){
        digitalWrite(stepPin,HIGH);
        delayMicroseconds(500);
        digitalWrite(stepPin,LOW);
        delayMicroseconds(500);
        }
      }*/
     
     //for servo1 ,2 control
     else if(rc_values[RC_CH2]<1300){
        pos -= 0.5;
        if(pos<0) pos = 0;
        Serial.println(pos);
        servo1.write(int(pos));
        servo2.write(int(180-pos));  
        } 
     else if(rc_values[RC_CH2]>1700){
        pos += 0.5;
        if(pos>180) pos = 180;
        Serial.println(pos);
        servo1.write(int(pos));
        servo2.write(int(180-pos));  
        }
    
    //for gripper's servo
    else if(rc_values[RC_CH4]>1700){
      pos1 += 0.5;
      if(pos1>180) pos1 = 180;
      Serial.println(pos1);
      servo3.write(int(pos1));
      }
    else if(rc_values[RC_CH4]<1300){
      pos1 -= 0.5;
      if(pos1<0) pos1 = 0;
      Serial.println(pos1);
      servo3.write(pos1);
      }
    }
}

//========================================END========================================//
