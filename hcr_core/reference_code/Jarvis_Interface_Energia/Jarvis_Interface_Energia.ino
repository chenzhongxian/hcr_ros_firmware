/*
#  Chefbot_ROS_Interface.ino
 #  
 #  Copyright 2015 Lentin Joseph <qboticslabs@gmail.com>
 #  Website : www.qboticslabs.com , www.lentinjoseph.com
 #  This program is free software; you can redistribute it and/or modify
 #  it under the terms of the GNU General Public License as published by
 #  the Free Software Foundation; either version 2 of the License, or
 #  (at your option) any later version.
 #  
 #  This program is distributed in the hope that it will be useful,
 #  but WITHOUT ANY WARRANTY; without even the implied warranty of
 #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 #  GNU General Public License for more details.
 #  
 #  You should have received a copy of the GNU General Public License
 #  along with this program; if not, write to the Free Software
 #  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 #  MA 02110-1301, USA.
 #  
 #  Some of the portion is adapted from I2C lib example code for MPU 6050
 */


//MPU 6050 Interfacing libraries

//Library to communicate with I2C devices
#include "Wire.h"
//I2C communication library for MPU6050
#include "I2Cdev.h"
//MPU6050 interfacing library
#include "MPU6050_6Axis_MotionApps20.h"
//Processing incoming serial data 
#include <Messenger.h>
//Contain definition of maximum limits of various data type
#include <limits.h>

//Creating MPU6050 Object
MPU6050 accelgyro(0x68);
//Messenger object
Messenger Messenger_Handler = Messenger();

/////////////////////////////////////////////////////////////////////////////////////////
//DMP options
//Set true if DMP init was successful
bool dmpReady = false;
//Holds actual interrupt status byte from MPU
uint8_t mpuIntStatus;
//return status after each device operation
uint8_t devStatus;
//Expected DMP paclet size
uint16_t packetSize;
//count of all bytes currently in FIFO
uint16_t fifoCount;
//FIFO storate buffer
uint8_t fifoBuffer[64];


#define OUTPUT_READABLE_QUATERNION
// #define OUTPUT_READABLE_YAWPITCHROLL
/////////////////////////////////////////////////////////////////////////////////////////

//orientation/motion vars
Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;

float euler[3];
float ypr[3];




// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

/////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////
//Encoder pins definition
// Left encoder
#define Left_Encoder_PinA PF_4
#define Left_Encoder_PinB PD_7

volatile long Left_Encoder_Ticks = 0;
volatile long Pre_Left_Encoder_Ticks = 0;
volatile bool LeftEncoderBSet;

//Right Encoder
#define Right_Encoder_PinA PD_6
#define Right_Encoder_PinB PC_7
volatile long Right_Encoder_Ticks = 0;
volatile long Pre_Right_Encoder_Ticks = 0;
volatile bool RightEncoderBSet;

/////////////////////////////////////////////////////////////////////////////////////////
//Motor Pin definition
//Left Motor pins
#define A_1 PA_3
#define B_1 PA_4
//PWM 1 pin number
#define PWM_1 PC_6

//Right Motor
#define A_2 PE_4 
#define B_2 PE_5
//PWM 2 pin number
#define PWM_2 PC_5

/////////////////////////////////////////////////////////////////////////////////////////
//Ultrasonic pins definition
// const int echo = 9, Trig = 10;
// long duration, cm;

/////////////////////////////////////////////////////////////////////////////////////////
//Battery level monitor for future upgrade
// #define BATTERY_SENSE_PIN PC_4

// float battery_level = 12;

//Reset pin for resetting Tiva C, if this PIN set high, Tiva C will reset
#define RESET_PIN PB_2

/////////////////////////////////////////////////////////////////////////////////////////
//Time  update variables
unsigned long LastUpdateMicrosecs = 0;		
unsigned long LastUpdateMillisecs = 0;
unsigned long CurrentMicrosecs = 0;
unsigned long MicrosecsSinceLastUpdate = 0;
float SecondsSinceLastUpdate = 0;

/////////////////////////////////////////////////////////////////////////////////////////
//Motor speed from PC
//Motor left and right speed
float motor_left_speed = 0;
float motor_right_speed = 0;

int M1Speed = 0;  // 电机速度-137到137,-137反转最大速度, 0电机停止, 137正转最大速度
int M2Speed = 0;  

// wheelPerimeter = pi * 136mm / 1000 = 0.4273 m
// wheelSpeedMax = 146 * wheelPerimeter / 60 = 1.0397 m/s
// float wheelPerimeter = 0.4273;
float wheelSpeedMax = 1.0397;

static int state1 = 0;        //state为灯的状态
boolean abortStatus = false;

/////////////////////////////////////////////////////////////////////////////////////////
//Setup serial, encoders, ultrasonic, MPU6050 and Reset functions
/////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  //Init Serial port with 115200 baud rate
  Serial.begin(115200);  // 与上位机通信 RX(1) --> PC_4   TX(1) -->PC_5
  // Serial2.begin(57600);  // 与电机控制器通信 RX(2) --> PD_6   TX(2) -->PD_7 (VBUS detection)
  Serial3.begin(57600);  // 与电机控制器通信 RX(3) --> PC_6   TX(3) -->PC_7

  // Wire.begin(3);   // SCL(3) --> PD_0   SDA(3) --> PD_1 
  // Wire.setModule(3);
  // Wire.begin();  

  //Setup MPU 6050
  // Setup_MPU6050();
  //Setup Encoders
  // SetupEncoders();
  //Setup Motors
  // SetupMotors();
  //Setup Ultrasonic
  // SetupUltrasonic();  
  //Setup Reset pins
  SetupReset();

  motorCleanEncoder();

  //Set up Messenger 
  Messenger_Handler.attach(OnMssageCompleted);
}

/////////////////////////////////////////////////////////////////////////////////////////
//SetupEncoders() Definition
/////////////////////////////////////////////////////////////////////////////////////////

// void SetupEncoders()
// {
//   // Quadrature encoders

//   // Left encoder
//   pinMode(Left_Encoder_PinA, INPUT_PULLUP);      // sets pin A as input  
//   pinMode(Left_Encoder_PinB, INPUT_PULLUP);      // sets pin B as input
//   //Attaching interrupt in Left_Enc_PinA.
//   attachInterrupt(Left_Encoder_PinA, do_Left_Encoder, RISING);

//   // Right encoder
//   pinMode(Right_Encoder_PinA, INPUT_PULLUP);      // sets pin A as input
//   pinMode(Right_Encoder_PinB, INPUT_PULLUP);      // sets pin B as input
//   //Attaching interrupt in Right_Enc_PinA.
//   attachInterrupt(Right_Encoder_PinA, do_Right_Encoder, RISING); 
// }


/////////////////////////////////////////////////////////////////////////////////////////
//Setup UltrasonicsSensor() function
/////////////////////////////////////////////////////////////////////////////////////////

// void SetupUltrasonic()
// {
//   pinMode(Trig, OUTPUT);
//   pinMode(echo, INPUT); 
// }



/////////////////////////////////////////////////////////////////////////////////////////
//Setup Reset() function
/////////////////////////////////////////////////////////////////////////////////////////

void SetupReset()
{
  pinMode(GREEN_LED,OUTPUT);
  pinMode(RESET_PIN,OUTPUT);

  ///Conenect RESET Pins to the RESET pin of launchpad,its the 16th PIN
  digitalWrite(RESET_PIN,HIGH);
}


/////////////////////////////////////////////////////////////////////////////////////////
//MAIN LOOP
/////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
  //Read from Serial port
  Read_From_Serial();

  //Send MPU 6050 values through serial port
  // Update_MPU6050();

  //Send time information through serial port
  Update_Time();

  //Send encoders values through serial port
  Update_Encoders();

  //Send ultrasonic values through serial port
  // Update_Ultra_Sonic();

  //Update motor values with corresponding speed and send speed values through serial port
  Update_Motors();

  //Send battery values through serial port
  // Update_Battery();
}

/////////////////////////////////////////////////////////////////////////////////////////
//Read from Serial Function
/////////////////////////////////////////////////////////////////////////////////////////

void Read_From_Serial()
{
  while(Serial.available() > 0)
  {
    int data = Serial.read();
    Messenger_Handler.process(data);
  } 
}

/////////////////////////////////////////////////////////////////////////////////////////
//OnMssg Complete function definition
/////////////////////////////////////////////////////////////////////////////////////////

void OnMssageCompleted()
{
  char reset[] = "r";
  char set_speed[] = "s";

  if(Messenger_Handler.checkString(reset))
  {
    Serial.println("Reset Done"); 
    Reset();
  }
  if(Messenger_Handler.checkString(set_speed))
  {
    //This will set the speed
    Set_Speed();
    return; 
  }
}

/////////////////////////////////////////////////////////////////////////////////////////
//do_Left_Encoder() Definitions
/////////////////////////////////////////////////////////////////////////////////////////

// void do_Left_Encoder()
// {
//   // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
//   LeftEncoderBSet = digitalRead(Left_Encoder_PinB);   // read the input pin
//   Left_Encoder_Ticks -= LeftEncoderBSet ? -1 : +1;
// }

/////////////////////////////////////////////////////////////////////////////////////////
//do_Right_Encoder() Definitions
/////////////////////////////////////////////////////////////////////////////////////////

// void do_Right_Encoder()
// {
//   RightEncoderBSet = digitalRead(Right_Encoder_PinB);   // read the input pin
//   Right_Encoder_Ticks += RightEncoderBSet ? -1 : +1;
// }



/////////////////////////////////////////////////////////////////////////////////////////
//Reset function
/////////////////////////////////////////////////////////////////////////////////////////

void Reset()
{
  digitalWrite(GREEN_LED,HIGH);
  delay(1000);
  digitalWrite(RESET_PIN,LOW);
  digitalWrite(GREEN_LED,LOW); 
}

/////////////////////////////////////////////////////////////////////////////////////////
//Will update ultrasonic sensors through serial port
/////////////////////////////////////////////////////////////////////////////////////////

// void Update_Ultra_Sonic()
// {
//   digitalWrite(Trig, LOW);
//   delayMicroseconds(2);
//   digitalWrite(Trig, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(Trig, LOW);
//   // The echo pin is used to read the signal from the PING))): a HIGH
//   // pulse whose duration is the time (in microseconds) from the sending
//   // of the ping to the reception of its echo off of an object.
//   duration = pulseIn(echo, HIGH);
//   // convert the time into a distance
//   cm = microsecondsToCentimeters(duration);

//   //Sending through serial port
//   Serial.print("u");
//   Serial.print("\t");
//   Serial.print(cm);
//   Serial.print("\n");
// }




/////////////////////////////////////////////////////////////////////////////////////////
//Update battery function
/////////////////////////////////////////////////////////////////////////////////////////

// void Update_Battery()
// {
//   battery_level = analogRead(PC_4); 

//   Serial.print("b");
//   Serial.print("\t");
//   Serial.print(battery_level);
//   Serial.print("\n");
// }




