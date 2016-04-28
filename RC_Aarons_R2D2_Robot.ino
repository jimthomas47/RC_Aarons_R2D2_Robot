/*
  Aaron's R2D2 Robot V2.1
  designed with love by Papa:  Jim Thomas Jan 2014
  modified for Aarons R2D2 remote control Mar 2016
  select: Arduino Mega 2560 controller
  ---- NRF 24L01+ 2.4Ghz Radio Receiver-----
  works with RC transmitterr (RC_controller_NRF2401)
  This sketch receives joystick values and button codes
  ----Differential Drive Robot Chassis----
  L298N Dual H Bridge
  Right DC motor, Left DC  motor as viewed from the castor in rear
  received analog values drive two motors L and R on the robot
  The analog values received are already scaled to -255 to +255
  postive value drive the motor forward  power 0 to 255 max
  negative value drive the motor reverse power absvalue 0-255 max
  v2.0 upgrades
  single joystick drive for motors using 45 degree coordinate rotation algorithm
  v2.1 upgrades
  adding WTV020-SD-16P module to produce variety of sounds, R2D2 beeps, robot voices
  parent hack thru TV remote control etc.
  remove Alert Pin beeper
  Thanks and (C) to Manicbug for the NRF24L01 libraries
*/
/* WTV020-SD-16P audio module library
  Created by Diego J. Arevalo, August 6th, 2012.
  slight modifications to timing inside by Jim Thomas
  Released into the public domain.
*/

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#include <Servo.h>

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <JQ6500_Serial.h>

JQ6500_Serial mp3(31, 30);


int joystick[3];

// masks for each buttons in s_register
const int R1Mask = 0x10;
const int R2Mask = 0x20;
const int L1Mask = 0x1000;
const int L2Mask = 0x2000;
const int RPushMask = 0x40;
const int LPushMask = 0x4000;
const int B1Mask = 0x100;
const int B2Mask = 0x200;
const int B3Mask = 0x400;
const int B4Mask = 0x800;
const int LLMask = 0x1;
const int LRMask = 0x2;
const int LDMask = 0x4;
const int LUMask = 0x8;
const int PushMask = 0x4040; // push either joystick

uint16_t PreviousSreg = 0x0000;

// structure to hold the payload 10bytes
struct payload_r {
  uint16_t sreg;
  int16_t j_RUD;
  int16_t j_RLR;
  int16_t j_LUD;
  int16_t j_LLR;
};

/* NRF 24L01+ pin connections for Mega
  pin 53 CE; pin 48 CSN; pin 51 MOSI; pin 52 SCK; pin 50 MISO
*/
RF24 radio(48, 53);
const uint64_t pipe = 0xE8E8F0F0E1LL;

/*  Motor Drive Controller Board Module L298N Dual H Bridge
  connections to Ardunio pwm capable Outputs...
*/
const int RmotorPinF = 3;  // Right Forward connect to MC board in
const int RmotorPinR = 2;  // Right Reverse connect to MC board in
const int LmotorPinF = 4;  // Left Forward connect to MC board in
const int LmotorPinR = 5;  // Left Reverse connect to MC board in

const int HmotorPinF = 10; // R2D2 head turn
const int HmotorPinR = 11; // R2D2 head turn

// arrays of constants to convert tractor drive to joystick drive
// lookup motorPower = [j_RUD][j_RLR]
// 6x6 version
const int LmotorPower [7][7] = {
  { -255, -255, -255, -255, -200, -150, 0},
  { -255, -200, -200, -200, -150, 0, 150},
  { -255, -200, -150, -150, 0, 150, 200},
  { -255, -200, -150, 0, 150, 200, 255},
  { -200, -150, 0, 150, 150, 200, 255},
  { -100, 0, 100, 200, 200, 200, 255},
  {0, 150, 200, 255, 255, 255, 255}
};
const int RmotorPower [7][7] = {
  {0, -150, -200 - 255, -255, -255, -255},
  {150, 0, -150, -200, -200, -200, -255},
  {200, 150, 0, -150, -150, -200, -255},
  {255, 200, 150, 0, -150, -200, -255},
  {255, 200, 150, 150, 0, -150, -200},
  {255, 200, 200, 200, 150, 0, -150},
  {255, 255, 255, 255, 200, 150, 0}

};

// output pin definitions - to robot head
const int SpinMotor  = 28;  // blk - motor that puts spin on disc / red Vbatt
const int ShootMotor = 32;  // blue - motor that ejects disc / orange Vbatt
const int LeftEyeLed = 29;  // yellow - left eye led / gnd = black
const int RightEyeLed = 33; // red - right eye led / gnd = black

const int SpeakerPin = 6;   // speaker output pin

const int SpotPin = 7;  // spotlight output
const int SpotServo  = 12;  // small servo motor for LED spotlight
int SpotAngle = 0;  // spotlight servo angle in degrees
boolean SpotButton = false; // for toggle of spotlight
boolean SpotOn = false; // spotlight on state
Servo servo1;  // Spotlight servo

const int ArmServo = 13;  // large servo motor for Arm up/down
Servo servo2;  // Arm servo
int ArmAngle = 90;  // current arm angle.. constrain to 0-180
int ArmIncrement = 1;  // # degrees to increment servo angle

const int RotPin = 36; // blue rotating light pin (transistor drive)
boolean RotButton = false;
boolean RotOn = false;

boolean EyesButton = false; // robot eyes
boolean EyesOn  = false;

// LED's in robot's hand
const int HandLED1 = 35;
const int HandLED2 = 37;
const int HandLED3 = 39;
int LaserTone = 60; // variable tone for hand laser LEDs
int count = 0;
int lcount = 0;

// pins to control WTV020-SD-16P uSD Audio module in Wtv020sd16p
const int resetPin = 26;
const int clockPin = 27;
const int dataPin = 30;
const int busyPin = 31;


/*
  Create an instance of the Wtv020sd16p class.
  1st parameter: Reset pin number.
  2nd parameter: Clock pin number.
  3rd parameter: Data pin number.
  4th parameter: Busy pin number.
*/

void setup(void) {
  Serial.begin(9600);
  printf_begin();

  // motor pins as outputs, and motors off: set to 0V = LOW
  pinMode (RmotorPinF, OUTPUT);
  pinMode (RmotorPinR, OUTPUT);
  pinMode (LmotorPinF, OUTPUT);
  pinMode (LmotorPinR, OUTPUT);
  digitalWrite(RmotorPinF, LOW);
  digitalWrite(RmotorPinR, LOW);
  digitalWrite(LmotorPinF, LOW);
  digitalWrite(LmotorPinF, LOW);

  // R2D2 head motor
  pinMode (HmotorPinF, OUTPUT);
  pinMode (HmotorPinR, OUTPUT);
  digitalWrite(HmotorPinF, LOW);
  digitalWrite(HmotorPinR, LOW);

  // JQ6500
  mp3.begin(9600);
  mp3.reset();
  mp3.setVolume(25);
  mp3.setLoopMode(MP3_LOOP_NONE);

  // Robot head
  pinMode (SpinMotor, OUTPUT);
  pinMode (ShootMotor, OUTPUT);
  pinMode (LeftEyeLed, OUTPUT);
  pinMode (RightEyeLed, OUTPUT);
  digitalWrite(SpinMotor, LOW);
  digitalWrite(ShootMotor, LOW);
  digitalWrite(LeftEyeLed, LOW);
  digitalWrite(RightEyeLed, LOW);

  pinMode (SpeakerPin, OUTPUT);
  pinMode (SpotPin, OUTPUT);
  pinMode (RotPin, OUTPUT);
  pinMode (HandLED1, OUTPUT);
  pinMode (HandLED2, OUTPUT);
  pinMode (HandLED3, OUTPUT);


  servo1.attach(SpotServo);
  servo2.attach(ArmServo);
  servo2.write(ArmAngle);   // initialize to 90

  // start the radio
  radio.begin();
  radio.setChannel(0x4F); // change from default setting (0x4C) to avoid interference
  radio.setDataRate(RF24_250KBPS);  // slow data rate for better reliability
  radio.setPayloadSize(10);  // 10 byte payload
  radio.openReadingPipe(1, pipe);
  radio.startListening();
  radio.printDetails();

  mp3.playFileByIndexNumber(9);
}

void loop(void) {
  int x, y;
  payload_r payload = {0, 0, 0, 0, 0};

  if (radio.available()) {
    radio.read(&payload, sizeof(payload));
  }

  //Serial.println(payload.sreg,BIN);
  // printf(" %4i %4i %4i %4i ",payload.j_RUD,payload.j_RLR,payload.j_LUD,payload.j_LLR);
  // printf("\n");

  // R2D2: LEFT joystick controls  drive motors,  -255 to 255
  // note: j_LLR value comes in inversed

  // diagonal drive
  //joystick[0] = -payload.j_RLR;
  //joystick[1] = payload.j_RUD;

  // quantize joystick values 0-6

  if (-payload.j_RLR < -200)     x = 0;
  else if (-payload.j_RLR < -150)x = 1;
  else if (-payload.j_RLR < -10) x = 2;
  else if (-payload.j_RLR < 10)  x = 3;
  else if (-payload.j_RLR < 150) x = 4;
  else if (-payload.j_RLR < 200) x = 5;
  else x = 6;

  if (payload.j_RUD < -200)     y = 0;
  else if (payload.j_RUD < -150)y = 1;
  else if (payload.j_RUD < -10) y = 2;
  else if (payload.j_RUD < 10)  y = 3;
  else if (payload.j_RUD < 150) y = 4;
  else if (payload.j_RUD < 200) y = 5;
  else y = 6;

  // lookup motor power values from the motor power table
  joystick[0] = LmotorPower[y][x];
  joystick[1] = RmotorPower[y][x];

  Serial.print(joystick[0]);
  Serial.print(' ');
  Serial.println(joystick[1]);

  // R2D2 Head turn
  joystick[2] = payload.j_LLR / 2;

  uint16_t buttons = payload.sreg & ~PreviousSreg;
  PreviousSreg = payload.sreg;

  if (buttons & B1Mask) {
    mp3.playFileByIndexNumber(1);
  }

  if (buttons & B2Mask) {
    mp3.playFileByIndexNumber(5);
  }

  if (buttons & B3Mask) {
    mp3.playFileByIndexNumber(6);
  }

  if (buttons & B4Mask) {
    mp3.playFileByIndexNumber(7);
  }

  if (buttons & R2Mask) {
    mp3.playFileByIndexNumber(3);
  }

  if (buttons & L2Mask) {
    mp3.playFileByIndexNumber(4);
  }

  if (buttons & LUMask) {
    mp3.playFileByIndexNumber(2);
  }

  if (buttons & LLMask) {
    mp3.playFileByIndexNumber(10);
  }

  if (buttons & LDMask) {
    mp3.playFileByIndexNumber(11);
  }

  if (buttons & LRMask) {
    mp3.playFileByIndexNumber(12);
  }

  if (buttons & L1Mask) {
    mp3.playFileByIndexNumber(8);
  }

  if (buttons & R1Mask) {
    mp3.playFileByIndexNumber(9);
  }

  /*
      // LED Spot Light On / Off
      if (LUMask & payload.sreg) {
        if (SpotButton == false) { //  button low to transition so toggle the spot
          SpotButton = true;
          if (SpotOn == true) {
            SpotOn = false;
            digitalWrite (SpotPin, LOW);
          }
          else {
            SpotOn = true;
            digitalWrite (SpotPin, HIGH);
          }
        }
      }
      else {
        SpotButton = false;
      }

      // Blue Rotation Light On / Off
      if (LRMask & payload.sreg) {
        if (RotButton == false) { //  button low to transition so toggle the spot
          RotButton = true;
          if (RotOn == true) {
            RotOn = false;
            digitalWrite (RotPin, LOW);
          }
          else {
            RotOn = true;
            digitalWrite (RotPin, HIGH);
          }
        }
      }
      else {
        RotButton = false;
      }

      // Robot Eyes On / Off
      if (LLMask & payload.sreg) {
        if (EyesButton == false) { //  button low to transition so toggle the spot
          EyesButton = true;
          if (EyesOn == true) {
            EyesOn = false;
            (RightEyeLed, LOW);
            digitalWrite(LeftEyeLed, LOW);
          }
          else {
            EyesOn = true;
            digitalWrite(RightEyeLed, HIGH);
            digitalWrite(LeftEyeLed, HIGH);
          }
        }
      }
      else {
        EyesButton = false;
      }

      // LED Spot Light servo
      SpotAngle = (payload.j_LLR + 255) / 3; // 0 to 180ish
      servo1.write(SpotAngle);


      // Arm Servo L1 button decreases angle, L2 button increases angle
      // 0 all up, 180 all down
      if (L2Mask & payload.sreg) {
        if (ArmAngle + ArmIncrement < 180) {
          ArmAngle = ArmAngle + ArmIncrement;
          servo2.write(ArmAngle);
        }
      }
      if (L1Mask & payload.sreg) {
        if (ArmAngle - ArmIncrement > 0) {
          ArmAngle = ArmAngle - ArmIncrement;
          servo2.write(ArmAngle);
        }
      }
  */
  // Serial.print(joystick[0]);
  // Serial.print(' ');
  // Serial.println(joystick[1]);


  // left (port) motor
  if (joystick[0] > 0) {      // forward
    analogWrite(LmotorPinF, joystick[0]);
    digitalWrite(LmotorPinR, LOW);
  }
  else if (joystick[0] < 0) { //reverse
    digitalWrite(LmotorPinF, LOW);
    analogWrite(LmotorPinR, -joystick[0]);
  }
  else {
    digitalWrite(LmotorPinF, LOW); // stopped
    digitalWrite(LmotorPinR, LOW);
  }
  // right (starbord) motor
  if (joystick[1] > 0) {      // forward
    analogWrite(RmotorPinF, joystick[1]);
    digitalWrite(RmotorPinR, LOW);
  }
  else if (joystick[1] < 0) { //reverse
    digitalWrite(RmotorPinF, LOW);
    analogWrite(RmotorPinR, -joystick[1]);
  }
  else {
    digitalWrite(RmotorPinF, LOW); // stopped
    digitalWrite(RmotorPinR, LOW);
  }

  //R2D2 Head motor
  if (joystick[2] >= 0) {
    analogWrite(HmotorPinF, joystick[2]);
    digitalWrite(HmotorPinR, LOW);
  }
  else {
    digitalWrite(HmotorPinF, LOW);
    analogWrite(HmotorPinR, -joystick[2]);
  }
  /*
      // shoot disk motors
      if (R1Mask & payload.sreg) {
        digitalWrite(SpinMotor, HIGH);
        digitalWrite(ShootMotor, HIGH);
      }
      else {
        digitalWrite(SpinMotor, LOW);
        digitalWrite(ShootMotor, LOW);
      }

      // Hand LEDs
      if (R2Mask & payload.sreg) {
        if (lcount == 16) {
          count += 1;
          lcount = 0;
        } else {
          lcount += 1;
        }
        if (count > 3) {
          count = 1;
        }
        //  tone (SpeakerPin, LaserTone);
        //  LaserTone += 150;
        //  if (LaserTone > 2000) {
        //   LaserTone = 60;
        //}
        if (count == 1) {
          digitalWrite(HandLED1, LOW);
        }
        else {
          digitalWrite(HandLED1, HIGH);
        }
        if (count == 2) {
          digitalWrite(HandLED2, LOW);
        }
        else {
          digitalWrite(HandLED2, HIGH);
        }
        if (count == 3) {
          digitalWrite(HandLED3, LOW);
        }
        else {
          digitalWrite(HandLED3, HIGH);
        }
        delay (2);
        //  noTone (SpeakerPin);
      }
      else {
        digitalWrite(HandLED1, LOW);
        digitalWrite(HandLED2, LOW);
        digitalWrite(HandLED3, LOW);
        // LaserTone = 60;
      }
       \

      // Play sound tracks from the sound card





      /*
        soundfile = random(79);
        if (LDMask & payload.sreg) {
        wtv020sd16p.stopVoice();
        wtv020sd16p.asyncPlayVoice(soundfile);
        Serial.print("Now playing sound file #: ");
        Serial.println(soundfile);
        }
        else {
        if (B1Mask & payload.sreg) {
          tone (SpeakerPin, 523);
          delay (10);
          noTone (SpeakerPin);
        }
        else {
          noTone (SpeakerPin) ;
        }
        if (B2Mask & payload.sreg) {
          tone (SpeakerPin, 587);
          delay (10);
          noTone (SpeakerPin);
        }
        else {
          noTone (SpeakerPin) ;
        }
        if (B3Mask & payload.sreg) {
          tone (SpeakerPin, 659);
          delay (10);
          noTone (SpeakerPin);
        }
        else {
          noTone (SpeakerPin) ;
        }
        if (B4Mask & payload.sreg) {
          tone (SpeakerPin, 698);
          delay (10);
          noTone (SpeakerPin);
        }
        else {
          noTone (SpeakerPin) ;
        }
        noTone (SpeakerPin);
        }
  */
}
