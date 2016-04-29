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

#include <RF24.h>
#include <JQ6500_Serial.h>

#include "printf.h"

JQ6500_Serial mp3(31, 30);

int joystick[3];

// masks for each buttons in s_register
const uint16_t BUTTON_LEFT                  = 0x0001;
const uint16_t BUTTON_RIGHT                 = 0x0002;
const uint16_t BUTTON_DOWN                  = 0x0004;
const uint16_t BUTTON_UP                    = 0x0008;
const uint16_t BUTTON_SHOULDER_RIGHT_TOP    = 0x0010;
const uint16_t BUTTON_SHOULDER_RIGHT_BOTTOM = 0x0020;
const uint16_t BUTTON_ANALOG_RIGHT          = 0x0040;
// Unused                                     0x0080
const uint16_t BUTTON_1                     = 0x0100;
const uint16_t BUTTON_2                     = 0x0200;
const uint16_t BUTTON_3                     = 0x0400;
const uint16_t BUTTON_4                     = 0x0800;
const uint16_t BUTTON_SHOULDER_LEFT_TOP     = 0x1000;
const uint16_t BUTTON_SHOULDER_LEFT_BOTTOM  = 0x2000;
const uint16_t BUTTON_ANALOG_LEFT           = 0x4000;
// Unused                                     0x8000
const uint16_t VALID_SREGISTER_MASK         = ~0x8080;

uint16_t previousSreg = 0x0000;

// Structure to hold the payload received over the radio from the
// remote control.
struct RemoteControlPayload {
  uint16_t sreg;
  int16_t j_RUD;
  int16_t j_RLR;
  int16_t j_LUD;
  int16_t j_LLR;
};

static_assert(sizeof(RemoteControlPayload) == 10,
              "Payload size must match remote control. Anything else can cause problems.");

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
  { -255, -255, -255, -255, -200, -150,   0 },
  { -255, -200, -200, -200, -150,    0, 150 },
  { -255, -200, -150, -150,    0,  150, 200 },
  { -255, -200, -150,    0,  150,  200, 255 },
  { -200, -150,    0,  150,  150,  200, 255 },
  { -100,    0,  100,  200,  200,  200, 255 },
  {    0,  150,  200,  255,  255,  255, 255 }
};
const int RmotorPower [7][7] = {
  {   0, -150, -200, -255, -255, -255, -255 },
  { 150,    0, -150, -200, -200, -200, -255 },
  { 200,  150,    0, -150, -150, -200, -255 },
  { 255,  200,  150,    0, -150, -200, -255 },
  { 255,  200,  150,  150,    0, -150, -200 },
  { 255,  200,  200,  200,  150,    0, -150 },
  { 255,  255,  255,  255,  200,  150,    0 }
};

int16_t joystickRange(int16_t value) {
  return min(-255, max(255, value));
}

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

  // start the radio
  radio.begin();
  radio.setChannel(0x4F); // change from default setting (0x4C) to avoid interference
  radio.setDataRate(RF24_250KBPS);  // slow data rate for better reliability
  radio.setPayloadSize(sizeof(RemoteControlPayload));
  radio.openReadingPipe(1, pipe);
  radio.startListening();
  radio.printDetails();

  mp3.playFileByIndexNumber(9);
}

void loop(void) {
  int x, y;
  RemoteControlPayload payload = {0, 0, 0, 0, 0};

  if (radio.available()) {
    radio.read(&payload, sizeof(RemoteControlPayload));

    // Defensively exclude unused bits from sregister.
    payload.sreg &= VALID_SREGISTER_MASK;

    // Check that joystick values are in the expected range
    payload.j_RLR = joystickRange(payload.j_RLR);
    payload.j_RUD = joystickRange(payload.j_RUD);
    payload.j_LLR = joystickRange(payload.j_LLR);
    payload.j_LUD = joystickRange(payload.j_LUD);
  }

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

  uint16_t buttons = payload.sreg & ~previousSreg;
  previousSreg = payload.sreg;

  if (buttons & BUTTON_1) {
    mp3.playFileByIndexNumber(1);
  }

  if (buttons & BUTTON_2) {
    mp3.playFileByIndexNumber(5);
  }

  if (buttons & BUTTON_3) {
    mp3.playFileByIndexNumber(6);
  }

  if (buttons & BUTTON_4) {
    mp3.playFileByIndexNumber(7);
  }

  if (buttons & BUTTON_UP) {
    mp3.playFileByIndexNumber(2);
  }

  if (buttons & BUTTON_LEFT) {
    mp3.playFileByIndexNumber(10);
  }

  if (buttons & BUTTON_DOWN) {
    mp3.playFileByIndexNumber(11);
  }

  if (buttons & BUTTON_RIGHT) {
    mp3.playFileByIndexNumber(12);
  }

  if (buttons & BUTTON_SHOULDER_RIGHT_TOP) {
    mp3.playFileByIndexNumber(9);
  }

  if (buttons & BUTTON_SHOULDER_RIGHT_BOTTOM) {
    mp3.playFileByIndexNumber(3);
  }

  if (buttons & BUTTON_SHOULDER_LEFT_TOP) {
    mp3.playFileByIndexNumber(8);
  }

  if (buttons & BUTTON_SHOULDER_LEFT_BOTTOM) {
    mp3.playFileByIndexNumber(4);
  }

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
}
