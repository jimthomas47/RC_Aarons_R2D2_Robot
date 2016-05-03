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
#include <math.h>

#include "printf.h"

JQ6500_Serial mp3(31, 30);

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
class Motor {
private:
  const int _forwardPin;
  const int _reversePin;

public:
  Motor(const int forwardPin, const int reversePin)
    : _forwardPin(forwardPin), _reversePin(reversePin) {
  }

  void setup() {
    pinMode(_forwardPin, OUTPUT);
    digitalWrite(_forwardPin, LOW);

    pinMode(_reversePin, OUTPUT);
    digitalWrite(_reversePin, LOW);
  }

  void setPower(const int16_t value) {
    if (value > 0) {
      analogWrite(_forwardPin, value);
      digitalWrite(_reversePin, LOW);
    } else {
      digitalWrite(_forwardPin, LOW);
      analogWrite(_reversePin, -value);
    }
  }
};

Motor rightLeg(3, 2);
Motor leftLeg(4, 5);
Motor head(10, 11);

const int16_t MOTOR_MAX_POWER = 255;
const float POWER_PER_RADIAN = 324.675324675;

int16_t joystickRange(int16_t value) {
  return min(-MOTOR_MAX_POWER, max(MOTOR_MAX_POWER, value));
}

void setup(void) {
  Serial.begin(9600);
  printf_begin();

  // Motors
  leftLeg.setup();
  rightLeg.setup();
  head.setup();

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

  updateLegMotors(payload.j_RLR, payload.j_RUD);
  head.setPower(payload.j_LLR / 2);

  if (previousSreg != payload.sreg) {
    updateButtons(payload.sreg & ~previousSreg);
    previousSreg = payload.sreg;
  }
}

void updateButtons(uint16_t buttons) {
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
}

void updateLegMotors(int16_t horizontal, int16_t vertical) {
  float power1 = min(hypot(horizontal, vertical), MOTOR_MAX_POWER);
  int16_t left = power1;
  int16_t right = power1;

  if (horizontal == 0) {
    if (vertical < 0) {
      left = -power1;
      right = -power1;
    }
  } else {
    float power2 = (atan(abs((float) vertical / (float) horizontal)) * POWER_PER_RADIAN - MOTOR_MAX_POWER) * (power1 / MOTOR_MAX_POWER);

    if (horizontal >= 0) {
      if (vertical >= 0) {
        left = power1;
        right = power2;
      } else {
        left = -power2;
        right = -power1;
      }
    } else {
      if (vertical >= 0) {
        left = power2;
        right = power1;
      } else {
        left = -power1;
        right = -power2;
      }
    }
  }

  rightLeg.setPower(right);
  leftLeg.setPower(left);
}
