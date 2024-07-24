#include <Arduino.h>
#include <NewPing.h>
#include <PWMServo.h>

#define US_SIDE_TRIG 2
#define US_SIDE_ECHO 3
#define RIGHT_F 4
#define RIGHT_R 5
#define RIGHT_PWM 6
#define LEFT_F 7
#define LEFT_R 8
#define SWEEPER 9
#define IR 10
#define US_FRONT_TRIG 11
#define US_FRONT_ECHO 12

#define ATTACK 0
#define CIRCLE 1
#define PUSH 2
#define HOLD 3
#define EXPLORE 4
#define NAIVE 5

// tuning
#define turn_speed 170        // wide turn speed for circling and exploring
#define sharp_speed 100 // sharper turn speed for pushing and circling
#define frontSensitivity 60   // max value for a significant detect
#define sideSensitivity 80    // max value for a significant detect while sweeping
#define max_distance 150      // how far we seeing
#define sweepLow 70           // start of sweep range
#define sweepGap 45           // range of sweep 
#define holdThreshold 9       // distance <= this value means wait for them to go over
#define curtainThreshold 20   // max distance the enemy can be ahead of us to still go through curtain


NewPing sonar(US_FRONT_TRIG, US_FRONT_ECHO, max_distance);
NewPing sweep_sonar(US_SIDE_TRIG, US_SIDE_ECHO, max_distance);

PWMServo sweeper;  // create servo object to control a servo

boolean canSeeEdge();
void straight(int rightSpeed);
void turn();
void stop();
void reverse();
void sweepUpdate();


uint16_t mode;      // state
int sweepPos;       // variable to store the servo position
bool sweepForward;  // sweep direction
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  sweepPos = sweepLow;
  sweepForward = true;
  mode = EXPLORE;
  sweeper.attach(SWEEPER);  // attaches the servo on pin 9 to the servo object
  // turn();
  // while (true);
  delay(5000);
}

int i = 0;
void loop() {
  sweeper.write(sweepPos);
  delay(50);
  int usVal = sonar.ping_cm();
  boolean edgeDetected = canSeeEdge();
  Serial.print("ON WHITE LINE? ");
  Serial.println(edgeDetected);
  if (usVal <= frontSensitivity) {
    Serial.print("FRONT DETECTED: ");
    Serial.println(usVal);
    if (usVal <= holdThreshold && mode != ATTACK && mode != PUSH) {
      mode = HOLD;
    } else if (edgeDetected) {
      if (mode == ATTACK && usVal <= 10) {
        mode = ATTACK;
      } else {
        mode = PUSH;
      }
    } else {
      mode = ATTACK;
    }
  } else if (mode != HOLD && mode != CIRCLE) {
    if (edgeDetected) {
      mode = CIRCLE;
    } else {
      mode = EXPLORE;
    }
  }
  usVal = sweep_sonar.ping_cm();
  Serial.print("SIDE VALUE: ");
  Serial.println(usVal);
  if (mode != ATTACK && usVal <= sideSensitivity) {
    double dist = usVal*cos(PI*(sweepPos - 90)/180);
    if (mode != EXPLORE && (sweepPos < 90)) {
      mode = CIRCLE;
    } else if (dist > curtainThreshold + 10) {
      mode = NAIVE;
    } else if (mode != EXPLORE && dist <= curtainThreshold) {
      mode = HOLD;
    } else if (mode != EXPLORE) {
      mode = CIRCLE;
    }
  }

  switch (mode) {
    case ATTACK:
      straight(255);
      break;
    case PUSH:
      straight(sharp_speed);
      break;
    case CIRCLE:
      if (edgeDetected) {
        straight(sharp_speed);
      } else {
        straight(turn_speed);
      }
      sweepUpdate();
      break;
    case EXPLORE:
      if (edgeDetected) {
        turn();
        delay(20);
        mode = CIRCLE;
      } else {
        straight(turn_speed);
      }
      break;
    case HOLD:
      stop();
      sweepUpdate();
      break;
    case NAIVE:
      turn();
      sweepUpdate();
      break;
  }
  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void reverse() {
  digitalWrite(RIGHT_F, HIGH);
  digitalWrite(RIGHT_R, LOW);
  digitalWrite(LEFT_F, LOW);
  digitalWrite(LEFT_R, HIGH);
  analogWrite(RIGHT_PWM, 255);
}
void turn() {
  digitalWrite(RIGHT_F, LOW);
  digitalWrite(RIGHT_R, HIGH);
  digitalWrite(LEFT_F, LOW);
  digitalWrite(LEFT_R, HIGH);
  
  analogWrite(RIGHT_PWM, 255);
}
void stop() {
  digitalWrite(RIGHT_F, LOW);
  digitalWrite(RIGHT_R, LOW);
  digitalWrite(LEFT_F, LOW);
  digitalWrite(LEFT_R, LOW);
}

void straight(int rightSpeed) {
  digitalWrite(RIGHT_F, LOW);
  digitalWrite(RIGHT_R, HIGH);
  digitalWrite(LEFT_F, HIGH);
  digitalWrite(LEFT_R, LOW);
  
  analogWrite(RIGHT_PWM, rightSpeed);
}

boolean canSeeEdge() {
	return (digitalRead(IR) == HIGH);
}

void sweepUpdate() {
  if (sweepPos >= sweepLow + sweepGap) {
    sweepPos = sweepLow + sweepGap;
    sweepForward = false;
  } else if (sweepPos <= sweepLow) {
    sweepForward = true;
  }

  if (sweepForward) {
    sweepPos += 2;
  } else {
    sweepPos -= 2;
  }
}