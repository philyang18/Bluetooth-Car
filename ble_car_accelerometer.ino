#include "Particle.h"
#include "argon_ble_setup.h"

// distance sensor variables
double SPEED_SOUND_CM_ROOM_TEMP_FAHR = 0.03444;
double CONV_FACTOR_CM_TO_IN = 0.3437;
double distanceCm;

// bluefruit app variables
const size_t UART_TX_BUF_SIZE = 20;
float X;
float Y;
float Z;

// motor variables
const int AIN1 = D6; // control pin 1 on the motor driver for the right motor
const int AIN2 = D7; // control pin 2 on the motor driver for the right motor
const int PWMA = D8; // speed control pin on the motor driver for the right motor

// the left motor will be controlled by the motor B pins on the motor driver
const int PWMB = D3; // speed control pin on the motor driver for the left motor
const int BIN2 = D4; // control pin 2 on the motor driver for the left motor
const int BIN1 = D5; // control pin 1 on the motor driver for the left motor

// Pins
const int PIN_TRIGGER = D2;
const int PIN_ECHO = D1;
const int PIN_PHOTORESISTOR = A0;
const int PIN_LIGHTS = A1;
const int PIN_BUTTON = A2;
const int PIN_SPEAKER = A3;

// variables that represent physical aspects of the car
int SPEED;
bool STOP = false;
int maxSpeed = 255;
bool lightsOn = true;
bool distSensorOn = true;
bool beeperOn = true;

// ACCELEROMETER FUNCTION THAT RECEIVES INFO FROM BLUEFRUIT APP
void onDataReceived(const uint8_t *data, size_t len, const BlePeerDevice &peer,void *context)
{
    X = *((float *)(data + 2));
    Y = *((float *)(data + 6));
    Z = *((float *)(data + 10));
    if (X > -0.1 && X < 0.1 && Y > -0.1 && Y < 0.1 && Z < -0.95) {
      SPEED = 0;
      carStop();
    } 
    else if (X > 0.05) {
      if(!STOP) {
        if(X > 0.8) {
          X = 0.8;
        }
        if (Y < 0.1 && Y > -0.1) {
            SPEED = abs(int(X * maxSpeed / 0.8));
            carForward(SPEED); 
        } else if (Y >= 0.1 && Y <= 1.0) {
            SPEED = abs(int(X * maxSpeed / 0.8));
            forwardLeft(SPEED - int(Y * SPEED));
        } else if (Y <= -0.1 && Y >= -1.0) {
            SPEED = abs(int(X * maxSpeed / 0.8));
            forwardRight(SPEED + int(Y * SPEED));
        } else {
            SPEED = abs(int(X * maxSpeed / 0.8));
            carForward(SPEED); 
        }
      } else {
        carStop();
      }
    } 
    else if (X < -0.5) {
      if(X < -0.8) {
        X = -0.8;
      }
      if (Y < 0.1 && Y > -0.1) {
        SPEED = abs(int(X * maxSpeed / 0.8));
        carBackward(SPEED);
      } else if (Y >= 0.1 && Y <= 1.0) {
        SPEED = abs(int(X * maxSpeed / 0.8));
        backwardLeft(SPEED - int(Y * SPEED));
      } else if (Y <= -0.1 && Y >= -1.0) {
        SPEED = abs(int(X * maxSpeed / 0.8));
        backwardRight(SPEED + int(Y * SPEED));
      } else {
        SPEED = abs(int(X * maxSpeed / 0.8));
        carBackward(SPEED);
      }
    } 
    else {
        SPEED = abs(int(X * maxSpeed / 0.8));
        carForward(SPEED);
    }
    publishEvents();
}

void setup()
{
  Serial.begin(9600);
  argon_ble_setup();
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(PIN_TRIGGER, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_PHOTORESISTOR, INPUT);
  pinMode(PIN_LIGHTS, OUTPUT);
  pinMode(PIN_BUTTON, INPUT);
  pinMode(PIN_SPEAKER, OUTPUT);
  Particle.function("updateMaxSpeed", updateMaxSpeed);
  Particle.function("toggleDistSensor", toggleDistSensor);
}

void loop()
{
  if(distSensorOn) {
    checkBeeperButton();
    checkDistanceSensor();
  } else { 
    STOP = false;
    distanceCm = 0; 
    checkBeeperButton();
  }
  checkBeeperButton();
  checkLights();
  
}

// FUNCTIONS FOR PHYSICAL COMPONENTS
void checkBeeperButton() {
  Serial.println(digitalRead(PIN_BUTTON));
  if (digitalRead(PIN_BUTTON) == 0) {
    beeperOn = !beeperOn;
  } 
}
void checkLights(){
  // AUTOMATIC LIGHTS
  if(analogRead(PIN_PHOTORESISTOR) < 2800 ) {
    analogWrite(PIN_LIGHTS, 0);
  } else if(analogRead(PIN_PHOTORESISTOR) >= 2800 && analogRead(PIN_PHOTORESISTOR) < 3300 ){
    analogWrite(PIN_LIGHTS, 100);
  } else {
    analogWrite(PIN_LIGHTS, 255);
  }
}
void checkDistanceSensor() {
  digitalWrite(PIN_TRIGGER, LOW); // prepare
  delayMicroseconds(2);
  digitalWrite(PIN_TRIGGER, HIGH); // prepare
  delayMicroseconds(10);
  digitalWrite(PIN_TRIGGER, LOW); // prepare
  int timeRoundTrip = pulseIn(PIN_ECHO, HIGH); // wait for round trip time
  distanceCm = timeRoundTrip * SPEED_SOUND_CM_ROOM_TEMP_FAHR / 2;

  // BEEPING NOISES 
  if (beeperOn) {
    if (distanceCm < 5.0) {
      tone(PIN_SPEAKER, 4000, 1500);
    } else if (distanceCm < 10.0) {
      tone(PIN_SPEAKER, 2000, 1000);
    } else if (distanceCm < 20.0) {
      tone(PIN_SPEAKER, 1000, 500);
    } else {
      noTone(PIN_SPEAKER);
    }
  } else {
    noTone(PIN_SPEAKER);
  }
  if ( (SPEED < 75 && distanceCm <= 5.0) || (SPEED >= 75 && SPEED < 125 && distanceCm <= 15.0) || (SPEED >= 125 && SPEED < 185 && distanceCm <= 25.0) || (SPEED >= 185 && distanceCm <= 30.0) ) {
    carStop();
    STOP = true;
    delay(1500);
    STOP = false;
  } else {
    STOP = false;
  }
}

// FUNCTIONS CALLED TO AND FROM THE WEB
void publishEvents() {
  Particle.publish("reading", String(SPEED) + ":" + String(maxSpeed) + ":" + String(beeperOn) + ":" + String(distanceCm));
}
// Called on by the web interface app- not called in this code
int updateMaxSpeed(String speed) {
  int max = speed.toInt();
  if (max >= 0 && max <=255){
    maxSpeed = max;
    return 1;
  } else {
    return 0;
  }
  Serial.println(maxSpeed);
}
// Called on by the web interface app- not called in this code
int toggleDistSensor(String state) {
  if(state == "true") {
    distSensorOn = true;
  } else {
    distSensorOn = false;
  }
  return 1;
}


// MOTOR CONTROL FUNCTIONS
void leftMotor(int motorSpeed) {
  if (motorSpeed > 0)
  {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }
  analogWrite(PWMB, abs(motorSpeed));
}
void rightMotor(int motorSpeed){
  if (motorSpeed > 0)
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
  analogWrite(PWMA, abs(motorSpeed));
}
void carForward(int motorSpeed) {
  rightMotor(motorSpeed);
  leftMotor(motorSpeed);
}
void carBackward(int motorSpeed) {
  rightMotor(-1 * motorSpeed);
  leftMotor(-1 * motorSpeed);
}
void carStop() {
  rightMotor(0);
  leftMotor(0);
}
void forwardLeft(int turningSpeed){
  leftMotor(turningSpeed);
}
void forwardRight(int turningSpeed) {
  rightMotor(turningSpeed);
}
void backwardLeft(int turningSpeed) {
  leftMotor(-1 * turningSpeed);
}
void backwardRight(int turningSpeed) {
  rightMotor(-1 * turningSpeed);
}

