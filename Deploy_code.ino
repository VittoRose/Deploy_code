/*  DEPLOY! project code

    More info about this project at https://deploy.unipi.it/

    by Rosellini Vittorio

    VittoRose (GitHub)

    Created: May 2023
    Updated: October 2023

*/

#include <Servo.h>
#include <MPU6050_tockn.h>
#include <Wire.h>


#define PUL 9  // Stepper driver connection
#define DIR 10
#define ENA 11

#define SZERO 23  // Buttons connection
#define PLUS45 25
#define MIN45 27
#define PLUS180 29
#define MIN180 31
#define ENDL 2      // Limit switches
#define ENDH 3

#define HGRESISTOR 7  // IR tirgger
#define LGRESISTOR 8
#define SIZE 10
#define ACC_X2 mpu6050.getAccX() * mpu6050.getAccX()
#define ACC_Y2 mpu6050.getAccY() * mpu6050.getAccY()
#define ACC_Z2 mpu6050.getAccZ() * mpu6050.getAccZ()

#define SERVOBTT 34            //Servo parameters
#define MAXSERVOANG 100
#define MINSERVOANG 10
#define SERVOSPEED 40
#define RESDELAY 300
#define SERVOLED 13
#define SERVOPIN 46

#define NSTEP 400  // Stepper parameters
#define TAU 4
#define CW HIGH
#define CCW LOW
#define INTERRUPT_COND RISING

/*
    extra button provided in PCB
    #define     EXTRA2  32
    #define     EXTRA3  33

*/
/*
---------------------------------------------------
        STEPPER
---------------------------------------------------
*/

bool ena = HIGH;
bool brk = HIGH;

bool _plus45 = LOW;
bool _min45 = LOW;
bool _plus180 = LOW;
bool _min180 = LOW;
bool _setzero = LOW;

volatile int counter1 = 0;
volatile int counter2 = 0;
volatile bool buttonInterrupt = LOW;

/*
---------------------------------------------------------------------
            SERVO
---------------------------------------------------------------------
*/

char txt_in = '0';
bool _servobtt = LOW;
uint8_t servo_ang = MAXSERVOANG;
Servo blackbody;

/*
-------------------------------------------------------------------
                    IR CAM
-------------------------------------------------------------------
*/

MPU6050 mpu6050(Wire);

unsigned long hg_timer = 0;
unsigned long lg_timer = 0;

int j = 0;
float avg = 0.0;
float fil[SIZE];


void setup() {
  Serial.begin(115200);

  /*
---------------------------------------------------
        STEPPER
---------------------------------------------------
*/
  pinMode(PUL, OUTPUT);  //Stepper driver signal pins
  pinMode(DIR, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(PLUS45, INPUT);  //Buttons
  pinMode(MIN45, INPUT);
  pinMode(PLUS180, INPUT);
  pinMode(MIN180, INPUT);
  pinMode(SZERO, INPUT);

  pinMode(ENDL, INPUT);  //Limit switches
  pinMode(ENDH, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENDL), end_low, INTERRUPT_COND);
  attachInterrupt(digitalPinToInterrupt(ENDH), end_high, INTERRUPT_COND);

  digitalWrite(PUL, HIGH);  // start with high => everything on
  digitalWrite(DIR, HIGH);
  digitalWrite(ENA, HIGH);

  _plus45 = digitalRead(PLUS45);  //save the value of the buttons at the starting time to detect changes
  _min45 = digitalRead(MIN45);
  _plus180 = digitalRead(PLUS180);
  _min180 = digitalRead(MIN180);
  _setzero = digitalRead(SZERO);

  /*
---------------------------------------------------------------------
            SERVO
---------------------------------------------------------------------
*/

  blackbody.attach(SERVOPIN);
  blackbody.write(MAXSERVOANG);

  pinMode(SERVOBTT, INPUT);

  pinMode(SERVOLED, OUTPUT);
  pinMode(LGRESISTOR, OUTPUT);
  pinMode(HGRESISTOR, OUTPUT);

  digitalWrite(SERVOLED, LOW);
  _servobtt = digitalRead(SERVOBTT);


  /*
  -----------------------------------------------------------------------
                        IR TRIGGER
  ----------------------------------------------------------------------
  */

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  digitalWrite(LGRESISTOR, LOW);
  digitalWrite(HGRESISTOR, LOW);

  Serial.println("\n------------------------------");
  Serial.println("\tARDUINO READY");
  Serial.println("------------------------------");
}

void loop() {

  /*
---------------------------------------------------
        STEPPER
---------------------------------------------------
*/

  txt_in = Serial.read();

  if ((digitalRead(MIN45) != _min45 || txt_in == 'a') && buttonInterrupt == LOW) {
    delay(300);
    Serial.println("Starting -45 degrees rotation");
    rotate_angle(45, 10, CCW);
  } else buttonInterrupt = LOW;

  if ((digitalRead(PLUS45) != _plus45 || txt_in == 'b') && buttonInterrupt == LOW) {
    delay(300);
    Serial.println("Starting +45 degrees rotation");
    rotate_angle(45, 10, CW);
  } else buttonInterrupt = LOW;


  if ((digitalRead(MIN180) != _min180 || txt_in == 'c') && buttonInterrupt == LOW) {
    delay(300);
    Serial.println("Starting -180 degrees rotation");
    rotate_angle(180, 10, CCW);
  } else buttonInterrupt = LOW;

  if ((digitalRead(PLUS180) != _plus180 || txt_in == 'd') && buttonInterrupt == LOW) {
    delay(300);
    Serial.println("Starting +180 degrees rotation");
    rotate_angle(180, 10, CW);
  } else buttonInterrupt = LOW;


  if (digitalRead(SZERO) != _setzero || txt_in == 'z') {
    delay(300);
    set_zero();
  }

  _plus45 = digitalRead(PLUS45);
  _min45 = digitalRead(MIN45);
  _plus180 = digitalRead(PLUS180);
  _min180 = digitalRead(MIN180);
  _setzero = digitalRead(SZERO);

  /*
---------------------------------------------------------------------
                        SERVO
---------------------------------------------------------------------
*/

  if (txt_in == 'e' || digitalRead(SERVOBTT) != _servobtt) {

    if (servo_ang == MAXSERVOANG) {
      Serial.println("--------------------------");
      Serial.println("Closing Servo");

      while (servo_ang > MINSERVOANG) {
        blackbody.write(servo_ang);
        delay(SERVOSPEED);
        servo_ang--;
      }

      Serial.println("Servo closed");
      Serial.println("--------------------------");

    } else if (servo_ang == MINSERVOANG) {
      Serial.println("--------------------------");
      Serial.println("Opening Servo");

      while (servo_ang < MAXSERVOANG) {
        blackbody.write(servo_ang);
        delay(SERVOSPEED);
        servo_ang++;
      }
      Serial.println("Servo open");
      Serial.println("--------------------------");
    }
  }

  _servobtt = digitalRead(SERVOBTT);

  /*
---------------------------------------------------------------------
                        MPU6050
---------------------------------------------------------------------
*/

  mpu6050.update();

  fil[j] = sqrt(ACC_X2 + ACC_Y2 + ACC_Z2);

  if (j < (SIZE - 1)) j++;
  else j = 0;

  avg = average(fil, SIZE);

  if (txt_in == 'h' || avg >= 1.6) {
    Serial.println("Heating the HG resistor");
    digitalWrite(HGRESISTOR, HIGH);
    hg_timer = millis();
  }

  if (txt_in == 'l' || avg <= 0.4) {
    Serial.println("Heating the LG resistor");
    digitalWrite(LGRESISTOR, HIGH);
    lg_timer = millis();
  }

  if (millis() - hg_timer >= RESDELAY) digitalWrite(HGRESISTOR, LOW);
  if (millis() - lg_timer >= RESDELAY) digitalWrite(LGRESISTOR, LOW);

  // RESET
  if (txt_in == 'r') {
    Serial.println("--------------------------------------");
    Serial.println("Reset in 3 seconds");
    Serial.println("--------------------------------------");
    delay(3000);
    asm volatile(" jmp 0 ");
  }
}

void rotate_angle(float ang, int t, bool dir) {
  /* function that add or subtract the ang value to the relative position of the PHP
    */

  uint8_t i = 0;
  uint8_t step = 0;
  step = round(TAU * NSTEP * ang / 360);
  Serial.println(step);

  if (dir == CW) {
    digitalWrite(DIR, HIGH);
  } else digitalWrite(DIR, LOW);

  for (i = 0; i < step; i++) {
    if (buttonInterrupt == LOW) {
      digitalWrite(PUL, LOW);
      delay(t);
      digitalWrite(PUL, HIGH);
    } else {
      Serial.println(" END SWITCH SIGNAL");
      break;
    }
  }
  Serial.println("DONE");
}

void set_zero() {
  // function that find the zero position of the php

  detachInterrupt(digitalPinToInterrupt(ENDL));
  digitalWrite(DIR, CCW);

  Serial.println("PHP reset");

  while (!digitalRead(ENDL)) {
    digitalWrite(PUL, LOW);
    delay(30);
    digitalWrite(PUL, HIGH);
  }

  Serial.println("Limit switch pressed");

  digitalWrite(DIR, CW);
  while (digitalRead(ENDL)) {
    digitalWrite(PUL, LOW);
    delay(30);
    digitalWrite(PUL, HIGH);
  }

  Serial.println("Limit switch released");


  attachInterrupt(digitalPinToInterrupt(ENDL), end_low, INTERRUPT_COND);
  delay(100);
}

void end_low() {
  // function executed in case the limit switch send a signal, case 0 degrees

  Serial.println("INTERRUPT LOW");
  detachInterrupt(digitalPinToInterrupt(ENDH));
  buttonInterrupt = HIGH;
  digitalWrite(DIR, CW);
  digitalWrite(PUL, HIGH);

  while (digitalRead(ENDL)) {

    counter1++;

    if (counter1 >= 200) {            // using counter as a delay
      digitalWrite(PUL, LOW);
      counter2++;
      if (counter2 >= 70) {
        digitalWrite(PUL, HIGH);
        counter2 = 0;
      }
      counter1 = 0;
    }

    if (digitalRead(ENDH)){
      digitalWrite(PUL, HIGH);
      Serial.println("-------------------------------");
      Serial.print("ERROR\t"); Serial.println("Both limit switches pressed");
      Serial.println("-------------------------------");
    }
  }
  attachInterrupt(digitalPinToInterrupt(ENDH), end_high, INTERRUPT_COND);
}

void end_high() {
  // function executed in case the limit switch send a signal, case 180 degrees

  Serial.println("INTERRUPT HIGH");
  detachInterrupt(digitalPinToInterrupt(ENDL));

  buttonInterrupt = HIGH;

  digitalWrite(DIR, CCW);
  digitalWrite(PUL, HIGH);

  while (digitalRead(ENDH)) {

    counter1++;

    if (counter1 >= 100) {
      digitalWrite(PUL, LOW);
      counter2++;
      if (counter2 >= 70) {
        digitalWrite(PUL, HIGH);
        counter2 = 0;
      }
      counter1 = 0;
    }
    
    if (digitalRead(ENDH)){
      digitalWrite(PUL, HIGH);
      Serial.println("-------------------------------");
      Serial.print("ERROR\t"); Serial.println("Both limit switches pressed");
      Serial.println("-------------------------------");
    }
  }
   attachInterrupt(digitalPinToInterrupt(ENDH), end_high, INTERRUPT_COND);

}

float average(float measure[], int size) {
  int i = 0;
  float avg = 0;

  for (i = 0; i < size; i++) {
    avg += (float)fil[i];
  }

  return avg / (float)size;
}