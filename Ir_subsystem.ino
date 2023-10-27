/*  DEPLOY! project code

    More info about this project at https://deploy.unipi.it/

    by Rosellini Vittorio

    VittoRose (GitHub)

    Created: May 2023
    Updated: July 2023
*/

#include <Servo.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

#define HGRESISTOR 7  // IR tirgger
#define LGRESISTOR 8
#define SIZE 10
#define ACC_X2 mpu6050.getAccX() * mpu6050.getAccX()
#define ACC_Y2 mpu6050.getAccY() * mpu6050.getAccY()
#define ACC_Z2 mpu6050.getAccZ() * mpu6050.getAccZ()

/*
---------------------------------------------------------------------
            SERVO
---------------------------------------------------------------------
*/
uint8_t pos;
char txt_in = '0';

int target = MINSERVOANG;
int next_target = MAXSERVOANG;
int last_position = 11;

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

void setup(){
    Serial.begin(9600);

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

    Serial.println("------------------------------");
    Serial.println("\tARDUINO READY");
    Serial.println("------------------------------");
}

void loop(){
    txt_in = Serial.read();

    /*
    ---------------------------------------------------------------------
                            SERVO
    ---------------------------------------------------------------------
    */


    if (txt_in == 'e' || digitalRead(SERVOBTT)) target = next_target;

    if (target > last_position) {
        Serial.println("opening phase...");

        for (pos = last_position; pos <= target; pos++) {
        blackbody.write(pos);
        delay(SERVOSPEED);
        next_target = MINSERVOANG;
        digitalWrite(SERVOLED, HIGH);
        }

        Serial.println("opening phase COMPLETED");
        digitalWrite(SERVOLED, LOW);
    }

    if (target < last_position) {
        Serial.println("closing phase...");

        for (pos = last_position; pos >= target; pos--) {
        blackbody.write(pos);
        delay(SERVOSPEED);
        next_target = MAXSERVOANG;
        digitalWrite(SERVOLED, HIGH);
        }

        Serial.println("closing phase COMPLETED");
        digitalWrite(SERVOLED, LOW);
    }

    last_position = target;

    if (digitalRead(SERVOBTT)) {

        Serial.println("closing phase...");

        for (pos = last_position; pos >= MINSERVOANG; pos--) {
        blackbody.write(pos);
        delay(SERVOSPEED);
        next_target = MAXSERVOANG;
        digitalWrite(SERVOLED, HIGH);
        }
    }

    /*
    ---------------------------------------------------------------------
                            MPU6050
    ---------------------------------------------------------------------
    */

    mpu6050.update();

    fil[j] = ACC_X2 + ACC_Y2 + ACC_Z2;

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

float average(float measure[], int size) {
  int i = 0;
  float avg = 0;

  for (i = 0; i < size; i++) {
    avg += (float)fil[i];
  }

  return avg / (float)size;
}
