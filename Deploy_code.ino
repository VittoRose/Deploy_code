/*  DEPLOY! project code

    More info about this project at https://deploy.unipi.it/

    by Rosellini Vittorio

    VittoRose (GitHub)

    Created: May 2023
    Updated: July 2023

*/
#include    <Servo.h>

#define     ENDL    2      // Arduino connection
#define     ENDH    3

#define     PUL     9      // Stepper driver connection 
#define     DIR     10      
#define     ENA     11

#define     SZERO   23      // Buttons connection
#define     PLUS45  25
#define     MIN45   27
#define     PLUS180 29
#define     MIN180  31

#define     SERVOLED    13
#define     SERVOPIN    46
#define     HGRESISTOR  7
#define     LGRESISTOR  8

#define     INTERVAL    900        //Servo parameters
#define     SERVOBTT    33
#define     MAXSERVOANG 100
#define     MINSERVOANG 10
#define     SERVOSPEED  40
#define     RESDELAY    30000

#define     NSTEP   400     // Stepper parameters
#define     TAU     4
#define     CW      HIGH
#define     CCW     LOW

/*
    extra button provided in PCB
    #define     EXTRA1  31
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
uint8_t   pos;
char    txt_in = '0';

unsigned long hg_timer = 0;
unsigned long lg_timer = 0;

int target = MINSERVOANG;
int next_target = MAXSERVOANG;
int last_position = 11;

Servo blackbody;

void setup(){
    Serial.begin(115200);

/*
---------------------------------------------------
        STEPPER
---------------------------------------------------
*/
    pinMode(PUL, OUTPUT);           //Stepper driver signal pins
    pinMode(DIR, OUTPUT);
    pinMode(ENA, OUTPUT);

    pinMode(PLUS45, INPUT);         //Buttons
    pinMode(MIN45, INPUT);
    pinMode(PLUS180, INPUT);
    pinMode(MIN180, INPUT);
    pinMode(SZERO,INPUT);
    
    pinMode(ENDL, INPUT);           //Limit switches
    pinMode(ENDH, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENDL), end_low, RISING);          
    attachInterrupt(digitalPinToInterrupt(ENDH), end_high, RISING);

    digitalWrite(PUL, HIGH);               // start with high => everything on
    digitalWrite(DIR, HIGH);
    digitalWrite(ENA, HIGH);

    _plus45 = digitalRead(PLUS45);              //save the value of the buttons at the starting time to detect changes
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
  digitalWrite(LGRESISTOR, LOW);
  digitalWrite(HGRESISTOR, LOW);

  Serial.println("------------------------------");
  Serial.println("\tARDUINO READY");
  Serial.println("------------------------------");

}

void loop(){

/*
---------------------------------------------------
        STEPPER
---------------------------------------------------
*/

    txt_in = Serial.read();

    if((digitalRead(MIN45) != _min45 || txt_in == 'a') && buttonInterrupt == LOW){
        delay(300);
        Serial.println("Starting -45 degrees rotation");
        rotate_angle(45, 10, CCW);
    }else buttonInterrupt = LOW;
    
    if((digitalRead(PLUS45) != _plus45 || txt_in == 'b') && buttonInterrupt == LOW){
        delay(300);
        Serial.println("Starting +45 degrees rotation");
        rotate_angle(45, 10, CW);
    }else buttonInterrupt = LOW;


   if((digitalRead(MIN180) != _min180 || txt_in == 'c') && buttonInterrupt == LOW){
        delay(300);
        Serial.println("Starting -180 degrees rotation");
        rotate_angle(180, 10, CCW);
    }else buttonInterrupt = LOW;

    if((digitalRead(PLUS180) != _plus180 || txt_in == 'd') && buttonInterrupt == LOW){
        delay(300);
        Serial.println("Starting +180 degrees rotation");
        rotate_angle(180, 10, CW);
    }else buttonInterrupt = LOW;


    if(digitalRead(SZERO) != _setzero || txt_in == 'z'){
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

    
if(txt_in == 'a' || digitalRead(SERVOBTT)) target = next_target;

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

  if(digitalRead(SERVOBTT)){

    Serial.println("closing phase...");
    
    for (pos = last_position; pos >= MINSERVOANG; pos--) {
      blackbody.write(pos);
      delay(SERVOSPEED);
      next_target = MAXSERVOANG;
      digitalWrite(SERVOLED, HIGH);
    }
  }

    if(txt_in == 'h') {
        Serial.println("Heating the HG resistor");
        digitalWrite(HGRESISTOR, HIGH);
        delay(30*1000);
        digitalWrite(HGRESISTOR, LOW);
    }
    if(txt_in == 'l') {
        Serial.println("Heating the LG resistor");
        digitalWrite(LGRESISTOR, HIGH);
        delay(30*1000);
        digitalWrite(LGRESISTOR, LOW);
    }

    if(txt_in == 'h'){
        Serial.println("Heating the HG resistor");
        digitalWrite(HGRESISTOR, HIGH);
        hg_timer = millis();
    }    

    if(millis() - hg_timer >= RESDELAY) digitalWrite(HGRESISTOR, LOW);

    if(txt_in == 'l'){
        Serial.println("Heating the LG resistor");
        digitalWrite(LGRESISTOR, HIGH);
        lg_timer = millis();
    }    

    if(millis() - lg_timer >= RESDELAY) digitalWrite(LGRESISTOR, LOW);


    if(txt_in == 'r') {
        Serial.println("--------------------------------------");
        Serial.println("Reset in 3 seconds");
        Serial.println("--------------------------------------");
        delay(3000);
        asm volatile (" jmp 0 ");
    }
}

void rotate_angle(float ang, int t, bool dir){
    /* function that add or subtract the ang value to the relative position of the PHP
    ang specify the value of the rotation, insert the PHP value the function evaluate with the gear ratio
    t, variable that specify the angular velocity, small t => fast rotation
    cw, use CW to increase the angle, use CCW to reduce the angle */

    int i = 0;
    int step = 0;
    step = round(TAU*NSTEP*ang/360);
    Serial.println(step);

    if(dir == CW){
        digitalWrite(DIR, HIGH);
    }
    else digitalWrite(DIR, LOW);

    for(i = 0; i < step; i++){
        if(buttonInterrupt == LOW){
            digitalWrite(PUL, LOW);
            delay(t);
            digitalWrite(PUL, HIGH);
        }else {
            Serial.println(" END SWITCH SIGNAL");
            break;
        }
    }
    Serial.println("DONE");

}

void set_zero(){
    // function that find the zero position of the php
   
    detachInterrupt(digitalPinToInterrupt(ENDL));
    digitalWrite(DIR, CCW);

    Serial.println("PHP reset");

    while(!digitalRead(ENDL)){
        digitalWrite(PUL, LOW);
        delay(30);
        digitalWrite(PUL, HIGH);
    }

    Serial.println("Limit switch pressed");
    
    digitalWrite(DIR, CW);
    while(digitalRead(ENDL)){
        digitalWrite(PUL, LOW);
        delay(30);
        digitalWrite(PUL, HIGH);
    }

    Serial.println("Limit switch released");

    
    attachInterrupt(digitalPinToInterrupt(ENDL), end_low, FALLING);
    delay(100);

}

void end_low(){
    // function executed in case the limit switch send a signal, case 0 degrees 

    Serial.println("INTERRUPT LOW");
    buttonInterrupt = HIGH;
    digitalWrite(DIR, CW);
    digitalWrite(PUL, HIGH);
    
    while(digitalRead(ENDL)){
        
        counter1++;
        
        if(counter1 >= 200){
            digitalWrite(PUL, LOW);
            counter2++;
            if(counter2 >= 70){
                digitalWrite(PUL, HIGH);
                counter2 = 0;
            }
            counter1 = 0;
        }
    }
}

void end_high(){
    // function executed in case the limit switch send a signal, case 180 degrees
    
    Serial.println("INTERRUPT HIGH");
    buttonInterrupt = HIGH;
    
    digitalWrite(DIR, CCW);
    digitalWrite(PUL, HIGH);
    
    while(digitalRead(ENDH)){
        
        counter1++;
        
        if(counter1 >= 100){
            digitalWrite(PUL, LOW);
            counter2++;
            if(counter2 >= 70){
                digitalWrite(PUL, HIGH);
                counter2 = 0;
            }
            counter1 = 0;
        }
    }
}