/*  DEPLOY! project code
    by Rosellini Vittorio
    Code purpose: 
        Control a stepper motor to change the PHP configuration 
        Control a servo motor to move a black body used to calibrate the termal camera
        Read an accelerometer used as a backup for the operator commands
*/
#include    <Servo.h>

#define     ENDL    2       // Arduino connection
#define     ENDH    3
#define     DIR     4
#define     PUL     5       
#define     SZERO   10
#define     ENA     7
#define     PLUS45  11
#define     MIN45   10
#define     PLUS180 9
#define     MIN180  12
#define     SERVOLED A4
#define     SERVOPIN A0

#define INTERVAL 900        //Servo parameters
#define SERVOBTT 2
#define MAXSERVOANG 100
#define MINSERVOANG 10
#define SERVOSPEED 40

#define     NSTEP   400     // Stepper parameters
#define     TAU     4
#define     CW      HIGH
#define     CCW     LOW

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
char  carattere = '0';
bool   stato_led = 0;

int target = MINSERVOANG;
int next_target = MAXSERVOANG;
int last_position = 11;

Servo blackbody;

void setup(){
    Serial.begin(9600);

/*
---------------------------------------------------
        STEPPER
---------------------------------------------------
*/
    pinMode(PUL, OUTPUT);
    pinMode(DIR, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(PLUS45, INPUT);
    pinMode(MIN45, INPUT);
    pinMode(PLUS180, INPUT);
    pinMode(MIN180, INPUT);
    pinMode(SZERO,INPUT);
    pinMode(ENDL, INPUT_PULLUP);
    pinMode(ENDH, INPUT_PULLUP);
    pinMode(LED_BUILTIN, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(ENDL), end_low, RISING);
    attachInterrupt(digitalPinToInterrupt(ENDH), end_high, RISING);

    digitalWrite(PUL, HIGH);
    digitalWrite(DIR, HIGH);
    digitalWrite(ENA, HIGH);

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

  blackbody.attach(SERVOPIN);
  blackbody.write(MAXSERVOANG);

  pinMode(SERVOBTT, INPUT);
  
  pinMode(SERVOLED, OUTPUT);
  digitalWrite(SERVOLED, LOW);

    Serial.println("ARDUINO READY");

}

void loop(){

/*
---------------------------------------------------
        STEPPER
---------------------------------------------------
*/

    if(digitalRead(MIN45) != _min45 && buttonInterrupt == LOW){
        delay(300);
        Serial.println("Rotazione -45");
        rotate_angle(45, 10, CCW);
    }else buttonInterrupt = LOW;
    
    if(digitalRead(PLUS45) != _plus45 & buttonInterrupt == LOW){
        delay(300);
        Serial.println("Rotazione +45");
        rotate_angle(45, 10, CW);
    }else buttonInterrupt = LOW;
    
    if(digitalRead(MIN180) != _min180 && buttonInterrupt == LOW){
        delay(300);
        Serial.println("Rotazione -180");
        rotate_angle(180, 10, CCW);
    }else buttonInterrupt = LOW;

    if(digitalRead(PLUS180) != _plus180 && buttonInterrupt == LOW){
        delay(300);
        Serial.println("Rotazione +180");
        rotate_angle(180, 10, CW);
    }else buttonInterrupt = LOW;

    if(digitalRead(SZERO) != _setzero){
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
carattere =  Serial.read();
    
if(carattere == 'a' || digitalRead(SERVOBTT)) target = next_target;

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
}

void rotate_angle(float ang, int t, bool dir){
    // function that add or subtract the ang value to the relative position of the PHP
    // ang specify the value of the rotation, insert the PHP value the function evaluate with the gear ratio
    // t, variable that specify the angular velocity, small t => fast rotation
    // cw, use SOMETHING to increase the angle, use SOMETHING ELSE to reduce the angle

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
            Serial.println("MAX ANGLE REACHED");
            break;
        }
    }
    Serial.println("DONE");

}

void set_zero(){
    // function that find the zero position of the php
   
    detachInterrupt(digitalPinToInterrupt(ENDL));
    digitalWrite(DIR, CCW);

    while(!digitalRead(ENDL)){
        digitalWrite(PUL, LOW);
        delay(30);
        digitalWrite(PUL, HIGH);
        Serial.println("In cerca del finecorsa");
    }

    Serial.println("Finecorsa trovato");
    
    digitalWrite(DIR, CW);
    while(digitalRead(ENDL)){
        digitalWrite(PUL, LOW);
        delay(30);
        digitalWrite(PUL, HIGH);
        Serial.println("Cercando di lasciare il finecorsa");
    }
    
    digitalWrite(LED_BUILTIN, LOW);
    
    Serial.println("Finecorsa rilasciato");
    attachInterrupt(digitalPinToInterrupt(ENDL), end_low, FALLING);
    delay(100);

}

void end_low(){
    // function executed in case the limit switch send a signal, case 0 degrees 

    Serial.println("INTERRUPT");
    buttonInterrupt = HIGH;
    digitalWrite(DIR, CW);
    digitalWrite(PUL, HIGH);
    
    while(digitalRead(ENDL)){
        
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

void end_high(){
    // function executed in case the limit switch send a signal, case 180 degrees

    Serial.println("INTERRUPT");
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