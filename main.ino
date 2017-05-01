
#include <Wire.h>
#include "ds3231.h"
#include "rtc_ds3231.h"

#define BUFF_MAX 128
#define UP 4
#define GOINGUP 3
#define BRAKING 2
#define MOVING 7
#define GOINGDOWN 1
#define DOWN 0
#define u 1
#define d 0
/*
TODO

Was wenn ausgeschalten wird
dann ist state = nicht bekannt und wird eventuell wiederholt

alarm setzen auf letzes up oderd own??

*/

//motorboard
const int
PWM_A   = 3,
DIR_A   = 12,
BRAKE_A = 9,
SNS_A   = A0;

//button
const int buttonPin = 2;     // the number of the pushbutton pin
int buttonState = 0;  
boolean manualMode = false;  

//
int direction;

int STATE = DOWN;
uint8_t time[8];
char recv[BUFF_MAX];
unsigned int recv_size = 0;
unsigned long prev, interval = 5000;
boolean actState;

//the time settings for up and down
int goUpHour = 8;
int goUpMin  = 00;
int goDownHour = 17;
int goDownMin  = 02;

//the calculated up and down times f.e. 887 ist z.B. 887 = 14(h) * 60 + 47(min) = 14:47
int goUpTime;
int goDownTime;
int nowTime; 
int actDOW = 8; //so the first time it will be changed

//runtimes of motor
//6 seconds is quite good
unsigned long motorStartTime;
int motorMaxRuntime = 6000;
unsigned long brakeStart;


//enable/disable printing
boolean debug = true;

void setup()
{
  
  if (debug) Serial.begin(9600);
  //clock
  Wire.begin();
  DS3231_init(DS3231_INTCN);
  memset(recv, 0, BUFF_MAX);
  //motor
  pinMode(BRAKE_A, OUTPUT);  // Brake pin on channel A
  pinMode(DIR_A, OUTPUT);    // Direction pin on channel A
  //button
  pinMode(buttonPin, INPUT);

  //calc the up and down time preset
  goUpTime = goUpHour * 60 + goUpMin;
  goDownTime = goDownHour * 60 + goDownMin;

//assume we are always down on start so last direction was d
  direction = d;

}

void loop()
{
  
  //first all time related stuff // show time once in a while
  char in;
  char buff[BUFF_MAX];
  unsigned long now = millis();
  struct ts t;
  //5000 millis over get new time and trigger actions
  if ((now - prev > interval) && (Serial.available() <= 0)) {
    DS3231_get(&t);

    // there is a compile time option in the library to include unixtime support
#ifdef CONFIG_UNIXTIME
    snprintf(buff, BUFF_MAX, "%d.%02d.%02d %02d:%02d:%02d %ld", t.year,
             t.mon, t.mday, t.hour, t.min, t.sec, t.unixtime);
#else
    snprintf(buff, BUFF_MAX, "%d.%02d.%02d %02d:%02d:%02d", t.year,
             t.mon, t.mday, t.hour, t.min, t.sec);
#endif

    //saving the actual time 
    prev = now;
    nowTime = t.hour * 60 + t.min;
    if (actDOW != t.wday){
      actDOW = t.wday;
      calcUpAndDownTime(actDOW);
    }
    //Printing all infos
    printUpDownTime();
    printNowTime(t.hour,t.min,nowTime,actDOW);
    printState(STATE);

    //see if we have to take actions
    if  (nowTime >= goUpTime && nowTime < goDownTime && STATE != MOVING && STATE != BRAKING) {
      //time where door should be up
      if (manualMode){
          if (STATE == UP){
            //is already up doing nothing but switching off manual mode
            manualMode = false;
            direction = u;
          } else { //STATE ist DOWN
            //es ist unten weil wir overrided haben also alles gut
          }
        
        } else {
          if (STATE != UP){
                direction = u;
                motorStart();
            }
        }
      
    } else if ((nowTime < goUpTime || nowTime >= goDownTime) && STATE != MOVING && STATE != BRAKING) {
      //time where door should be down
      if (manualMode){
        if (STATE == DOWN){
          //is already down
          manualMode = false;
          direction = d;
        } else {
          //state is UP we wanted it that way
        }
      } else {
        if (STATE != DOWN){
           direction = d;
           motorStart();
        }
      }
      //going down if
     
    }
  }//end of the every 5 secons loop

  //button is pressed
  buttonState = digitalRead(buttonPin);
  //Serial.println(buttonState);
  if (buttonState == HIGH){
    //override only if not moving
    if (STATE != MOVING && STATE != BRAKING){
      manualMode = true;
      if (direction  == u){ 
        direction = d;
      } else if (direction == d){
        direction = u;
      }
      digitalWrite(DIR_A, direction);   // setting direction to HIGH the motor will spin forward
      motorStart();
    }  
  }

//checking if we have to break
  if (STATE == MOVING) {
    int delta = now - motorStartTime;
    //Serial.println(delta);
    if (delta > motorMaxRuntime) {
      motorBrake();
    }
  } else if (STATE == BRAKING) {
    int delta = now - brakeStart;
    if (delta > 1000){
      motorStop();
    }  
  }

/*

  if (Serial.available() > 0) {
    in = Serial.read();
Serial.println(in);
    if ((in == 10 || in == 13) && (recv_size > 0)) {
      Serial.println("parse");
      parse_cmd(recv, recv_size);
      recv_size = 0;
      recv[0] = 0;
    } else if (in < 48 || in > 122) {
      ;       // ignore ~[0-9A-Za-z]
    } else if (recv_size > BUFF_MAX - 2) {   // drop lines that are too long
      // drop
      recv_size = 0;
      recv[0] = 0;
    } else if (recv_size < BUFF_MAX - 2) {
      recv[recv_size] = in;
      recv[recv_size + 1] = 0;
      recv_size += 1;
    }

  }

*/
  delay(200);

}

void motorStart(){
  STATE = MOVING;
  printState(STATE);
  printDirection();
  //Serial.println();
  digitalWrite(BRAKE_A, LOW);  // setting brake LOW disable motor brake
  digitalWrite(DIR_A, direction);   // setting direction to HIGH the motor will spin forward

  analogWrite(PWM_A, 255);     // Set the speed of the motor, 255 is the maximum value
  motorStartTime = millis();
}

void motorBrake(){
  STATE = BRAKING;
  printState(STATE);
  digitalWrite(BRAKE_A, HIGH);  // raise the brake
  analogWrite(PWM_A, 0);       // turn off power to the motor#
  brakeStart = millis();
}

void motorStop(){
  //release the brake
  if (direction == u) {
    STATE = UP;
  } else {
    STATE = DOWN;
  }
  printState(STATE);
  digitalWrite(BRAKE_A, LOW);
  digitalWrite(DIR_A, 0);//test if led B goes off
}

void calcUpAndDownTime(int dow){
  if (debug) Serial.print("changing times :");
  if (debug) Serial.println(dow);
  if (dow % 7== 0 || dow % 6 == 0){
    goUpHour = 7;
    goUpMin  = 00;
    goDownHour = 20;
    goDownMin  = 00;
    //sun or sat
  } else {
    //weekdays
    goUpHour = 7;
    goUpMin  = 00;
    goDownHour = 20;
    goDownMin  = 00;
  }
  goUpTime = goUpHour * 60 + goUpMin;
  goDownTime = goDownHour * 60 + goDownMin;
}


void printDirection(){
  if (debug){
    if (direction == u){
      Serial.println("UP >>>>>>>");
    } else {
        Serial.println("DOWN <<<<<<<<");
    }
  }
}

void printUpDownTime(){
  if (debug){
    Serial.println("=========================");
    Serial.print("goUpTime   : ");
    if (goUpHour < 10) {Serial.print("0");}
    Serial.print(goUpHour);
    Serial.print(":");
    if (goUpMin < 10) {Serial.print("0");}
    Serial.print(goUpMin);
    Serial.print(" (");
    Serial.print(goUpTime);
    Serial.println(")");
    Serial.print("goDownTime : ");
    if (goDownHour < 10) {Serial.print("0");}
    Serial.print(goDownHour);
    Serial.print(":");
    if (goDownMin < 10) {Serial.print("0");}
    Serial.print(goDownMin);
    Serial.print(" (");
    Serial.print(goDownTime);
    Serial.println(")");
  }
}

void printState(int s){
  if (debug){
    Serial.print("State      : ");
    if      (s == 4){Serial.print("UP");}
    else if (s == 3){Serial.print("GOINGUP");}
    else if (s == 2){Serial.print("BRAKING");}
    else if (s == 7){Serial.print("MOVING");}
    else if (s == 1){Serial.print("GOINGDOWN");}
    else if (s == 0){Serial.print("DOWN");}
    else {Serial.print("unknown");}
    if (manualMode){
        Serial.println(" (OVERRIDE)");
    } else {
      Serial.println("");
    }
  }
}

void printNowTime(int h, int m, int n,int dow){
  if (debug){
    Serial.print("now        : ");
    if (h < 10) {Serial.print("0");}
    Serial.print(h);
    Serial.print(":");
    if (m < 10) {Serial.print("0");}
    Serial.print(m);
    Serial.print(" (");
    Serial.print(n);
    Serial.println(")");
    Serial.print("dow        : ");
    Serial.println(dow);
  }
}


void parse_cmd(char *cmd, int cmdsize)
{
  uint8_t i;
  uint8_t reg_val;
  char buff[BUFF_MAX];
  struct ts t;

  snprintf(buff, BUFF_MAX, "cmd was '%s' %d\n", cmd, cmdsize);
  Serial.print(buff);

  // TssmmhhWDDMMYYYY aka set time
  if (cmd[0] == 84 && cmdsize == 16) {
    //T355720619112011
    t.sec = inp2toi(cmd, 1);
    t.min = inp2toi(cmd, 3);
    t.hour = inp2toi(cmd, 5);
    t.wday = cmd[7] - 48;
    t.mday = inp2toi(cmd, 8);
    t.mon = inp2toi(cmd, 10);
    t.year = inp2toi(cmd, 12) * 100 + inp2toi(cmd, 14);
    DS3231_set(t);
    Serial.println("OK");
  } else if (cmd[0] == 49 && cmdsize == 1) {  // "1" get alarm 1
    DS3231_get_a1(&buff[0], 59);
    Serial.println(buff);
  } else if (cmd[0] == 50 && cmdsize == 1) {  // "2" get alarm 1
    DS3231_get_a2(&buff[0], 59);
    Serial.println(buff);
  } else if (cmd[0] == 51 && cmdsize == 1) {  // "3" get aging register
    Serial.print("aging reg is ");
    Serial.println(DS3231_get_aging(), DEC);
  } else if (cmd[0] == 65 && cmdsize == 9) {  // "A" set alarm 1
    DS3231_set_creg(DS3231_INTCN | DS3231_A1IE);
    //ASSMMHHDD
    for (i = 0; i < 4; i++) {
      time[i] = (cmd[2 * i + 1] - 48) * 10 + cmd[2 * i + 2] - 48; // ss, mm, hh, dd
    }
    uint8_t flags[5] = { 0, 0, 0, 0, 0 };
    DS3231_set_a1(time[0], time[1], time[2], time[3], flags);
    DS3231_get_a1(&buff[0], 59);
    Serial.println(buff);
  } else if (cmd[0] == 66 && cmdsize == 7) {  // "B" Set Alarm 2
    DS3231_set_creg(DS3231_INTCN | DS3231_A2IE);
    //BMMHHDD
    for (i = 0; i < 4; i++) {
      time[i] = (cmd[2 * i + 1] - 48) * 10 + cmd[2 * i + 2] - 48; // mm, hh, dd
    }
    uint8_t flags[5] = { 0, 0, 0, 0 };
    DS3231_set_a2(time[0], time[1], time[2], flags);
    DS3231_get_a2(&buff[0], 59);
    Serial.println(buff);
  } else if (cmd[0] == 67 && cmdsize == 1) {  // "C" - get temperature register
    Serial.print("temperature reg is ");
    Serial.println(DS3231_get_treg(), DEC);
  } else if (cmd[0] == 68 && cmdsize == 1) {  // "D" - reset status register alarm flags
    reg_val = DS3231_get_sreg();
    reg_val &= B11111100;
    DS3231_set_sreg(reg_val);
  } else if (cmd[0] == 70 && cmdsize == 1) {  // "F" - custom fct
    reg_val = DS3231_get_addr(0x5);
    Serial.print("orig ");
    Serial.print(reg_val, DEC);
    Serial.print("month is ");
    Serial.println(bcdtodec(reg_val & 0x1F), DEC);
  } else if (cmd[0] == 71 && cmdsize == 1) {  // "G" - set aging status register
    DS3231_set_aging(0);
  } else if (cmd[0] == 83 && cmdsize == 1) {  // "S" - get status register
    Serial.print("status reg is ");
    Serial.println(DS3231_get_sreg(), DEC);
  } else {
    Serial.print("unknown command prefix ");
    Serial.println(cmd[0]);
    Serial.println(cmd[0], DEC);
  }
}
