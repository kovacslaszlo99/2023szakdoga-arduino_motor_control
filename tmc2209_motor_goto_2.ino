#include <TMCStepper.h>
#include <math.h>
#include <Arduino.h>
#include <TimerOne.h>

#define RA_MANUAL_UP_PIN 14
#define RA_MANUAL_DOW_PIN 15
#define DEC_MANUAL_UP_PIN 16
#define DEC_MANUAL_DOW_PIN 17
#define MANUAL_SPEED_1_PIN 18
#define MANUAL_SPEED_2_PIN 19

#define RA_MODE_BEGIN 0
#define RA_MODE_ACCELERATION 1
#define RA_MODE_RUN 2
#define RA_MODE_DECELERATION 3
#define RA_MODE_DONE 4
#define RA_MODE_CLOCK_MACHINE 5

#define DEC_MODE_BEGIN 0
#define DEC_MODE_ACCELERATION 1
#define DEC_MODE_RUN 2
#define DEC_MODE_DECELERATION 3
#define DEC_MODE_DONE 4

#define SERIAL_PORT_RA Serial1
#define SERIAL_PORT_DEC Serial2
#define DRIVER_ADDRESS 0b00
#define R_SENSE 0.11f
#define SIDEREAL_STEP_TO_STEP_DELAY 5178.125
#define RMS_CURRENT_GOTO 1500
#define RMS_CURRENT_TRACKING 1000
#define SOFTWARE_VERSION "1.2"

#define RA_STEP_PIN 3
#define RA_DIR_PIN 2
#define RA_EN_PIN 4

#define DEC_STEP_PIN 9
#define DEC_DIR_PIN 10
#define DEC_EN_PIN 11

TMC2209Stepper driver_ra(&SERIAL_PORT_RA, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper driver_dec(&SERIAL_PORT_DEC, R_SENSE, DRIVER_ADDRESS);

double micro = 0;
bool is_move = false;
int global_min_delay = 5;
bool tracking = false;
double RA_tracking_last = 0;
int sidereal_tracking_quatro = 1;
int rms_current_goto_var = RMS_CURRENT_GOTO;
int rms_current_tracking_var = RMS_CURRENT_TRACKING;
bool tracking_dir = true;
double trackingCounter = 0;
unsigned long debuging_tracking_timer = 0;
bool tracking_time = false;
double tracking_time_s = 0;

int RA_Mode = 0;
double RA_Acceleration = 0;
double RA_i = 0;
double RA_last = 0;
double  RA_Run_Step = 0;
int RA_Step = 0;
int RA_acceleration_array[3200];
int RA_MinDelay = 6;
int RA_MinDelay_man = 2000;
int RA_acceleration_step_man = 500;
double ra_step_abs;
double ra_step_counter = 0;
bool ra_man = false;
unsigned long last_ra_man_button_pressed = 0;


int DEC_Mode = 0;
double DEC_Acceleration = 0;
double DEC_i = 0;
double DEC_last = 0;
double  DEC_Run_Step = 0;
int DEC_Step = 0;
int DEC_acceleration_array[3200];
int DEC_MinDelay = 6;
int DEC_MinDelay_man = 2000;
int DEC_acceleration_step_man = 500;
double dec_step_abs;
double dec_step_counter = 0;
bool dec_man = false;
unsigned long last_dec_man_button_pressed = 0;

int RA_MANUAL_UP_BUTTON_value = 1;
int RA_MANUAL_DOW_BUTTON_value = 1;
int DEC_MANUAL_UP_BUTTON_value = 1;
int DEC_MANUAL_DOW_BUTTON_value = 1;
int MANUAL_SPEED_1_value = 1;
int MANUAL_SPEED_2_value = 1;

#define step(STEP_PIN) digitalWrite(STEP_PIN, !digitalRead(STEP_PIN))
#define are_we_There_Yet(last) get_micros() >= last
#define get_micros() micro
#define oneStepRA(delay) if(are_we_There_Yet(RA_last + delay)){ step(RA_STEP_PIN); RA_last = get_micros(); RA_Step++; ra_step_counter++; }
#define oneStepDEC(delay) if(are_we_There_Yet(DEC_last + delay)){ step(DEC_STEP_PIN); DEC_last = get_micros(); DEC_Step++; dec_step_counter++; }
#define microsAdd() micro++
#define customDelay()  delayMicroseconds(1); microsAdd();
#define driverEnable(EN_PIN, mod) digitalWrite(EN_PIN, mod?LOW:HIGH)
#define driverDIR(DIR_PIN, mod) digitalWrite(DIR_PIN, mod?LOW:HIGH)
#define RA_DriverEnable(mod) driverEnable(RA_EN_PIN, mod)
#define DEC_DriverEnable(mod) driverEnable(DEC_EN_PIN, mod)
#define RA_DriverDIR(mod) driverDIR(RA_DIR_PIN, mod)
#define DEC_DriverDIR(mod) driverDIR(DEC_DIR_PIN, mod)

void setup() {
  Serial.begin(9600);

  Serial5.begin(115200);

  SERIAL_PORT_RA.begin(115200);
  driver_ra.begin();
  driver_ra.microsteps(128);
  driver_ra.toff(3);
  driver_ra.rms_current(rms_current_goto_var);
  driver_ra.pwm_autoscale(true);
  //driver_ra.en_spreadCycle(true);
  driver_ra.TCOOLTHRS(0xFFFFF);

  SERIAL_PORT_DEC.begin(115200);
  driver_dec.begin();
  driver_dec.microsteps(128);
  driver_dec.toff(3);
  driver_dec.rms_current(rms_current_goto_var);
  driver_dec.pwm_autoscale(true);
  //driver_dec.en_spreadCycle(true);
  driver_dec.TCOOLTHRS(0xFFFFF);

  pinMode(RA_STEP_PIN, OUTPUT);
  pinMode(RA_DIR_PIN, OUTPUT);
  pinMode(RA_EN_PIN, OUTPUT);

  pinMode(DEC_STEP_PIN, OUTPUT);
  pinMode(DEC_DIR_PIN, OUTPUT);
  pinMode(DEC_EN_PIN, OUTPUT);

  pinMode(RA_MANUAL_UP_PIN, INPUT_PULLUP);
  pinMode(RA_MANUAL_DOW_PIN, INPUT_PULLUP);
  pinMode(DEC_MANUAL_UP_PIN, INPUT_PULLUP);
  pinMode(DEC_MANUAL_DOW_PIN, INPUT_PULLUP);
  pinMode(MANUAL_SPEED_1_PIN, INPUT_PULLUP);
  pinMode(MANUAL_SPEED_2_PIN, INPUT_PULLUP);

  RA_DriverEnable(false);
  DEC_DriverEnable(false);

  MANUAL_SPEED_1_value = digitalRead(MANUAL_SPEED_1_PIN);
  MANUAL_SPEED_2_value = digitalRead(MANUAL_SPEED_2_PIN);

  Timer1.initialize(SIDEREAL_STEP_TO_STEP_DELAY);
  Timer1.attachInterrupt(tracking_fun);

  Serial.println("----------GOTO 2------------");
  setSpeed(MANUAL_SPEED_1_value, MANUAL_SPEED_2_value);
}

void tracking_fun(void){
  if(tracking && RA_Mode == RA_MODE_CLOCK_MACHINE){
      step(RA_STEP_PIN);
      trackingCounter++;
      if(sidereal_tracking_quatro == 8){
        sidereal_tracking_quatro = 1;
        Timer1.setPeriod(SIDEREAL_STEP_TO_STEP_DELAY+1);
      }else{
        Timer1.setPeriod(SIDEREAL_STEP_TO_STEP_DELAY);
      }
      sidereal_tracking_quatro++;
    }
}

void setSpeed(int bitOne, int bitTwo){
  int sum_bits = bitOne + (bitTwo*2);
  switch(sum_bits){
    case 0:
      RA_MinDelay_man = 2000;
      DEC_MinDelay_man = 2000;
      DEC_acceleration_step_man = 500;
      RA_acceleration_step_man = 500;
      break;
    case 1:
      RA_MinDelay_man = 1000;
      DEC_MinDelay_man = 1000;
      DEC_acceleration_step_man = 500;
      RA_acceleration_step_man = 500;
      break;
    case 2:
      RA_MinDelay_man = 200;
      DEC_MinDelay_man = 200;
      DEC_acceleration_step_man = 500;
      RA_acceleration_step_man = 500;
      break;
    case 3:
      RA_MinDelay_man = 10;
      DEC_MinDelay_man = 10;
      DEC_acceleration_step_man = 2000;
      RA_acceleration_step_man = 2000;
      break;
  }
}

void move(){
  if(RA_Mode == RA_MODE_BEGIN){
    RA_DriverEnable(true);
    RA_Mode = RA_MODE_ACCELERATION;
  }else if(RA_Mode == RA_MODE_ACCELERATION){
    if(RA_i <= 0){
      RA_Mode = RA_MODE_RUN;
      RA_i = 0;
      RA_Step = 0;
    }else if(RA_Step >= 2){
      RA_i--;
      RA_Step = 0;
    }else{
      oneStepRA(RA_acceleration_array[int(RA_i)]);
    }
  }else if(RA_Mode == RA_MODE_RUN){
    if(RA_i > RA_Run_Step){
      RA_Mode = RA_MODE_DECELERATION;
      RA_i = 0;
      RA_Step = 0;
    }else if(RA_Step >= 2){
      RA_i++;
      RA_Step = 0;
    }else{
      oneStepRA(RA_MinDelay);
    }
  }else if(RA_Mode == RA_MODE_DECELERATION){
    if(RA_i >= RA_Acceleration){
      RA_Mode = RA_MODE_DONE;
      RA_i = 0;
      RA_Step = 0;
    }else if(RA_Step >= 2){
      RA_i++;
      RA_Step = 0;
    }else{
      oneStepRA(RA_acceleration_array[int(RA_i)]);
    }
  }else if(RA_Mode == RA_MODE_DONE){
    if(!tracking){
      RA_DriverEnable(false);
    }else{
      RA_Mode == RA_MODE_CLOCK_MACHINE;
    }
  }

  if(DEC_Mode == DEC_MODE_BEGIN){
    DEC_DriverEnable(true);
    DEC_Mode = DEC_MODE_ACCELERATION;
  }else if(DEC_Mode == DEC_MODE_ACCELERATION){
    if(DEC_i <= 0){
      DEC_Mode = DEC_MODE_RUN;
      DEC_i = 0;
      DEC_Step = 0;
    }else if(DEC_Step >= 2){
      DEC_i--;
      DEC_Step = 0;
    }else{
      oneStepDEC(DEC_acceleration_array[int(DEC_i)]);
    }
  }else if(DEC_Mode == DEC_MODE_RUN){
    if(DEC_i > DEC_Run_Step){
      DEC_Mode = DEC_MODE_DECELERATION;
      DEC_i = 0;
      DEC_Step = 0;
    }else if(DEC_Step >= 2){
      DEC_i++;
      DEC_Step = 0;
    }else{
      oneStepDEC(DEC_MinDelay);
    }
  }else if(DEC_Mode == DEC_MODE_DECELERATION){
    if(DEC_i >= DEC_Acceleration){
      DEC_Mode = DEC_MODE_DONE;
      DEC_i = 0;
      DEC_Step = 0;
    }else if(DEC_Step >= 2){
      DEC_i++;
      DEC_Step = 0;
    }else{
      oneStepDEC(DEC_acceleration_array[int(DEC_i)]);
    }
  }else if(DEC_Mode == DEC_MODE_DONE){
    DEC_DriverEnable(false);
  }

  customDelay();
}

void calculal(double ra_step, double dec_step){
  RA_DriverDIR(ra_step >= 0);
  DEC_DriverDIR(dec_step >= 0);
  
  if(tracking){
    RA_Mode = RA_MODE_BEGIN;
    driver_ra.rms_current(rms_current_goto_var);
  }
  
  if(ra_step == 0){
    RA_Mode = RA_MODE_DONE;
  }

  if(dec_step == 0){
    DEC_Mode = DEC_MODE_DONE;
  }

  ra_step_abs = abs(ra_step);
  dec_step_abs = abs(dec_step);

  RA_Acceleration = (ra_step_abs < 6400)? int(ra_step_abs/2) : 3200;
  DEC_Acceleration = (dec_step_abs < 6400)? int(dec_step_abs/2) : 3200;

  RA_MinDelay = (ra_step_abs >= dec_step_abs)? global_min_delay : (int((dec_step_abs/ra_step_abs)*global_min_delay));
  DEC_MinDelay = (dec_step_abs >= ra_step_abs)? global_min_delay : (int((ra_step_abs/dec_step_abs)*global_min_delay));

  RA_i = RA_Acceleration-1;
  DEC_i = DEC_Acceleration-1;
  RA_Step = 0;
  DEC_Step = 0;
  ra_step_counter = 0;
  dec_step_counter = 0;


  RA_Run_Step = ra_step_abs - (RA_Acceleration*2);
  DEC_Run_Step = dec_step_abs - (DEC_Acceleration*2);

  RA_last = get_micros();
  DEC_last = get_micros();
  for(double i = 0; i < RA_Acceleration; i++){
    double currentDelay = RA_MinDelay + (pow(i, 4) / 100000000000);
    RA_acceleration_array[int(i)] = int(currentDelay);
  }
  for(double i = 0; i < DEC_Acceleration; i++){
    double currentDelay = DEC_MinDelay + (pow(i, 4) / 100000000000);
    DEC_acceleration_array[int(i)] = int(currentDelay);
  }
}

void ra_man_calc(){
  if(tracking){
    RA_Mode = RA_MODE_BEGIN;
    driver_ra.rms_current(rms_current_goto_var);
  }
  for(double i = 0; i < RA_acceleration_step_man; i++){
    double currentDelay = RA_MinDelay_man + (pow(i, 4) / 100000000000);
    RA_acceleration_array[int(i)] = int(currentDelay);
  }
  RA_i = RA_acceleration_step_man-1;
  RA_Step = 0;
  ra_step_counter = 0;
  RA_last = get_micros();
}

void dec_man_calc(){
  for(double i = 0; i < DEC_acceleration_step_man; i++){
    double currentDelay = DEC_MinDelay_man + (pow(i, 4) / 100000000000);
    DEC_acceleration_array[int(i)] = int(currentDelay);
  }
  DEC_i = DEC_acceleration_step_man-1;
  DEC_Step = 0;
  dec_step_counter = 0;
  DEC_last = get_micros();
}

void dec_man_move(){
  if(DEC_Mode == DEC_MODE_BEGIN){
    DEC_Mode = DEC_MODE_ACCELERATION;
    DEC_DriverEnable(true);
  }
  if(DEC_Mode == DEC_MODE_ACCELERATION){
    if(!dec_man){
      DEC_Mode = DEC_MODE_DECELERATION;
      DEC_Step = 0;
    }else if(DEC_i <= 0){
      DEC_Mode = DEC_MODE_RUN;
      DEC_i = 0;
      DEC_Step = 0;
    }else if(DEC_Step >= 2){
      DEC_i--;
      DEC_Step = 0;
    }else{
      oneStepDEC(DEC_acceleration_array[int(DEC_i)]);
    }
  }else if(DEC_Mode == DEC_MODE_RUN){
    if(!dec_man){
      DEC_Mode = DEC_MODE_DECELERATION;
      DEC_i = 0;
      DEC_Step = 0;
    }else if(DEC_Step >= 2){
      DEC_i++;
      DEC_Step = 0;
    }else{
      oneStepDEC(DEC_MinDelay_man);
    }
  }else if(DEC_Mode == DEC_MODE_DECELERATION){
   if(DEC_i >= DEC_acceleration_step_man){
      DEC_Mode = DEC_MODE_DONE;
      DEC_i = 0;
      DEC_Step = 0;
    }else if(DEC_Step >= 2){
      DEC_i++;
      DEC_Step = 0;
    }else{
      oneStepDEC(DEC_acceleration_array[int(DEC_i)]);
    }
  }else if(DEC_Mode == DEC_MODE_DONE){
    DEC_DriverEnable(false);
    DEC_Mode = DEC_MODE_BEGIN;
    Serial.print("manualstepdec;");
    Serial.print((digitalRead(DEC_DIR_PIN) == LOW?"":"-"));
    Serial.println(dec_step_counter/2);
  }
}

void ra_man_move(){
  if(RA_Mode == RA_MODE_BEGIN){
    RA_Mode = RA_MODE_ACCELERATION;
    RA_DriverEnable(true);
  }
  if(RA_Mode == RA_MODE_ACCELERATION){
    if(!ra_man){
      RA_Mode = RA_MODE_DECELERATION;
      RA_Step = 0;
    }else if(RA_i <= 0){
      RA_Mode = RA_MODE_RUN;
      RA_i = 0;
      RA_Step = 0;
    }else if(RA_Step >= 2){
      RA_i--;
      RA_Step = 0;
    }else{
      oneStepRA(RA_acceleration_array[int(RA_i)]);
    }
  }else if(RA_Mode == RA_MODE_RUN){
    if(!ra_man){
      RA_Mode = RA_MODE_DECELERATION;
      RA_i = 0;
      RA_Step = 0;
    }else if(RA_Step >= 2){
      RA_i++;
      RA_Step = 0;
    }else{
      oneStepRA(RA_MinDelay_man);
    }
  }else if(RA_Mode == RA_MODE_DECELERATION){
   if(RA_i >= RA_acceleration_step_man){
      RA_Mode = RA_MODE_DONE;
      RA_i = 0;
      RA_Step = 0;
    }else if(RA_Step >= 2){
      RA_i++;
      RA_Step = 0;
    }else{
      oneStepRA(RA_acceleration_array[int(RA_i)]);
    }
  }else if(RA_Mode == RA_MODE_DONE){
    if(tracking){
      RA_Mode = RA_MODE_CLOCK_MACHINE;
      driver_ra.rms_current(rms_current_tracking_var);
      RA_DriverDIR(tracking_dir);
    }else{
      RA_DriverEnable(false);
      RA_Mode = RA_MODE_BEGIN;
    }
    Serial.print("manualstepra;");
    Serial.print((digitalRead(RA_DIR_PIN) == LOW?"":"-"));
    Serial.println(ra_step_counter/2);
  }
}

void loop() {
  if (Serial5.available()) {
    String receivedData = Serial5.readStringUntil('\n');
    receivedData.trim();
    if (receivedData.length() > 0) {
      String tag = getValue(receivedData,';',0);
      if(tag == "move"){
        String move_ra_dec = getValue(receivedData,';',1);
        char *eptr;
        String ra = getValue(move_ra_dec,',',0);
        int ra_len = ra.length() + 1;
        char ra_char_array[ra_len];
        ra.toCharArray(ra_char_array, ra_len);

        String dec = getValue(move_ra_dec,',',1);
        int dec_len = dec.length() + 1;
        char dec_char_array[dec_len];
        dec.toCharArray(dec_char_array, dec_len);

        double ra_move_step = strtod(ra_char_array, &eptr);
        double dec_move_step = strtod(dec_char_array, &eptr);
        calculal(ra_move_step, dec_move_step);
        Serial.print(ra_move_step);
        Serial.print(" ");
        Serial.println(dec_move_step);
        is_move = true;
      }else if(tag == "tracking"){
        String tracking_str = getValue(receivedData,';',1);
        if(tracking_str == "on"){
          tracking = true;
          RA_Mode = RA_MODE_CLOCK_MACHINE;
          RA_DriverEnable(true);
          driver_ra.rms_current(rms_current_tracking_var);
          RA_DriverDIR(tracking_dir);
          Serial.println("Tracking ON");
          trackingCounter = 0;
        }else if(tracking_str == "off"){
          tracking = false;
          RA_Mode = RA_MODE_BEGIN;
          RA_DriverEnable(false);
          Serial.print("Tracking OFF;");
          Serial.println(trackingCounter);
        }
      }else if(tag == "trackingtime"){
        String tracking_str = getValue(receivedData,';',1);
        if(tracking_str == "on"){
          String tracking_time_str = getValue(receivedData,';',2);
          char *eptr;
          int tracking_time_str_len = tracking_time_str.length() + 1;
          char tracking_time_str_char_array[tracking_time_str_len];
          tracking_time_str.toCharArray(tracking_time_str_char_array, tracking_time_str_len);

          tracking_time_s = strtod(tracking_time_str_char_array, &eptr);
          tracking_time = true;
          tracking = true;
          RA_Mode = RA_MODE_CLOCK_MACHINE;
          debuging_tracking_timer = millis();
          RA_DriverEnable(true);
          driver_ra.rms_current(rms_current_tracking_var);
          RA_DriverDIR(tracking_dir);
          Serial.println("Tracking Time ON");
          trackingCounter = 0;
        }
      }else if(tag == "set"){
        String set_tag = getValue(receivedData,';',1);
        if(set_tag == "currentgoto"){
          String set_current_goto = getValue(receivedData,';',2);
          char buf[set_current_goto.length()+1];
          set_current_goto.toCharArray(buf, set_current_goto.length()+1);
          rms_current_goto_var = atoi(buf);
          Serial.println(rms_current_goto_var);
        }else if(set_tag == "currenttracking"){
          String set_current_tracking = getValue(receivedData,';',2);
          char buf[set_current_tracking.length()+1];
          set_current_tracking.toCharArray(buf, set_current_tracking.length()+1);
          rms_current_tracking_var = atoi(buf);
          Serial.println(rms_current_tracking_var);
        }else if(set_tag == "trackingdir"){
          String set_tracking_dir = getValue(receivedData,';',2);
          tracking_dir = (set_tracking_dir == "true") ? true : false;
          Serial.println(tracking_dir);
        }
      }else if(tag == "ping"){
        Serial5.println("pong");
        Serial.println("pong");
      }else if(tag == "version"){
        Serial5.print("Version: ");
        Serial5.println(SOFTWARE_VERSION);
      }
    }
  }

  if(tracking_time){
    if(debuging_tracking_timer + tracking_time_s*1000 <= millis()){
      tracking = false;
      RA_Mode = RA_MODE_BEGIN;
      RA_DriverEnable(false);
      Serial.print("Tracking OFF;");
      Serial.println(trackingCounter);
      tracking_time = false;
    }
  }

  if(is_move){
    double last = millis();
    while((RA_Mode != RA_MODE_DONE && RA_Mode != RA_MODE_CLOCK_MACHINE) || DEC_Mode != DEC_MODE_DONE){
      move();
    }
    double now = millis();
    double time = (now - last)*0.001;
    Serial.print(ra_step_counter/2);
    Serial.print(" ");
    Serial.print(dec_step_counter/2);
    Serial.print(" time(s): ");
    Serial.println(time);
    is_move = false;
    if(tracking){
      RA_Mode = RA_MODE_CLOCK_MACHINE;
      driver_ra.rms_current(rms_current_tracking_var);
      RA_DriverDIR(tracking_dir);
      RA_DriverEnable(true);
    }else{
      RA_Mode = RA_MODE_BEGIN;
    }
    DEC_Mode = DEC_MODE_BEGIN;
    DEC_DriverEnable(false);
    if(!tracking){
      RA_DriverEnable(false);
    }
  }

  if(digitalRead(MANUAL_SPEED_1_PIN) != MANUAL_SPEED_1_value || digitalRead(MANUAL_SPEED_2_PIN) != MANUAL_SPEED_2_value){
    MANUAL_SPEED_1_value = digitalRead(MANUAL_SPEED_1_PIN);
    MANUAL_SPEED_2_value = digitalRead(MANUAL_SPEED_2_PIN);
    setSpeed(MANUAL_SPEED_1_value, MANUAL_SPEED_2_value);
  }

  if(digitalRead(RA_MANUAL_UP_PIN) != RA_MANUAL_UP_BUTTON_value && millis() - last_ra_man_button_pressed > 50){
    RA_MANUAL_UP_BUTTON_value = digitalRead(RA_MANUAL_UP_PIN);
    if(RA_MANUAL_UP_BUTTON_value == 0){
      if(RA_Mode != RA_MODE_ACCELERATION && RA_Mode != RA_MODE_RUN && RA_Mode != RA_MODE_DECELERATION){
        Serial.println("RA Manuális mozgatás fel ON");
        ra_man = true;
        RA_DriverDIR(true);
        ra_man_calc();
      }
    }else{
      Serial.println("RA Manuális mozgatás fel OFF");
      ra_man = false;
      last_ra_man_button_pressed = millis();
    }
  }

  if(digitalRead(RA_MANUAL_DOW_PIN) != RA_MANUAL_DOW_BUTTON_value && millis() - last_ra_man_button_pressed > 50){
    RA_MANUAL_DOW_BUTTON_value = digitalRead(RA_MANUAL_DOW_PIN);
    if(RA_MANUAL_DOW_BUTTON_value == 0){
      if(RA_Mode != RA_MODE_ACCELERATION && RA_Mode != RA_MODE_RUN && RA_Mode != RA_MODE_DECELERATION){
        Serial.println("RA Manuális mozgatás le ON");
        ra_man = true;
        RA_DriverDIR(false);
        ra_man_calc();
      }
    }else{
      Serial.println("RA Manuális mozgatás le OFF");
      ra_man = false;
      last_ra_man_button_pressed = millis();
    }
  }

  if(digitalRead(DEC_MANUAL_UP_PIN) != DEC_MANUAL_UP_BUTTON_value && millis() - last_dec_man_button_pressed > 50){
    DEC_MANUAL_UP_BUTTON_value = digitalRead(DEC_MANUAL_UP_PIN);
    if(DEC_MANUAL_UP_BUTTON_value == 0){
      if(DEC_Mode != DEC_MODE_ACCELERATION && DEC_Mode != DEC_MODE_RUN && DEC_Mode != DEC_MODE_DECELERATION){
        Serial.println("DEC Manuális mozgatás fel ON");
        dec_man = true;
        DEC_DriverDIR(true);
        dec_man_calc();
        }
    }else{
      Serial.println("DEC Manuális mozgatás fel OFF");
      dec_man = false;
      last_dec_man_button_pressed = millis();
    }
  }

  if(digitalRead(DEC_MANUAL_DOW_PIN) != DEC_MANUAL_DOW_BUTTON_value && millis() - last_dec_man_button_pressed > 50){
    DEC_MANUAL_DOW_BUTTON_value = digitalRead(DEC_MANUAL_DOW_PIN);
    if(DEC_MANUAL_DOW_BUTTON_value == 0){
      if(DEC_Mode != DEC_MODE_ACCELERATION && DEC_Mode != DEC_MODE_RUN && DEC_Mode != DEC_MODE_DECELERATION){
        Serial.println("DEC Manuális mozgatás le ON");
        DEC_DriverDIR(false);
        dec_man = true;
        dec_man_calc();
      }
    }else{
      Serial.println("DEC Manuális mozgatás le OFF");
      dec_man = false;
      last_dec_man_button_pressed = millis();
    }
  }

  if((RA_Mode != RA_MODE_CLOCK_MACHINE && RA_Mode != RA_MODE_BEGIN )|| ra_man){
    ra_man_move();
  }

  if(DEC_Mode != DEC_MODE_BEGIN || dec_man){
    dec_man_move();
  }
  
  customDelay();
}

String getValue(String data, char separator, int index){
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;
  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }
  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

