//PCB version
#include <Keypad.h>
#include <SPI.h>
//#include <Adafruit_GFX.h>
//#include <ArducamSSD1306.h>
#include <splash.h>
#include <Wire.h>  // For I2C comm, but needed for not getting compile error
#include <math.h>
#include <PID_v1.h>
#include "max6675.h"
#include <SD.h>
#include <TimeLib.h>
#include <DS1307RTC.h>


/*
  HardWare I2C pins
  A4   SDA
  A5   SCL
*/

tmElements_t tm;

//byte rowPins[ROWS] = {36, 39, 34, 35}; // Pins for rows
//byte colPins[COLS] = {32, 33, 25}; // Pins for columns

//byte rowPins[ROWS] = {22, 24, 26, 28}; // Pins for rows Connection on PCB
//byte colPins[COLS] = {30, 32, 34}; // Pins for columns

//Potentiometers setup
int raw2 = 0;         // lid analog read
int Vin = 5;          //arduino supply voltage
float Vout = 0;       //lid voltage read
float R1_Res = 5490;  // Lid potentiometer resistor
float R2_Res = 0;     //lid adjusted resistor
float buffer2 = 0;
int temp = 0;
int char_temp = 0;
int temp_lid = 0;

//Set hold times
unsigned long hold_init = 0;  // Initial hold time
unsigned long init_hr = 0;
unsigned long init_min = 0;
unsigned long init_sec = 0;
unsigned long hold_denat = 0;  // Stage hold time in seconds
unsigned long denat_hr = 0;
unsigned long denat_min = 0;
unsigned long denat_sec = 0;
unsigned long hold_anneal = 0;  // Stage hold time in seconds
unsigned long anneal_hr = 0;
unsigned long anneal_min = 0;
unsigned long anneal_sec = 0;
unsigned long hold_ext = 0;  // Stage hold time in seconds
unsigned long ext_hr = 0;
unsigned long ext_min = 0;
unsigned long ext_sec = 0;
unsigned long hold_final = 0;
unsigned long final_hr = 0;
unsigned long final_min = 0;
unsigned long final_sec = 0;
unsigned long hold_activ = 0;
unsigned long activ_hr = 0;
unsigned long activ_min = 0;
unsigned long activ_sec = 0;

unsigned long rna_hold_1 = 0;
unsigned long rna_hold_1_hr = 0;
unsigned long rna_hold_1_min = 0;
unsigned long rna_hold_1_sec = 0;
unsigned long rna_hold_2 = 0;
unsigned long rna_hold_2_hr = 0;
unsigned long rna_hold_2_min = 0;
unsigned long rna_hold_2_sec = 0;
unsigned long rna_hold_3 = 0;
unsigned long rna_hold_3_hr = 0;
unsigned long rna_hold_3_min = 0;
unsigned long rna_hold_3_sec = 0;

unsigned long rna_hold;
unsigned long rna_extract_hold = 0;
unsigned long rna_extract_hr = 0;
unsigned long rna_extract_min = 0;
unsigned long rna_extract_sec = 0;

//Set stage temperatures
int init_temp = 0;
int denat_temp = 0;
int anneal_temp = 0;
int ext_temp = 0;
int final_temp = 0;
int lid_temp = 60;
int activ_temp = 0;
int rna_temp_1 = 0;
int rna_temp_2 = 0;
int rna_temp_3 = 0;
int rna_temp;
int rna_step = 1;
int rna_extract_temp = 0;

int emer_temp = 105;

int buzz_count = 0;

//Pins defintions for arduino I/O devices
int chan1 = 39;   //relay module channel 1
int chan2 = 41;   //relay module channel 2
int peltier = 6;  // peltier module enable
int buzzer = 5;   //buzzer pin
int LED_RED = 8;
int LED_YEL = 9;
int LED_BLU = 10;
int btn_start = 23;
int btn_pause = 36;
int lid = 4;  //lid power MOSFET enable
int btn_qPCR = 37;
int light = 42;  //LED PM enable

int PCR_output = 48;  //reaction begin flag
//int  fluo_set = 51;
int end_reaction = 12;  //end reaction flag
int ON_output = 11;     //reaction in progress flag
//int  ON_output = 53;//use for pause output
int ESP_sense = 13;  //ESP incoming data flag
int esp_mega = 25;   //I/O pin
//int esp_mega_2 = 25;  //I/O pin

int Res_Pin = A0;  //lid voltage pin

//thermocouple pins
int thermoDO = 50;
int thermoCS = 29;
int thermoCLK = 52;

const int chipSelect = 40;  //SD Card


int status = 26;  //if idle or reaction in progress
int count_status = 0;
int timer_mult = 1;
//Set duty cycles for each stage hold
//int  anneal_duty = 0;
//int  ext_duty = 0;
//int  denat_duty = 0;
//int lid_duty;

//Stages definition
int idle = 0;
int start = 1;  // Initial heating
int denat = 2;
int anneal = 3;
int ext = 4;
int emergency = 5;
int set = 6;       // set PCR parameters
int fin = 7;       // Final hold
int lid_heat = 8;  // heat lid stage
int select = 9;
int rna_set = 10;
int reverse = 11;
int extract = 12;
int extract_set = 13;
int activation = 14;
int end_cycle = 15;
int fluo_meas = 16;
int pcr = 17;

//Variables definiton
int num_cycles = 0;  // total num. of cycles to run
int cycle = 0;       // current num. of cycles run
int num_steps = 0;
int num_steps_rna = 0;
int type = 0;  // type of reaction

double temp_rate = 0;   // temp. variation rate between stages
double rate_start = 0;  // hold initial temp.
double rate_end = 0;    // hold current step temp

double heat_rate = 1.75;  //heat/cool ramp

bool emer_blink = true;  // blink control for LED
bool pause = false;
bool qPCR = false;
bool buzz = false;
bool break_flag = false;

int flag_rate = 0;   // flag to control rate_start & rate_end variables
int flag = 0;        // interrupt flag for lcd % timer update
int flag_step = 0;   // step timer update
int flag_btn = 1;    // start btn. listener
int flag_pause = 1;  // start btn. listener
int flag_end = 0;    // end of cycles monitor
int flag_qPCR = 1;   // qPCR btn. listener
int flag_pid = 0;
int flag_pid_2 = 0;
int flag_rev_step_count = 0;
int flag_auto = 0;
int flag_manual = 0;

int step_count = 0;
int manual_count = 0;
int BT_count = 0;

int lid_flag = 0;
unsigned long lid_flag_time = 0;
unsigned long time_old = 0;
unsigned long time_new = 0;
unsigned long time_diff = 0;

int flag_disp = 2;

int rate_count = 0;  // step time elapsed
int count_time = 0;  // total rum time timer flag
int count_hold = 0;  // stage hold time counter flag
int count_step = 0;  // step run time flag
int count_qPCR = 0;

char num_samples = 0;
char num_nc = 0;
int flag_nc = 0;
int flag_set = 0;
char num_set[16] = { 0 };
int num_set_total;
int count_set = 0;
int set_indx = 0;

int pcr_step = idle;  // cuurent stage

int flag_read = 0;
int flag_time = 0;
int flag_tm = 0;
int flag_lcd_init = 0;
int flag_ESP = 0;

//Time variables definition
unsigned long sec = 0;          // total run time num. of seconds
unsigned long estim_sec = 0;    // estimated total run time
unsigned long avg_sec = 0;      // current cycle run time
unsigned long avg_sec_mem = 0;  // avg. overall cycle run time
unsigned long rem_sec = 0;      // remaining run time
unsigned long step_rem = 0;
unsigned long rem_sec_rna = 0;
unsigned long rem_hour;
unsigned long rem_min;
unsigned long rem_sec_serial;

//int esp_counter;
unsigned long step_rem_hour_display = 0;
unsigned long step_rem_min_display = 0;
unsigned long step_rem_sec_display = 0;
unsigned long rem_hour_display = 0;
unsigned long rem_min_display = 0;
unsigned long rem_sec_display = 0;

int percentage_elap = 0;

char run_time[20];
char step_time[20];
char avg_time[20];
char rem_time[20];
char Step_Rem[20];

int key_temp;
int temp_adjust;

int f1, f2, f3, f4, f5, f6, f7, f8;

int offset = 30;
int denat_minim, denat_maxim, anneal_minim, anneal_maxim, ext_minim, ext_maxim;

//Display variables definition
String stage = "Idle:";  // holds current stage string variable
String trend;            // holds temp. variation (heating, cooling or hold)
String inc_data_s;
String animal;
String disease;
String character;
int inc_data;
int disease_code;

//Define Variables we'll be connecting to
double denatSetpoint, denatInput, denatOutput, denatOutputP;
double annealSetpoint, annealInput, annealOutput, annealOutputP;
double extSetpoint, extInput, extOutput, extOutputP;
//double rnaSetpoint1, rnaInput1, rnaOutput1, rnaSetpoint2, rnaInput2, rnaOutput2, rnaSetpoint3, rnaInput3, rnaOutput3;
double rnaSetpoint, rnaInput, rnaOutput;
double revSetpoint, revInput, revOutput;
//double revCSetpoint, revCInput, revCOutput;
double lidSetpoint = 0, lidInput, lidOutput;

//Specify the links and initial tuning parameters
PID denatPID(&denatInput, &denatOutput, &denatSetpoint, 6, 5, 3, DIRECT);
PID annealPID(&annealInput, &annealOutput, &annealSetpoint, 12, 5, 3, DIRECT);
PID extPID(&extInput, &extOutput, &extSetpoint, 12, 5, 3, DIRECT);
PID rnaPID(&rnaInput, &rnaOutput, &rnaSetpoint, 12, 5, 3, DIRECT);
//PID lidPID(&lidInput, &lidOutput, &lidSetpoint, 75, 2, 3, DIRECT);
PID lidPID(&lidInput, &lidOutput, &lidSetpoint, 75, 25, 25, DIRECT);
PID revPID(&revInput, &revOutput, &revSetpoint, 6, 5, 3, DIRECT);
//PID revPIDC(&revCInput, &revCOutput, &revCSetpoint, 6, 2, 3, REVERSE);

//PID rnaPID1(&rnaInput1, &rnaOutput1, &rnaSetpoint1, 30, 2, 3,DIRECT);
//PID rnaPID2(&rnaInput2, &rnaOutput2, &rnaSetpoint2, 30, 2, 3,DIRECT);
//PID rnaPID3(&rnaInput3, &rnaOutput3, &rnaSetpoint3, 30, 2, 3,DIRECT);

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

void setup() {

  // Start Serial
  Serial1.begin(115200);
  delay(100);
  Serial.begin(9600);

  pinMode(lid, OUTPUT);
  pinMode(chan1, OUTPUT);
  pinMode(chan2, OUTPUT);
  pinMode(peltier, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLU, OUTPUT);
  pinMode(LED_YEL, OUTPUT);
  pinMode(light, OUTPUT);
  pinMode(ON_output, OUTPUT);
  pinMode(PCR_output, OUTPUT);
  pinMode(btn_start, INPUT);
  pinMode(btn_qPCR, INPUT);
  pinMode(ESP_sense, INPUT);
  pinMode(A14, INPUT);
  pinMode(A0, INPUT);
  pinMode(A4, INPUT);
  pinMode(A3, INPUT);
  pinMode(A2, INPUT);
  pinMode(A1, INPUT);
  //pinMode(A14, INPUT);
  //  pinMode(A15, INPUT);
  // pinMode(13, INPUT);
  //pinMode(12, INPUT);
  pinMode(end_reaction, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(esp_mega, OUTPUT);
  pinMode(chipSelect, OUTPUT);
  pinMode(thermoCS, OUTPUT);



  digitalWrite(end_reaction, LOW);
  analogWrite(peltier, 0);
  digitalWrite(chan1, LOW);
  digitalWrite(chan2, LOW);
  //digitalWrite(pause_output,LOW);
  digitalWrite(PCR_output, LOW);
  digitalWrite(chipSelect, HIGH);
  digitalWrite(thermoCS, HIGH);


  //turn the PID on
  annealPID.SetMode(AUTOMATIC);
  extPID.SetMode(AUTOMATIC);
  denatPID.SetMode(AUTOMATIC);
  rnaPID.SetMode(AUTOMATIC);
  //rnaPID1.SetMode(AUTOMATIC);
  //rnaPID2.SetMode(AUTOMATIC);
  //rnaPID3.SetMode(AUTOMATIC);
  //annealPIDP.SetMode(AUTOMATIC);
  //extPIDP.SetMode(AUTOMATIC);
  //denatPIDP.SetMode(AUTOMATIC);
  lidPID.SetMode(AUTOMATIC);
  revPID.SetMode(AUTOMATIC);
  //revPIDC.SetMode(AUTOMATIC);

  lidSetpoint = lid_temp;
  //lidPID.SetOutputLimits(20, 175);


  //delay(50);
  //delay(50);

  //Serial.println("ATmega ok");

  // check for SD card presence and setup SD card

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    //Serial.println("Card failed, or not present");
    // don't do anything more:
    //while (1);
  }

  SPI.end();






  // initialize timer1
  noInterrupts();  // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 31250;            // compare match register 16MHz/256/2Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts
}

ISR(TIMER1_COMPA_vect)  // timer compare interrupt service routine
{
  flag = 1;
  count_time = count_time + 1;
  count_step = count_step + 1;

  if (count_time == 2) {  // happens every 1 second
    Count_Timer();
  }
  Flag_Check();
}

void loop() {
  Temp_Read();  // read well temperature
  Lid_Read();

  // if (flag_tm == 0){
  //   flag_read = 1;

  // flag_tm = 1;
  // flag_read = 0;
  // }

  if (((temp >= emer_temp) || (temp_lid > 150)) && (flag_lcd_init == 1)) {  // stop process if system overheats
    pcr_step = emergency;
  }

  if ((cycle <= num_cycles) && (pcr_step != end_cycle)) {  // run pcr cycles for specified number of cycles
    if (pcr_step == idle) {                                // idle state
      if (flag_end == 1) {
        //digitalWrite(PCR_output, HIGH);
        //delay(100);
        count_time = 0;
      }
      if (flag_lcd_init == 1) {
        flag_read = 1;
        RTC.read(tm);
        Serial1.print("page1.n0.val=");
        //delay(50);
        Serial1.print(tm.Month);
        //delay(50);
        Serial1.write(0xFF);
        //delay(50);
        Serial1.write(0xFF);
        //delay(50);
        Serial1.write(0xFF);
        //delay(50);
        Serial1.print("page1.n1.val=");
        //delay(50);
        Serial1.print(tm.Day);
        //delay(50);
        Serial1.write(0xFF);
        //delay(50);
        Serial1.write(0xFF);
        //delay(50);
        Serial1.write(0xFF);
        //delay(50);
        Serial1.print("page1.n2.val=");
        //delay(50);
        Serial1.print((tmYearToCalendar(tm.Year)) % 100);
        //delay(50);
        Serial1.write(0xFF);
        //delay(50);
        Serial1.write(0xFF);
        //delay(50);
        Serial1.write(0xFF);
        //delay(50);
        Serial1.print("page1.n3.val=");
        //delay(50);
        Serial1.print(tm.Hour);
        //delay(50);
        Serial1.write(0xFF);
        //delay(50);
        Serial1.write(0xFF);
        //delay(50);
        Serial1.write(0xFF);
        //delay(50);
        Serial1.print("page1.n4.val=");
        //delay(50);
        Serial1.print(tm.Minute);
        //delay(50);
        Serial1.write(0xFF);
        //delay(50);
        Serial1.write(0xFF);
        //delay(50);
        Serial1.write(0xFF);

        Serial1.print("page 1");
        //delay(50);
        Serial1.write(0xFF);
        //delay(50);
        Serial1.write(0xFF);
        //delay(50);
        Serial1.write(0xFF);

        Serial1.print("page1.");
        //delay(50);
        Serial1.write(0xFF);
        //delay(50);
        Serial1.write(0xFF);
        //delay(50);
        Serial1.write(0xFF);
        //delay(50);
        flag_lcd_init = 0;
        flag_read = 0;
        flag_ESP = 1;
      }

      if (flag_time == 1) {
        RTC.read(tm);
        flag_time = 0;
      }
      //Lid_Pid();
      //delay(250);
      flag_end = 0;
      flag_btn = 1;
      cycle = 0;
      num_cycles = 0;
      rna_step = 1;
      rna_hold_1 = 0;
      rna_hold_2 = 0;
      rna_hold_3 = 0;
      rna_temp_1 = 0;
      rna_temp_2 = 0;
      rna_temp_3 = 0;
      rna_hold_1_hr = 0;
      rna_hold_1_min = 0;
      rna_hold_1_sec = 0;
      rna_hold_2_hr = 0;
      rna_hold_2_min = 0;
      rna_hold_2_sec = 0;
      rna_hold_3_hr = 0;
      rna_hold_3_min = 0;
      rna_hold_3_sec = 0;

      rna_extract_temp = 0;
      rna_extract_hold = 0;
      rna_extract_hr = 0;
      rna_extract_min = 0;
      rna_extract_sec = 0;
      num_steps_rna = 0;
      num_steps = 0;
      temp_rate = 0;
      hold_init = 0;
      hold_final = 0;
      hold_denat = 0;
      hold_anneal = 0;
      hold_ext = 0;
      hold_activ = 0;
      key_temp = 0;
      init_temp = 0;
      init_hr = 0;
      init_min = 0;
      init_sec = 0;
      denat_temp = 0;
      denat_hr = 0;
      denat_min = 0;
      denat_sec = 0;
      anneal_temp = 0;
      anneal_hr = 0;
      anneal_min = 0;
      anneal_sec = 0;
      ext_temp = 0;
      ext_hr = 0;
      ext_min = 0;
      ext_sec = 0;
      activ_temp = 0;
      activ_hr = 0;
      activ_min = 0;
      activ_sec = 0;
      final_temp = 0;
      final_hr = 0;
      final_min = 0;
      final_sec = 0;
      type = 0;
      flag_auto = 0;
      flag_manual = 0;
      manual_count = 0;
      num_samples = 0;
      num_nc = 0;
      flag_nc = 0;
      flag_set = 0;
      num_set_total = 0;
      count_set = 0;
      buzz_count = 0;
      buzz = false;
      step_count = 0;
      set_indx = 0;
      for (int a = 0; a < 16; a++) {
        num_set[a] = 0;
      }

      stage = "Idle:";
      trend = "Press start";
      analogWrite(peltier, 0);
      analogWrite(lid, 0);
      digitalWrite(chan1, LOW);
      digitalWrite(chan2, LOW);
      digitalWrite(PCR_output, LOW);
      digitalWrite(end_reaction, LOW);
      digitalWrite(ON_output, LOW);
      digitalWrite(buzzer, LOW);
      LED(0, 0, 0);
      TimerReset();
      avg_sec = 0;

      if ((digitalRead(ESP_sense) == 1) && (pcr_step == idle) && (flag_ESP == 1)) {
        //if (0) {
        //Serial.print("ESP Sense");
        flag_read = 1;
        while (digitalRead(ESP_sense) == 1) {
          //while (0) {
          if (Serial.available() > 0) {
            char_temp = Serial.read();
            if (char_temp == 1) {
              while (char_temp != 127) {
                if (Serial.available() > 0) {
                  char_temp = Serial.read();
                  if (step_count == 0) {
                    rna_extract_temp = char_temp;
                    step_count = step_count + 1;
                  } else if (step_count == 1) {
                    rna_extract_hr = char_temp;
                    step_count = step_count + 1;
                  } else if (step_count == 2) {
                    rna_extract_min = char_temp;
                    step_count = step_count + 1;
                  } else if (step_count == 3) {
                    rna_extract_sec = char_temp;
                    step_count = step_count + 1;
                  } else if (step_count == 4) {
                    animal = Serial.readStringUntil('\0');
                    disease = Serial.readStringUntil('\0');
                    step_count = step_count + 1;
                  }
                }
              }
              rna_extract_hold = rna_extract_hr * 3600 + rna_extract_min * 60 + rna_extract_sec;
              pcr_step = lid_heat;
              type = 3;
              cycle = 1;
              num_cycles = 1;
              Rem_Time();
              rem_sec = estim_sec;
            }

            else if (char_temp == 2) {
              while (char_temp != 127) {
                if (Serial.available() > 0) {
                  char_temp = Serial.read();
                  if (step_count == 0) {
                    num_steps_rna = char_temp;
                    step_count = step_count + 1;
                  } else if (step_count == 1) {
                    rna_temp_1 = char_temp;
                    step_count = step_count + 1;
                  } else if (step_count == 2) {
                    rna_hold_1_hr = char_temp;
                    step_count = step_count + 1;
                  } else if (step_count == 3) {
                    rna_hold_1_min = char_temp;
                    step_count = step_count + 1;
                  } else if (step_count == 4) {
                    rna_hold_1_sec = char_temp;
                    rna_hold_1 = rna_hold_1_hr * 3600 + rna_hold_1_min * 60 + rna_hold_1_sec;
                    step_count = step_count + 1;
                  } else if (step_count == 5) {
                    rna_temp_2 = char_temp;
                    step_count = step_count + 1;
                  } else if (step_count == 6) {
                    rna_hold_2_hr = char_temp;
                    step_count = step_count + 1;
                  } else if (step_count == 7) {
                    rna_hold_2_min = char_temp;
                    step_count = step_count + 1;
                  } else if (step_count == 8) {
                    rna_hold_2_sec = char_temp;
                    rna_hold_2 = rna_hold_2_hr * 3600 + rna_hold_2_min * 60 + rna_hold_2_sec;
                    step_count = step_count + 1;
                  } else if (step_count == 9) {
                    rna_temp_3 = char_temp;
                    step_count = step_count + 1;
                  } else if (step_count == 10) {
                    rna_hold_3_hr = char_temp;
                    step_count = step_count + 1;
                  } else if (step_count == 11) {
                    rna_hold_3_min = char_temp;
                    step_count = step_count + 1;
                  } else if (step_count == 12) {
                    rna_hold_3_sec = char_temp;
                    rna_hold_3 = rna_hold_3_hr * 3600 + rna_hold_3_min * 60 + rna_hold_3_sec;
                    step_count = step_count + 1;
                  } else if (step_count == 13) {
                    animal = Serial.readStringUntil('\0');
                    disease = Serial.readStringUntil('\0');
                    step_count = step_count + 1;
                  }
                }
              }
              pcr_step = lid_heat;
              type = 1;
              num_cycles = 1;
              cycle = 1;
              //rem_sec = rna_hold_1 + rna_hold_2 + rna_hold_3;
              Rem_Time();
              rem_sec = estim_sec;
            }

            else if (char_temp == 3) {

              while (char_temp != 127) {
                if (Serial.available() > 0) {
                  char_temp = Serial.read();
                  if (step_count == 0) {
                    num_samples = char_temp;
                    step_count = step_count + 1;
                  } else if (step_count == 1) {
                    num_nc = char_temp;
                    if (num_nc == 0) {
                      step_count = step_count + 1;
                    } else {
                      BT_count = 0;
                      while (BT_count < num_nc) {
                        if (Serial.available() > 0) {
                          num_set[BT_count] = Serial.read();
                          BT_count = BT_count + 1;
                        }
                      }
                      step_count = step_count + 1;
                      BT_count = 0;
                    }
                  } else if (step_count == 2) {
                    activ_temp = char_temp;
                    step_count = step_count + 1;
                  } else if (step_count == 3) {
                    activ_hr = char_temp;
                    step_count = step_count + 1;
                  } else if (step_count == 4) {
                    activ_min = char_temp;
                    step_count = step_count + 1;
                  } else if (step_count == 5) {
                    activ_sec = char_temp;
                    hold_activ = activ_hr * 3600 + activ_min * 60 + activ_sec;
                    step_count = step_count + 1;
                  } else if (step_count == 6) {
                    num_steps = char_temp;
                    step_count = step_count + 1;
                  } else if (step_count == 7) {
                    num_cycles = char_temp;
                    step_count = step_count + 1;
                  } else if (step_count == 8) {
                    init_temp = char_temp;
                    step_count = step_count + 1;
                  } else if (step_count == 9) {
                    init_hr = char_temp;
                    step_count = step_count + 1;
                  } else if (step_count == 10) {
                    init_min = char_temp;
                    step_count = step_count + 1;
                  } else if (step_count == 11) {
                    init_sec = char_temp;
                    hold_init = init_hr * 3600 + init_min * 60 + init_sec;
                    step_count = step_count + 1;
                  } else if (step_count == 12) {
                    denat_temp = char_temp;
                    step_count = step_count + 1;
                  } else if (step_count == 13) {
                    denat_hr = char_temp;
                    step_count = step_count + 1;
                  } else if (step_count == 14) {
                    denat_min = char_temp;
                    step_count = step_count + 1;
                  } else if (step_count == 15) {
                    denat_sec = char_temp;
                    hold_denat = denat_hr * 3600 + denat_min * 60 + denat_sec;
                    if (num_steps == 2)
                      step_count = 20;
                    else if (num_steps == 3)
                      step_count = step_count + 1;
                  } else if (step_count == 16) {
                    anneal_temp = char_temp;
                    step_count = step_count + 1;
                  } else if (step_count == 17) {
                    anneal_hr = char_temp;
                    step_count = step_count + 1;
                  } else if (step_count == 18) {
                    anneal_min = char_temp;
                    step_count = step_count + 1;
                  } else if (step_count == 19) {
                    anneal_sec = char_temp;
                    hold_anneal = anneal_hr * 3600 + anneal_min * 60 + anneal_sec;
                    step_count = step_count + 1;
                  } else if (step_count == 20) {
                    ext_temp = char_temp;
                    step_count = step_count + 1;
                  } else if (step_count == 21) {
                    ext_hr = char_temp;
                    step_count = step_count + 1;
                  } else if (step_count == 22) {
                    ext_min = char_temp;
                    step_count = step_count + 1;
                  } else if (step_count == 23) {
                    ext_sec = char_temp;
                    hold_ext = ext_hr * 3600 + ext_min * 60 + ext_sec;
                    step_count = step_count + 1;
                  } else if (step_count == 24) {
                    final_temp = char_temp;
                    step_count = step_count + 1;
                  } else if (step_count == 25) {
                    final_hr = char_temp;
                    step_count = step_count + 1;
                  } else if (step_count == 26) {
                    final_min = char_temp;
                    step_count = step_count + 1;
                  } else if (step_count == 27) {
                    final_sec = char_temp;
                    hold_final = final_hr * 3600 + final_min * 60 + final_sec;
                    step_count = step_count + 1;
                  } else if (step_count == 28) {
                    animal = Serial.readStringUntil('\0');
                    disease = Serial.readStringUntil('\0');
                    step_count = step_count + 1;
                  }
                }
              }
              pcr_step = lid_heat;
              type = 2;
              denatSetpoint = denat_temp;
              annealSetpoint = anneal_temp;
              extSetpoint = ext_temp;
              rnaSetpoint = activ_temp;
              Rem_Time();
              rem_sec = estim_sec;
            }
          }
        }
        Serial.flush();
        delay(200);
        flag_read = 0;
      }
    }

    else if (pcr_step == extract_set) {
      //      digitalWrite(end_reaction, HIGH);
      //      digitalWrite(PCR_output, HIGH);
      flag_read = 1;
      while (pcr_step == extract_set) {
        Lid_Read();
        //Lid_Pid();
        if (Serial1.available() > 0) {
          inc_data = Read_data();
        }
        if (step_count == 0) {
          rna_extract_temp = inc_data;
          step_count++;
        } else if (step_count == 1) {
          rna_extract_hr = inc_data;
          step_count++;
        } else if (step_count == 2) {
          rna_extract_min = inc_data;
          step_count++;
        } else if (step_count == 3) {
          rna_extract_sec = inc_data;
          rna_extract_hold = rna_extract_hr * 3600 + rna_extract_min * 60 + rna_extract_sec;
          step_count++;
        } else if (step_count == 4) {
          if (inc_data == 254) {
            pcr_step = lid_heat;
            type = 3;
            cycle = 1;
            num_cycles = 1;
            //estim_sec = abs((lid_temp - temp_lid)/2) + abs((rna_extract_temp - temp) / 2) + rna_extract_hold;
            Rem_Time();
            rem_sec = estim_sec;
            flag_read = 0;
          } else {
            pcr_step = idle;
            flag_read = 0;
          }
        }
      }
    }

    else if (pcr_step == rna_set) {  //RNA Rev. transc. settings
      //      digitalWrite(end_reaction, HIGH);
      //      digitalWrite(PCR_output, HIGH);
      flag_read = 1;
      while (pcr_step == rna_set) {
        Lid_Read();
        //Lid_Pid();
        // TimerReset();

        if ((step_count < 12) && (Serial1.available() > 0)) {
          inc_data = Read_data();

          if (num_steps_rna == 0) {
            num_steps_rna = inc_data;
          } else if (step_count == 0) {
            rna_temp_1 = inc_data;
            step_count = step_count + 1;
          } else if (step_count == 1) {
            rna_hold_1_hr = inc_data;
            step_count = step_count + 1;
          } else if (step_count == 2) {
            rna_hold_1_min = inc_data;
            step_count = step_count + 1;
          } else if (step_count == 3) {
            rna_hold_1_sec = inc_data;
            rna_hold_1 = rna_hold_1_hr * 3600 + rna_hold_1_min * 60 + rna_hold_1_sec;
            if ((num_steps_rna == 1)) {
              step_count = 12;
            } else
              step_count = step_count + 1;
          } else if (step_count == 4) {
            rna_temp_2 = inc_data;
            step_count = step_count + 1;
          } else if (step_count == 5) {
            rna_hold_2_hr = inc_data;
            step_count = step_count + 1;
          } else if (step_count == 6) {
            rna_hold_2_min = inc_data;
            step_count = step_count + 1;
          } else if (step_count == 7) {
            rna_hold_2_sec = inc_data;
            rna_hold_2 = rna_hold_2_hr * 3600 + rna_hold_2_min * 60 + rna_hold_2_sec;
            if ((num_steps_rna == 2)) {
              step_count = 12;
            } else
              step_count = step_count + 1;
          } else if (step_count == 8) {
            rna_temp_3 = inc_data;
            step_count = step_count + 1;
          } else if (step_count == 9) {
            rna_hold_3_hr = inc_data;
            step_count = step_count + 1;
          } else if (step_count == 10) {
            rna_hold_3_min = inc_data;
            step_count = step_count + 1;
          } else if (step_count == 11) {
            rna_hold_3_sec = inc_data;
            rna_hold_3 = rna_hold_3_hr * 3600 + rna_hold_3_min * 60 + rna_hold_3_sec;
            step_count = step_count + 1;
          }
        } else if ((step_count == 12)) {
          if (Serial1.available() > 0) {
            inc_data = Read_data();
          }
          if (inc_data == 254) {
            pcr_step = lid_heat;
            type = 1;
            Rem_Time();
            rem_sec = estim_sec;
            num_cycles = 1;
            cycle = 1;
            flag_read = 0;
            // Serial1.flush();
          } else {
            pcr_step = idle;
            flag_read = 0;
          }
        }
      }
    }

    else if (pcr_step == set) {  // PCR settings
      //      digitalWrite(end_reaction, HIGH);
      //      digitalWrite(PCR_output, HIGH);
      flag_read = 1;
      while (pcr_step == set) {
        Lid_Read();
        //Lid_Pid();
        //char customKey = customKeypad.getKey();

        //digitalWrite(PCR_output, LOW);


        if (Serial1.available() > 0) {
          inc_data = Read_data();
        }
        if (step_count == 0) {
          num_samples = inc_data;
          step_count = step_count + 1;
        } else if (step_count == 1) {
          activ_temp = inc_data;
          step_count = step_count + 1;
        } else if (step_count == 2) {
          activ_hr = inc_data;
          step_count = step_count + 1;
        } else if (step_count == 3) {
          activ_min = inc_data;
          step_count = step_count + 1;
        } else if (step_count == 4) {
          activ_sec = inc_data;
          hold_activ = activ_hr * 3600 + activ_min * 60 + activ_sec;
          step_count = step_count + 1;
        } else if (step_count == 5) {
          num_steps = inc_data;
          step_count = step_count + 1;
        } else if (step_count == 6) {
          num_cycles = inc_data;
          step_count = step_count + 1;
        } else if (step_count == 7) {
          init_temp = inc_data;
          step_count = step_count + 1;
        } else if (step_count == 8) {
          init_hr = inc_data;
          step_count = step_count + 1;
        } else if (step_count == 9) {
          init_min = inc_data;
          step_count = step_count + 1;
        } else if (step_count == 10) {
          init_sec = inc_data;
          hold_init = init_hr * 3600 + init_min * 60 + init_sec;
          step_count = step_count + 1;
        } else if (step_count == 11) {
          denat_temp = inc_data;
          step_count = step_count + 1;
        } else if (step_count == 12) {
          denat_hr = inc_data;
          step_count = step_count + 1;
        } else if (step_count == 13) {
          denat_min = inc_data;
          step_count = step_count + 1;
        } else if (step_count == 14) {
          denat_sec = inc_data;
          hold_denat = denat_hr * 3600 + denat_min * 60 + denat_sec;
          if (num_steps == 3) {
            step_count = step_count + 1;
          } else if (num_steps == 2) {
            step_count = 19;
          }
        } else if (step_count == 15) {
          anneal_temp = inc_data;
          step_count = step_count + 1;
        } else if (step_count == 16) {
          anneal_hr = inc_data;
          step_count = step_count + 1;
        } else if (step_count == 17) {
          anneal_min = inc_data;
          step_count = step_count + 1;
        } else if (step_count == 18) {
          anneal_sec = inc_data;
          hold_anneal = anneal_hr * 3600 + anneal_min * 60 + anneal_sec;
          step_count = step_count + 1;
        } else if (step_count == 19) {
          ext_temp = inc_data;
          step_count = step_count + 1;
        } else if (step_count == 20) {
          ext_hr = inc_data;
          step_count = step_count + 1;
        } else if (step_count == 21) {
          ext_min = inc_data;
          step_count = step_count + 1;
        } else if (step_count == 22) {
          ext_sec = inc_data;
          hold_ext = ext_hr * 3600 + ext_min * 60 + ext_sec;
          step_count = step_count + 1;
        } else if (step_count == 23) {
          final_temp = inc_data;
          step_count = step_count + 1;
        } else if (step_count == 24) {
          final_hr = inc_data;
          step_count = step_count + 1;
        } else if (step_count == 25) {
          final_min = inc_data;
          step_count = step_count + 1;
        } else if (step_count == 26) {
          final_sec = inc_data;
          hold_final = final_hr * 3600 + final_min * 60 + final_sec;
          step_count = step_count + 1;
        } else if (step_count == 27) {
          if (inc_data == 254) {
            pcr_step = lid_heat;
            type = 2;
            Rem_Time();
            rem_sec = estim_sec;
            denatSetpoint = denat_temp;
            annealSetpoint = anneal_temp;
            extSetpoint = ext_temp;
            rnaSetpoint = activ_temp;
            flag_read = 0;
          } else {
            pcr_step = idle;
            flag_read = 0;
          }
        }
      }
    }

    else if (pcr_step == lid_heat) {
      stage = "Heating lid...";
      trend = "";
      Dispfnc();

      Rem_Time();

      flag_read = 1;

      Serial1.print("page 33");
      //delay(50);
      Serial1.write(0xFF);
      //delay(50);
      Serial1.write(0xFF);
      //delay(50);
      Serial1.write(0xFF);
      //delay(50);

      digitalWrite(end_reaction, LOW);
      digitalWrite(PCR_output, HIGH);
      delay(60);
      Serial.write(type);
      delay(60);

      if ((type == 1)) {
        Serial.write(num_steps_rna);
        delay(60);
        Serial.write(rna_step);
        delay(60);
        if (num_steps_rna == 1) {
          Serial.write(rna_temp_1);
          delay(60);
          Serial.write(rna_hold_1_hr);
          delay(60);
          Serial.write(rna_hold_1_min);
          delay(60);
          Serial.write(rna_hold_1_sec);
          delay(60);
          rem_hour = rem_sec / 3600;
          rem_min = (rem_sec % 3600) / 60;
          rem_sec_serial = rem_sec % 60;
          Serial.write(rem_hour);
          delay(60);
          Serial.write(rem_min);
          delay(60);
          Serial.write(rem_sec_serial);
          delay(60);
        } else if (num_steps_rna == 2) {
          Serial.write(rna_temp_1);
          delay(60);
          Serial.write(rna_hold_1_hr);
          delay(60);
          Serial.write(rna_hold_1_min);
          delay(60);
          Serial.write(rna_hold_1_sec);
          delay(60);
          Serial.write(rna_temp_2);
          delay(60);
          Serial.write(rna_hold_2_hr);
          delay(60);
          Serial.write(rna_hold_2_min);
          delay(60);
          Serial.write(rna_hold_2_sec);
          delay(60);
          rem_hour = rem_sec / 3600;
          rem_min = (rem_sec % 3600) / 60;
          rem_sec_serial = rem_sec % 60;
          Serial.write(rem_hour);
          delay(60);
          Serial.write(rem_min);
          delay(60);
          Serial.write(rem_sec_serial);
          delay(60);
        } else if (num_steps_rna == 3) {
          Serial.write(rna_temp_1);
          delay(60);
          Serial.write(rna_hold_1_hr);
          delay(60);
          Serial.write(rna_hold_1_min);
          delay(60);
          Serial.write(rna_hold_1_sec);
          delay(60);
          Serial.write(rna_temp_2);
          delay(60);
          Serial.write(rna_hold_2_hr);
          delay(60);
          Serial.write(rna_hold_2_min);
          delay(60);
          Serial.write(rna_hold_2_sec);
          delay(60);
          Serial.write(rna_temp_3);
          delay(60);
          Serial.write(rna_hold_3_hr);
          delay(60);
          Serial.write(rna_hold_3_min);
          delay(60);
          Serial.write(rna_hold_3_sec);
          delay(60);
          rem_hour = rem_sec / 3600;
          rem_min = (rem_sec % 3600) / 60;
          rem_sec_serial = rem_sec % 60;
          Serial.write(rem_hour);
          delay(60);
          Serial.write(rem_min);
          delay(60);
          Serial.write(rem_sec_serial);
          delay(60);
        }
        String myString = "Rev. Transcription";
        String command = "page36.t0.txt=\"" + myString + "\"";
        Serial1.print(command);
        //delay(50);
        Serial1.write(0xFF);
        //delay(50);
        Serial1.write(0xFF);
        //delay(50);
        Serial1.write(0xFF);
        animal = "Rev. Transc";
        disease = " ";
      } else if ((type == 2) || (type == 4)) {
        Serial.write(num_samples);
        delay(60);
        Serial.write(num_nc);
        delay(60);
        if (num_nc == 1) {
          Serial.write(num_samples);
          delay(60);
        } else if (num_nc > 1) {
          for (int i = 0; i < num_nc; i++) {
            Serial.write(num_set[i]);
            delay(60);
          }
        }
        Serial.write(activ_temp);
        delay(60);
        Serial.write(activ_hr);
        delay(60);
        Serial.write(activ_min);
        delay(60);
        Serial.write(activ_sec);
        delay(60);
        Serial.write(num_steps);
        delay(60);
        Serial.write(num_cycles);
        delay(60);
        Serial.write(init_temp);
        delay(60);
        Serial.write(init_hr);
        delay(60);
        Serial.write(init_min);
        delay(60);
        Serial.write(init_sec);
        delay(60);
        Serial.write(denat_temp);
        delay(60);
        Serial.write(denat_hr);
        delay(60);
        Serial.write(denat_min);
        delay(60);
        Serial.write(denat_sec);
        delay(60);
        if (num_steps == 3) {
          Serial.write(anneal_temp);
          delay(60);
          Serial.write(anneal_hr);
          delay(60);
          Serial.write(anneal_min);
          delay(60);
          Serial.write(anneal_sec);
          delay(60);
        }
        Serial.write(ext_temp);
        delay(60);
        Serial.write(ext_hr);
        delay(60);
        Serial.write(ext_min);
        delay(60);
        Serial.write(ext_sec);
        delay(60);
        Serial.write(final_temp);
        delay(60);
        Serial.write(final_hr);
        delay(60);
        Serial.write(final_min);
        delay(60);
        Serial.write(final_sec);
        delay(60);
        Serial.write(cycle);
        delay(60);
        rem_hour = rem_sec / 3600;
        rem_min = (rem_sec % 3600) / 60;
        rem_sec_serial = rem_sec % 60;
        Serial.write(rem_hour);
        delay(60);
        Serial.write(rem_min);
        delay(60);
        Serial.write(rem_sec_serial);
        delay(60);
        if (type == 2) {
          animal = "PCR";
          disease = " ";
        }
        String myString = "PCR";
        String command = "page36.t0.txt=\"" + myString + "\"";
        Serial1.print(command);
        //delay(50);
        Serial1.write(0xFF);
        //delay(50);
        Serial1.write(0xFF);
        //delay(50);
        Serial1.write(0xFF);
      } else if ((type == 3)) {
        Serial.write(1);
        delay(60);
        Serial.write(1);
        delay(60);
        Serial.write(rna_extract_temp);
        delay(60);
        Serial.write(rna_extract_hr);
        delay(60);
        Serial.write(rna_extract_min);
        delay(60);
        Serial.write(rna_extract_sec);
        delay(60);
        rem_hour = rem_sec / 3600;
        rem_min = (rem_sec % 3600) / 60;
        rem_sec_serial = rem_sec % 60;
        Serial.write(rem_hour);
        delay(60);
        Serial.write(rem_min);
        delay(60);
        Serial.write(rem_sec_serial);
        delay(60);
        String myString = "Extraction";
        String command = "page36.t0.txt=\"" + myString + "\"";
        Serial1.print(command);
        //delay(50);
        Serial1.write(0xFF);
        //delay(50);
        Serial1.write(0xFF);
        //delay(50);
        Serial1.write(0xFF);
        animal = "Extraction";
        disease = " ";
      }
      // String myString = animal;
      // String myString2 = disease;
      // String command = "page36.t0.txt=\"" + myString + " " + myString2 + "\"";
      // Serial1.print(command);
      // //delay(50);
      // Serial1.write(0xFF);
      // //delay(50);
      // Serial1.write(0xFF);
      // //delay(50);
      // Serial1.write(0xFF);
      Serial.write(animal.c_str());
      delay(100);
      Serial.write('\0');
      delay(60);
      Serial.write(disease.c_str());
      delay(100);
      Serial.write('\0');
      delay(60);
      digitalWrite(PCR_output, LOW);
      delay(60);
      digitalWrite(ON_output, HIGH);

      flag_read = 0;

      //  while (0) {
      while (temp_lid < lid_temp) {
        Temp_Read();  // read well temperature
        Lid_Read();
        Lid_Pid();
        if (flag == 1) {
          Dispfnc();
        }
        if (flag_btn == 0) {
          pcr_step = end_cycle;
          break;
        }
      }
      //     pcr_step = (type == 1) * reverse + (type == 2) * activation + (type == 3) * extract;
      if (flag_btn != 0) {
        if (type == 1) {
          pcr_step = reverse;
        } else if (type == 3) {
          pcr_step = extract;
        } else if ((type == 2) || (type == 4)) {
          if ((hold_activ == 0) || (activ_temp == 0))
            pcr_step = start;
          else
            pcr_step = activation;
        }
        TimerReset();
      }
    }

    else if (pcr_step == activation) {
      cycle = 0;
      Lid_Pid();
      stage = "Activat.:";
      rnaSetpoint = activ_temp;
      if (temp < (activ_temp - 2)) {
        trend = "heat...";
        digitalWrite(chan1, HIGH);
        digitalWrite(chan2, HIGH);
        delay(50);
        analogWrite(peltier, 255);
        LED(1, 0, 0);
        if (flag_step == 1) {
          Step_Time();
        }
      } else {
        trend = "hold...";
        LED(0, 1, 0);
        step_rem = hold_activ;
        while ((count_hold < ((hold_activ * 2))) && (temp < emer_temp)) {
          Temp_Read();
          rnaInput = temp;
          rnaPID.Compute();
          analogWrite(peltier, rnaOutput);
          Lid_Read();
          Lid_Pid();
          //delay(250);

          if (flag == 1) {
            hold_func();
            Dispfnc();
          }
          if (flag_btn == 0) {
            pcr_step = end_cycle;
            break;
          }
        }
        if (flag_btn != 0) {
          analogWrite(peltier, 0);
          delay(50);
          if ((hold_init == 0) || (init_temp == 0)) {
            pcr_step = denat;
          } else {
            pcr_step = start;
          }
          TimerReset();
        }
      }
    }

    else if (pcr_step == start) {  //Initial high temp
      cycle = 0;
      Lid_Pid();
      stage = "Initializ.:";
      denatSetpoint = init_temp;
      if (temp < (init_temp - 4)) {
        trend = "heat...";
        digitalWrite(chan1, HIGH);
        digitalWrite(chan2, HIGH);
        delay(50);
        analogWrite(peltier, 255);
        LED(1, 0, 0);
        if (flag_step == 1) {
          Step_Time();
        }
      } else {
        trend = "hold...";
        LED(0, 1, 0);
        step_rem = hold_init;
        while ((count_hold < ((hold_init * 2))) && (temp < emer_temp)) {
          Temp_Read();
          if (temp > (init_temp))
            analogWrite(peltier, 0);
          else {
            denatPID.Compute();
            analogWrite(peltier, denatOutput);
          }
          Lid_Read();
          Lid_Pid();
          //delay(250);

          if (flag == 1) {
            hold_func();
            Dispfnc();
          }
          if (flag_btn == 0) {
            pcr_step = end_cycle;
            break;
          }
        }
        if (flag_btn != 0) {
          analogWrite(peltier, 0);
          delay(50);
          pcr_step = denat;
          cycle = cycle + 1;
          TimerReset();
        }
      }
    } else if (pcr_step == denat) {  //Denaturation
      stage = "Denat.:";
      Lid_Pid();
      denatSetpoint = denat_temp;

      if (temp < (denat_temp - 4)) {
        trend = "heat...";
        digitalWrite(chan1, HIGH);
        digitalWrite(chan2, HIGH);
        delay(50);
        analogWrite(peltier, 255);
        LED(1, 0, 0);
        if (flag_step == 1) {
          Step_Time();
        }
      } else {

        flag_read = 1;
        digitalWrite(PCR_output, HIGH);
        delay(60);
        Serial.write(type);
        delay(60);
        Serial.write(cycle);
        delay(60);

        rem_hour = rem_sec / 3600;
        rem_min = (rem_sec % 3600) / 60;
        rem_sec_serial = rem_sec % 60;
        Serial.write(rem_hour);
        delay(60);
        Serial.write(rem_min);
        delay(60);
        Serial.write(rem_sec_serial);
        delay(60);

        digitalWrite(PCR_output, LOW);

        flag_read = 0;

        trend = "hold...";
        LED(0, 1, 0);
        step_rem = hold_denat + 2;
        while ((count_hold < ((hold_denat * 2 + 4))) && (temp < emer_temp)) {
          Temp_Read();
          denatInput = temp;
          if (temp > (denat_temp))
            analogWrite(peltier, 0);
          else {
            denatPID.Compute();
            analogWrite(peltier, denatOutput);
          }
          Lid_Read();
          Lid_Pid();
          //delay(250);

          if (flag == 1) {
            hold_func();
            Dispfnc();
          }
          if (flag_btn == 0) {
            pcr_step = end_cycle;
            break;
          }
        }
        if (flag_btn != 0) {
          if (num_steps == 2) {
            analogWrite(peltier, 0);
            delay(50);
            pcr_step = ext;
          } else {
            analogWrite(peltier, 0);
            delay(50);
            pcr_step = anneal;
          }
          TimerReset();
        }
      }
    } else if (pcr_step == anneal) {  // Annealing
      stage = "Anneal.:";
      Lid_Pid();
      annealSetpoint = anneal_temp;
      if (temp > (anneal_temp + 2)) {
        trend = "cooling...";
        digitalWrite(chan1, LOW);
        digitalWrite(chan2, LOW);
        delay(50);
        analogWrite(peltier, 255);
        //digitalWrite(peltier, HIGH);
        LED(0, 0, 1);
        if (flag_step == 1) {
          Step_Time();
        }
      } else {
        trend = "hold...";
        analogWrite(peltier, 0);
        delay(50);
        digitalWrite(chan1, HIGH);
        digitalWrite(chan2, HIGH);
        delay(50);
        LED(0, 1, 0);
        step_rem = hold_anneal;
        while ((count_hold < ((hold_anneal * 2))) && (temp < emer_temp)) {
          Temp_Read();
          annealInput = temp;
          annealPID.Compute();
          analogWrite(peltier, annealOutput);
          //analogWrite(peltier, 128);
          //delay(10);
          Lid_Read();
          Lid_Pid();
          //delay(250);

          if (flag == 1) {
            hold_func();
            Dispfnc();
          }
          if (flag_btn == 0) {
            pcr_step = end_cycle;
            break;
          }
        }
        if (flag_btn != 0) {
          analogWrite(peltier, 0);
          delay(50);
          pcr_step = ext;
          TimerReset();
        }
      }
    } else if (pcr_step == ext) {  //  Extension
      Lid_Pid();
      extSetpoint = ext_temp;
      if ((temp < (ext_temp - 2)) && (num_steps == 3)) {
        stage = "Extension:";
        trend = "heat...";
        digitalWrite(chan1, HIGH);
        digitalWrite(chan2, HIGH);
        delay(50);
        analogWrite(peltier, 255);
        LED(1, 0, 0);
        if (flag_step == 1) {
          Step_Time();
        }
      } else if ((temp > (ext_temp + 2)) && (num_steps == 2)) {
        stage = "An-Ext:";
        trend = "cooling...";
        digitalWrite(chan1, LOW);
        digitalWrite(chan2, LOW);
        delay(50);
        analogWrite(peltier, 255);
        LED(0, 0, 1);
        if (flag_step == 1) {
          Step_Time();
        }
      } else {
        trend = "hold...";
        analogWrite(peltier, 0);
        delay(50);
        digitalWrite(chan1, HIGH);
        digitalWrite(chan2, HIGH);
        delay(50);
        LED(0, 1, 0);
        step_rem = hold_ext;
        while ((count_hold < ((hold_ext * 2))) && (temp < emer_temp)) {
          Temp_Read();
          extInput = temp;
          extPID.Compute();
          analogWrite(peltier, extOutput);
          Lid_Read();
          Lid_Pid();
          //delay(250);

          if (flag == 1) {
            hold_func();
            Dispfnc();
          }
          if (flag_btn == 0) {
            pcr_step = end_cycle;
            break;
          }
        }
        if (flag_btn != 0) {
          analogWrite(peltier, 0);
          delay(50);
          pcr_step = denat;
          TimerReset();
          Avg_Time();
          Rem_Time();
          Estim_Time();
        }
        if (cycle == num_cycles) {  // extra extension time at last cycle
          stage = "Final Ext.:";
          trend = "hold...";
          digitalWrite(chan1, HIGH);
          digitalWrite(chan2, HIGH);
          delay(50);
          LED(0, 1, 0);
          step_rem = hold_final;
          extSetpoint = final_temp;
          while ((count_hold < (hold_final * 2)) && (temp < emer_temp)) {
            Temp_Read();
            extInput = temp;
            extPID.Compute();
            analogWrite(peltier, extOutput);
            Lid_Read();
            Lid_Pid();
            //delay(250);

            if (flag == 1) {
              hold_func();
              Dispfnc();
            }
            if (flag_btn == 0) {
              pcr_step = end_cycle;
              break;
            }
          }
        }
        cycle = cycle + 1;
      }
    } else if (pcr_step == extract) {

      Lid_Pid();

      rna_hold = rna_extract_hold;
      rnaSetpoint = rna_extract_temp;
      rna_temp = rna_extract_temp;
      if (temp < (rna_temp - 2)) {
        trend = "heat...";
        digitalWrite(chan1, HIGH);
        digitalWrite(chan2, HIGH);
        delay(50);
        analogWrite(peltier, 255);
        LED(1, 0, 0);
      } else {
        trend = "hold...";
        LED(0, 1, 0);
        step_rem = rna_hold;
        rem_sec = rna_hold;

        flag_read = 1;
        digitalWrite(PCR_output, HIGH);
        delay(60);
        Serial.write(type);
        delay(60);
        rem_hour = step_rem / 3600;
        rem_min = (step_rem % 3600) / 60;
        rem_sec_serial = step_rem % 60;
        Serial.write(rem_hour);
        delay(60);
        Serial.write(rem_min);
        delay(60);
        Serial.write(rem_sec_serial);
        delay(60);
        digitalWrite(PCR_output, LOW);
        flag_read = 0;

        while (count_hold < ((rna_hold * 2)) && (temp < emer_temp)) {
          Temp_Read();
          rnaInput = temp;
          rnaPID.Compute();
          analogWrite(peltier, rnaOutput);
          Lid_Read();
          Lid_Pid();
          //delay(250);

          if (flag == 1) {
            hold_func();
            Dispfnc();
          }
          if (flag_btn == 0) {
            pcr_step = end_cycle;
            break;
          }
        }
        cycle = cycle + 1;
      }
    }

    else if (pcr_step == reverse) {
      stage = "Rev. Transc.";
      Lid_Pid();
      if (flag_rev_step_count == 0) {
        if (rna_step == 1) {
          rna_hold = rna_hold_1;
          rnaSetpoint = rna_temp_1;
          rna_temp = rna_temp_1;
        } else if (rna_step == 2) {
          rna_hold = rna_hold_2;
          rnaSetpoint = rna_temp_2;
          rna_temp = rna_temp_2;
        } else if (rna_step == 3) {
          rna_hold = rna_hold_3;
          rnaSetpoint = rna_temp_3;
          rna_temp = rna_temp_3;
        }
        flag_rev_step_count = 1;
      }

      if (temp < (rna_temp - 2)) {
        trend = "heat...";
        digitalWrite(chan1, HIGH);
        digitalWrite(chan2, HIGH);
        delay(50);
        analogWrite(peltier, 255);
        LED(1, 0, 0);

        if (flag_step == 1) {
          Step_Time();
        }
      } else if (temp > (rna_temp + 2)) {
        trend = "cooling";
        digitalWrite(chan1, LOW);
        digitalWrite(chan2, LOW);
        delay(50);
        analogWrite(peltier, 255);
        LED(0, 0, 1);

        if (flag_step == 1) {
          Step_Time();
        }
      } else {
        trend = "hold...";
        analogWrite(peltier, 0);
        delay(50);
        digitalWrite(chan1, HIGH);
        digitalWrite(chan2, HIGH);
        delay(50);
        LED(0, 1, 0);

        if (rna_step == 1) {
          if (num_steps_rna == 1) {
            rem_sec = rna_hold_1;
          } else if (num_steps_rna == 2) {
            rem_sec = rna_hold_1 + rna_hold_2 + abs((rna_temp_1 - rna_temp_2));
          } else if (num_steps_rna == 3) {
            rem_sec = rna_hold_1 + rna_hold_2 + rna_hold_3 + abs((rna_temp_2 - rna_temp_1)) + abs((rna_temp_3 - rna_temp_2));
          }
          rem_hour = rem_sec / 3600;
          rem_min = (rem_sec % 3600) / 60;
          rem_sec_serial = rem_sec % 60;
        }

        if (rna_step == 2) {
          if (num_steps_rna == 2) {
            rem_sec = rna_hold_2;
          } else if (num_steps_rna == 3) {
            rem_sec = rna_hold_2 + rna_hold_3 + abs((rna_temp_3 - rna_temp_2));
          }
          rem_hour = rem_sec / 3600;
          rem_min = (rem_sec % 3600) / 60;
          rem_sec_serial = rem_sec % 60;
        }

        if (rna_step == 3) {
          rem_sec = rna_hold_3;
          rem_hour = rem_sec / 3600;
          rem_min = (rem_sec % 3600) / 60;
          rem_sec_serial = rem_sec % 60;
        }

        step_rem = rna_hold;
        flag_read = 1;
        digitalWrite(PCR_output, HIGH);
        delay(60);
        Serial.write(type);
        delay(60);
        Serial.write(rna_step);
        delay(60);
        Serial.write(rem_hour);
        delay(60);
        Serial.write(rem_min);
        delay(60);
        Serial.write(rem_sec_serial);
        delay(60);
        digitalWrite(PCR_output, LOW);
        flag_read = 0;

        while ((count_hold < (rna_hold * 2)) && (temp < emer_temp)) {
          Temp_Read();

          if ((rna_temp > 55) || (rna_temp == 55)) {
            if (flag_pid == 0) {
              analogWrite(peltier, 0);
              delay(50);
              digitalWrite(chan1, HIGH);
              digitalWrite(chan2, HIGH);
              delay(50);
              rnaPID.SetControllerDirection(DIRECT);
              flag_pid = 1;
            }
            rnaInput = temp;
            rnaPID.Compute();
            analogWrite(peltier, rnaOutput);
            //delay(250);
          } else {
            if (temp > (rna_temp + 0.5)) {
              flag_pid_2 = 0;
              if (flag_pid == 0) {
                analogWrite(peltier, 0);
                delay(50);
                digitalWrite(chan1, LOW);
                digitalWrite(chan2, LOW);
                delay(50);
                rnaPID.SetControllerDirection(REVERSE);
                flag_pid = 1;
              }
              rnaInput = temp;
              rnaPID.Compute();
              analogWrite(peltier, rnaOutput);
              //delay(250);
            } else if (temp < (rna_temp - 0.5)) {
              flag_pid = 0;
              if (flag_pid_2 == 0) {
                analogWrite(peltier, 0);
                delay(50);
                digitalWrite(chan1, HIGH);
                digitalWrite(chan2, HIGH);
                delay(50);
                rnaPID.SetControllerDirection(DIRECT);
                flag_pid_2 = 1;
              }
              rnaInput = temp;
              rnaPID.Compute();
              analogWrite(peltier, rnaOutput);
              //delay(250);
            } else {
              rnaInput = temp;
              rnaPID.Compute();
              analogWrite(peltier, rnaOutput);
              //delay(250);
            }
          }
          Lid_Read();
          Lid_Pid();
          if (flag == 1) {
            Dispfnc();
            //count_hold = count_hold + 1;
            hold_func();
            LED(0, 1, 0);
          }
          if (flag_btn == 0) {
            pcr_step = end_cycle;
            break;
          }
        }
        flag_pid = 0;
        flag_pid_2 = 0;
        flag_rev_step_count = 0;
        analogWrite(peltier, 0);
        delay(50);
        if (rna_step == num_steps_rna) {
          pcr_step = end_cycle;
        } else {
          rna_step = rna_step + 1;
        }
        TimerReset();
      }
    }

    else if (pcr_step == pcr) {
      flag_read = 1;
      SPI.begin();
      delay(100);
      while (pcr_step == pcr) {
        Lid_Read();
        if (Serial1.available() > 0) {
          inc_data = Read_data();
        }
        if (step_count == 0) {
          num_samples = inc_data;
          step_count = step_count + 1;
        } else if (step_count == 1) {
          disease_code = inc_data;
          step_count = step_count + 1;
        } else if (step_count == 2) {
          if (inc_data == 254) {
            step_count = step_count + 1;
          } else {
            pcr_step = idle;
            flag_read = 0;
          }
        } else if (step_count == 3) {
          int tab_counter = 0;
          File dataFile = SD.open("kalds.txt");
          int currentLine = 0;
          if (dataFile) {
            while (currentLine < (disease_code - 1)) {
              if (dataFile.available()) {
                char character = dataFile.read();
                if (character == '\n') {  // Check for newline character
                  currentLine++;
                }
              }
            }
            while (dataFile.available() && (tab_counter < 29)) {
              if (tab_counter == 0) {
                character = dataFile.readStringUntil('\t');
                activ_temp = character.toInt();
                tab_counter++;
              } else if (tab_counter == 1) {
                character = dataFile.readStringUntil('\t');
                activ_hr = character.toInt();
                tab_counter++;
              } else if (tab_counter == 2) {
                character = dataFile.readStringUntil('\t');
                activ_min = character.toInt();
                tab_counter++;
              } else if (tab_counter == 3) {
                character = dataFile.readStringUntil('\t');
                activ_sec = character.toInt();
                hold_activ = activ_hr * 3600 + activ_min * 60 + activ_sec;
                tab_counter++;
              } else if (tab_counter == 4) {
                character = dataFile.readStringUntil('\t');
                num_steps = character.toInt();
                tab_counter++;
              } else if (tab_counter == 5) {
                character = dataFile.readStringUntil('\t');
                num_cycles = character.toInt();
                tab_counter++;
              } else if (tab_counter == 6) {
                character = dataFile.readStringUntil('\t');
                init_temp = character.toInt();
                tab_counter++;
              } else if (tab_counter == 7) {
                character = dataFile.readStringUntil('\t');
                init_hr = character.toInt();
                tab_counter++;
              } else if (tab_counter == 8) {
                character = dataFile.readStringUntil('\t');
                init_min = character.toInt();
                tab_counter++;
              } else if (tab_counter == 9) {
                character = dataFile.readStringUntil('\t');
                init_sec = character.toInt();
                hold_init = init_hr * 3600 + init_min * 60 + init_sec;
                tab_counter++;
              } else if (tab_counter == 10) {
                character = dataFile.readStringUntil('\t');
                denat_temp = character.toInt();
                tab_counter++;
              } else if (tab_counter == 11) {
                character = dataFile.readStringUntil('\t');
                denat_hr = character.toInt();
                tab_counter++;
              } else if (tab_counter == 12) {
                character = dataFile.readStringUntil('\t');
                denat_min = character.toInt();
                tab_counter++;
              } else if (tab_counter == 13) {
                character = dataFile.readStringUntil('\t');
                denat_sec = character.toInt();
                hold_denat = denat_hr * 3600 + denat_min * 60 + denat_sec;
                tab_counter++;
              } else if (tab_counter == 14) {
                if (num_steps == 2) {
                  character = dataFile.readStringUntil('\t');
                  ext_temp = character.toInt();
                  tab_counter = 19;
                } else if (num_steps == 3) {
                  character = dataFile.readStringUntil('\t');
                  anneal_temp = character.toInt();
                  tab_counter++;
                }
              } else if (tab_counter == 15) {
                character = dataFile.readStringUntil('\t');
                anneal_hr = character.toInt();
                tab_counter++;
              } else if (tab_counter == 16) {
                character = dataFile.readStringUntil('\t');
                anneal_min = character.toInt();
                tab_counter++;
              } else if (tab_counter == 17) {
                character = dataFile.readStringUntil('\t');
                anneal_sec = character.toInt();
                hold_anneal = anneal_hr * 3600 + anneal_min * 60 + anneal_sec;
                tab_counter++;
              } else if (tab_counter == 18) {
                character = dataFile.readStringUntil('\t');
                ext_temp = character.toInt();
                tab_counter++;
              } else if (tab_counter == 19) {
                character = dataFile.readStringUntil('\t');
                ext_hr = character.toInt();
                tab_counter++;
              } else if (tab_counter == 20) {
                character = dataFile.readStringUntil('\t');
                ext_min = character.toInt();
                tab_counter++;
              } else if (tab_counter == 21) {
                character = dataFile.readStringUntil('\t');
                ext_sec = character.toInt();
                hold_ext = ext_hr * 3600 + ext_min * 60 + ext_sec;
                tab_counter++;
              } else if (tab_counter == 22) {
                character = dataFile.readStringUntil('\t');
                final_temp = character.toInt();
                tab_counter++;
              } else if (tab_counter == 23) {
                character = dataFile.readStringUntil('\t');
                final_hr = character.toInt();
                tab_counter++;
              } else if (tab_counter == 24) {
                character = dataFile.readStringUntil('\t');
                final_min = character.toInt();
                tab_counter++;
              } else if (tab_counter == 25) {
                character = dataFile.readStringUntil('\t');
                final_sec = character.toInt();
                hold_final = final_hr * 3600 + final_min * 60 + final_sec;
                tab_counter++;
              } else if (tab_counter == 26) {
                character = dataFile.readStringUntil('\t');
                animal = character;
                tab_counter++;
              } else if (tab_counter == 27) {
                character = dataFile.readStringUntil('\t');
                disease = character;
                tab_counter++;
              } else if (tab_counter == 28) {
                character = dataFile.readStringUntil('\t');
                if (character.toInt() == 254) {
                  pcr_step = lid_heat;
                  type = 4;
                  Rem_Time();
                  rem_sec = estim_sec;
                  denatSetpoint = denat_temp;
                  annealSetpoint = anneal_temp;
                  extSetpoint = ext_temp;
                  rnaSetpoint = activ_temp;
                  flag_read = 0;
                  tab_counter++;
                  step_count = step_count + 1;
                  SPI.end();
                } else {
                  pcr_step = idle;
                  flag_read = 0;
                  tab_counter++;
                  step_count = step_count + 1;
                  SPI.end();
                }
              }
            }
          } else {
            //Serial.println("no datafile");
            pcr_step = idle;
            flag_read = 0;
            SPI.end();
            SD.end();
            delay(50);
            SD.begin(chipSelect);
          }
        }
      }
    }


    else {  // case of overheating
      pcr_step = emergency;
      while (1) {
        analogWrite(lid, 0);
        analogWrite(peltier, 0);
        digitalWrite(chan1, LOW);
        digitalWrite(chan2, LOW);

        if (flag == 1) {
          Count_LED();
        }
      }
    }
  }

  else if (pcr_step == end_cycle) {  // end of cycles

    flag_end = 1;
    count_time = 0;
    count_step = 0;
    analogWrite(peltier, 0);
    delay(50);
    digitalWrite(chan1, LOW);
    digitalWrite(chan2, LOW);
    analogWrite(lid, 0);
    if (buzz_count == 0) {
      flag_read = 1;
      digitalWrite(PCR_output, LOW);
      digitalWrite(end_reaction, HIGH);
      digitalWrite(ON_output, HIGH);
      delay(60);
      RTC.read(tm);
      delay(60);
      Serial.write(tm.Month);
      delay(60);
      Serial.write(tm.Day);
      delay(60);
      Serial.write(((tmYearToCalendar(tm.Year)) % 100));
      delay(60);
      Serial.write(tm.Hour);
      delay(60);
      Serial.write(tm.Minute);
      delay(60);

      Serial1.print("page 1");
      //delay(50);
      Serial1.write(0xFF);
      //delay(50);
      Serial1.write(0xFF);
      //delay(50);
      Serial1.write(0xFF);
      //delay(50);

      flag_read = 0;
    }
    if (flag == 1) {
      Count_LED();
    }
    if (buzz_count >= 14) {
      pcr_step = idle;
      cycle = 0;
    }
  }

  else {  // end of cycles
    pcr_step = end_cycle;
    flag_end = 1;
    count_time = 0;
    count_step = 0;
    analogWrite(peltier, 0);
    delay(50);
    digitalWrite(chan1, LOW);
    digitalWrite(chan2, LOW);
    analogWrite(lid, 0);
  }
}

void Dispfnc() {  //  lcd display function
  Serial1.print("page36.status.val=");
  //delay(50);
  Serial1.print(pcr_step);
  //delay(50);
  Serial1.write(0xFF);
  //delay(50);
  Serial1.write(0xFF);
  //delay(50);
  Serial1.write(0xFF);
  //delay(50);
  Serial1.print("page36.lid.val=");
  //delay(50);
  Serial1.print(temp_lid);
  //delay(50);
  Serial1.write(0xFF);
  //delay(50);
  Serial1.write(0xFF);
  //delay(50);
  Serial1.write(0xFF);
  //delay(50);
  Serial1.print("page36.well.val=");
  //delay(50);
  Serial1.print(temp);
  //delay(50);
  Serial1.write(0xFF);
  //delay(50);
  Serial1.write(0xFF);
  //delay(50);
  Serial1.write(0xFF);
  //delay(50);
  Serial1.print("page36.cycle.val=");
  //delay(50);
  Serial1.print(cycle);
  //delay(50);
  Serial1.write(0xFF);
  //delay(50);
  Serial1.write(0xFF);
  //delay(50);
  Serial1.write(0xFF);
  //delay(50);

  Serial1.print("page36.totcyc.val=");
  //delay(50);
  Serial1.print(num_cycles);
  //delay(50);
  Serial1.write(0xFF);
  //delay(50);
  Serial1.write(0xFF);
  //delay(50);
  Serial1.write(0xFF);
  //delay(50);

  Serial1.print("page36.s.val=");
  //delay(50);
  Serial1.print(rem_sec_serial);
  //delay(50);
  Serial1.write(0xFF);
  //delay(50);
  Serial1.write(0xFF);
  //delay(50);
  Serial1.write(0xFF);
  //delay(50);
  Serial1.print("page36.m.val=");
  //delay(50);
  Serial1.print(rem_min);
  //delay(50);
  Serial1.write(0xFF);
  //delay(50);
  Serial1.write(0xFF);
  //delay(50);
  Serial1.write(0xFF);
  //delay(50);
  Serial1.print("page36.h.val=");
  //delay(50);
  Serial1.print(rem_hour);
  //delay(50);
  Serial1.write(0xFF);
  //delay(50);
  Serial1.write(0xFF);
  //delay(50);
  Serial1.write(0xFF);
  //delay(50);
  Serial1.print("page36.j0.val=");
  Serial1.print(percentage_elap);
  //delay(50);
  Serial1.write(0xFF);
  //delay(50);
  Serial1.write(0xFF);
  //delay(50);
  Serial1.write(0xFF);
  //delay(50);
}

void Count_Timer() {  // run time and remaing time counter
  if (flag_end == 0) {
    if ((pcr_step == idle) || (pcr_step == set) || (pcr_step == rna_set) || (pcr_step == select) || (pcr_step == extract_set) || (pcr_step == pcr)) {
      sec = 0;
      count_time = 0;
    } else {
      sec = sec + 1;
      if (pause == 0) {
        rem_sec = rem_sec - 1;
        step_rem = step_rem - 1;
      }
      if ((rem_sec == 4294967295) || (rem_sec == 4294967294)) {
        rem_sec = 0;
      }
      if ((step_rem == 4294967295) || (step_rem == 4294967294)) {
        step_rem = 0;
      }
      count_time = 0;
      rem_hour = rem_sec / 3600;
      rem_min = (rem_sec % 3600) / 60;
      rem_sec_serial = rem_sec % 60;
      //percentage_elap = ((estim_sec - rem_sec) * 100) / estim_sec;
      percentage_elap = ((estim_sec - rem_sec) * 100) / estim_sec;
    }
  }
}

void Temp_Read() {  // well temperature reader
  temp = thermocouple.readCelsius() + 1;
  delay(250);
}

void Lid_Read() {  // lid temperature reader
  raw2 = analogRead(Res_Pin);
  if (raw2) {
    buffer2 = raw2 * Vin;
    Vout = (buffer2) / 1024.0;
    buffer2 = (Vin / Vout) - 1;
    R2_Res = R1_Res * buffer2;
    temp_lid = -35.97 * log(R2_Res) + 353.14;
  }
}

void Step_Time() {  // step run time counter
  avg_sec = avg_sec + 1;
  //  if ((type == 1) || (type == 3)) {
  //    rem_sec = rem_sec + 1;
  //  }
  count_step = 0;
  flag_step = 0;
}

void Avg_Time() {  // average cycle time calculator
  if (cycle == 1) {
  } else if (cycle == 2) {
    avg_sec_mem = avg_sec + hold_denat + hold_anneal + hold_ext;
  } else {
    avg_sec_mem = (avg_sec + hold_denat + hold_anneal + hold_ext) * 0.8 + avg_sec_mem * 0.2;
  }
  avg_sec = 0;
}

void Estim_Time() {  // Estimated total run time calculator
  if (cycle >= 3)
    estim_sec = avg_sec_mem * num_cycles;
}

void Rem_Time() {  //  Remaining run time calculator
  if ((type == 2) || (type == 4)) {
    if (cycle == 0) {
      if (num_steps == 2) {
        if (hold_activ > 0)
          estim_sec = abs((lid_temp - temp_lid) / heat_rate) + abs((activ_temp - temp) / heat_rate) + hold_activ + abs((init_temp - activ_temp) / heat_rate) + hold_init + abs((denat_temp - init_temp) / heat_rate) + (hold_denat + hold_ext + 2 * abs((denat_temp - ext_temp) / heat_rate)) * (num_cycles - cycle) + hold_final;
        else
          estim_sec = abs((lid_temp - temp_lid / heat_rate)) + abs((init_temp - temp) / heat_rate) + hold_init + abs((denat_temp - init_temp) / heat_rate) + (hold_denat + hold_ext + 2 * abs((denat_temp - ext_temp) / heat_rate)) * (num_cycles - cycle) + hold_final;
      } else if (num_steps == 3) {
        if (hold_activ > 0)
          estim_sec = abs((lid_temp - temp_lid) / heat_rate) + abs((activ_temp - temp) / heat_rate) + hold_activ + abs((init_temp - activ_temp) / heat_rate) + hold_init + abs((denat_temp - init_temp) / heat_rate) + (hold_denat + hold_anneal + hold_ext + abs((denat_temp - anneal_temp) / heat_rate) + abs((ext_temp - anneal_temp) / heat_rate) + abs((denat_temp - ext_temp) / heat_rate)) * (num_cycles - cycle) + hold_final;
        else
          estim_sec = abs((lid_temp - temp_lid) / heat_rate) + abs((init_temp - temp) / heat_rate) + hold_init + abs((denat_temp - init_temp) / heat_rate) + (hold_denat + hold_anneal + hold_ext + abs((denat_temp - anneal_temp) / heat_rate) + abs((ext_temp - anneal_temp) / heat_rate) + abs((denat_temp - ext_temp) / heat_rate)) * (num_cycles - cycle) + hold_final;
      }
    } else if (cycle >= 3)
      rem_sec = avg_sec_mem * (num_cycles - cycle) + hold_final;
  } else if (type == 1) {
    if (num_steps_rna == 1) {
      estim_sec = abs((lid_temp - temp_lid)) + abs((rna_temp_1 - temp) / heat_rate) + rna_hold_1;
    } else if (num_steps_rna == 2) {
      estim_sec = abs((lid_temp - temp_lid)) + abs((rna_temp_1 - temp) / heat_rate) + rna_hold_1 + (abs((rna_temp_2 - rna_temp_1) / heat_rate) + rna_hold_2);
    } else if (num_steps_rna == 3) {
      estim_sec = abs((lid_temp - temp_lid)) + abs((rna_temp_1 - temp) / heat_rate) + rna_hold_1 + (abs((rna_temp_2 - rna_temp_1) / heat_rate) + rna_hold_2) + (abs((rna_temp_3 - rna_temp_2) / heat_rate) + rna_hold_3);
    }
  } else if (type == 3) {
    estim_sec = abs((lid_temp - temp_lid) / heat_rate) + abs((rna_extract_temp - temp) / heat_rate) + rna_extract_hold;
  }
}

void LED(bool red, bool yel, bool blue) {  //  LEDs light control function
  digitalWrite(LED_RED, red);
  digitalWrite(LED_YEL, yel);
  digitalWrite(LED_BLU, blue);
}

void TimerReset() {  // reset counters at end of cycle
  count_hold = 0;
  flag_step = 0;
  count_step = 0;
  flag_rate = 0;
  rate_count = 0;
  step_rem = 0;
}

void Count_LED() {  // LED light blinking function
  flag = 0;
  if (pcr_step == emergency) {
    digitalWrite(LED_BLU, LOW);
    digitalWrite(LED_YEL, LOW);
    digitalWrite(LED_RED, emer_blink);
    emer_blink = !emer_blink;
    if (buzz_count < 14) {
      digitalWrite(buzzer, buzz);
      buzz = !buzz;
      buzz_count = buzz_count + 1;
    } else {
      digitalWrite(buzzer, LOW);
    }
  } else if (flag_end == 1) {
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_YEL, LOW);
    digitalWrite(LED_BLU, emer_blink);
    emer_blink = !emer_blink;
    if (buzz_count < 14) {
      digitalWrite(buzzer, buzz);
      buzz = !buzz;
      buzz_count = buzz_count + 1;
    } else {
      digitalWrite(buzzer, LOW);
    }
  }
}

void Flag_Check() {  // push button read

  count_status = count_status + 1;
  //send status
  if (count_status == (60 * timer_mult)) {
    flag_time = 1;
    count_status = 0;
  }

  if (flag_read == 0) {
    if ((pcr_step == idle) && (flag_time == 1)) {
      Serial1.print("page1.n0.val=");
      //delay(50);
      Serial1.print(tm.Month);
      //delay(50);
      Serial1.write(0xFF);
      //delay(50);
      Serial1.write(0xFF);
      //delay(50);
      Serial1.write(0xFF);
      //delay(50);
      Serial1.print("page1.n1.val=");
      //delay(50);
      Serial1.print(tm.Day);
      //delay(50);
      Serial1.write(0xFF);
      //delay(50);
      Serial1.write(0xFF);
      //delay(50);
      Serial1.write(0xFF);
      //delay(50);
      Serial1.print("page1.n2.val=");
      //delay(50);
      Serial1.print((tmYearToCalendar(tm.Year)) % 100);
      //delay(50);
      Serial1.write(0xFF);
      //delay(50);
      Serial1.write(0xFF);
      //delay(50);
      Serial1.write(0xFF);
      //delay(50);
      Serial1.print("page1.n3.val=");
      //delay(50);
      Serial1.print(tm.Hour);
      //delay(50);
      Serial1.write(0xFF);
      //delay(50);
      Serial1.write(0xFF);
      //delay(50);
      Serial1.write(0xFF);
      //delay(50);
      Serial1.print("page1.n4.val=");
      //delay(50);
      Serial1.print(tm.Minute);
      //delay(50);
      Serial1.write(0xFF);
      //delay(50);
      Serial1.write(0xFF);
      //delay(50);
      Serial1.write(0xFF);
      //delay(50);
    }

    Serial1.print("page1.status.val=");
    //delay(50);
    Serial1.print(pcr_step);
    //delay(50);
    Serial1.write(0xFF);
    //delay(50);
    Serial1.write(0xFF);
    //delay(50);
    Serial1.write(0xFF);
    //delay(50);
    Serial1.print("page1.lid.val=");
    //delay(50);
    Serial1.print(temp_lid);
    //delay(50);
    Serial1.write(0xFF);
    //delay(50);
    Serial1.write(0xFF);
    //delay(50);
    Serial1.write(0xFF);
    //delay(50);
    Serial1.print("page1.well.val=");
    //delay(50);
    Serial1.print(temp);
    //delay(50);
    Serial1.write(0xFF);
    //delay(50);
    Serial1.write(0xFF);
    //delay(50);
    Serial1.write(0xFF);
    //delay(50);
  }


  if (flag_read == 0) {
    if (Serial1.available() > 0) {
      inc_data = Read_data();
      if (inc_data == 101) {
        pcr_step = set;
      } else if (inc_data == 102) {
        pcr_step = pcr;
      } else if (inc_data == 103) {
        pcr_step = rna_set;
      } else if (inc_data == 104) {
        pcr_step = extract_set;
      } else if (inc_data == 105) {
        flag_btn = 0;
        pcr_step = end_cycle;
      } else if (inc_data == 106) {
        qPCR = true;
      } else if (inc_data == 107) {
        flag_pause = true;
      } else if (inc_data == 108) {
        flag_pause = false;
      } else if (inc_data == 109) {
        flag_lcd_init = 1;
      }
    }
  }

  if ((pcr_step == extract) || (pcr_step == end_cycle) || (pcr_step == reverse) || (pcr_step == denat) || (pcr_step == anneal) || (pcr_step == ext) || (pcr_step == activation) || (pcr_step == start)) {
    Dispfnc();
  }

  if (status == 14) {
    //  fluo_check();
  }
}

void Lid_Pid() {
  if (temp_lid < (lid_temp - 2)) {
    analogWrite(lid, 255);
  } else {
    lidInput = temp_lid;
    lidPID.Compute();
    analogWrite(lid, lidOutput);
  }
}

int Read_data() {
  int data = Serial1.read();
  Serial1.read();
  Serial1.read();
  Serial1.read();
  delay(50);
  return data;
}

void hold_func() {
  count_hold = count_hold + 1;
  LED(0, 1, 0);
  flag = 0;
}
