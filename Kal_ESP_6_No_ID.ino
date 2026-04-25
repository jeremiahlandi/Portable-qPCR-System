/*
   Copyright (c) 2015, Majenko Technologies
   All rights reserved.

   Redistribution and use in source and binary forms, with or without modification,
   are permitted provided that the following conditions are met:

 * * Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.

 * * Redistributions in binary form must reproduce the above copyright notice, this
     list of conditions and the following disclaimer in the documentation and/or
     other materials provided with the distribution.

 * * Neither the name of Majenko Technologies nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
   ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
   ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Create a WiFi access point and provide a web server on it. */

#include <Adafruit_AS7341.h>
#include <Wire.h>
//#include <Adafruit_GFX.h>
#include <math.h>
#include "SPI.h"
#include "SD.h"
#include "FS.h"
#include "BluetoothSerial.h"

//#include "ty_model.h"


//#include <TensorFlowLite_ESP32.h>
//#include "tensorflow/lite/micro/kernels/micro_ops.h"
//#include "tensorflow/lite/micro/micro_error_reporter.h"
//#include "tensorflow/lite/micro/micro_interpreter.h"
//#include "tensorflow/lite/micro/all_ops_resolver.h"
//
//#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"

//#define USE_PIN // Uncomment this to use PIN during pairing. The pin is specified on the line below
const char *pin = "1234";  // Change this to more secure PIN.

String device_name = "ESP32-KX";

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;


// Figure out what's going on in our model
//#define DEBUG 1

#ifndef APSSID
#define APSSID "ESPap"
#define APPSK "thereisnospoon"
#endif

//SPIClass hspi(HSPI);

/* Set these to your desired credentials. */
const char *ssid = APSSID;
const char *password = APPSK;

const int chipSelect = 5;  //SD Card

//ESP8266WebServer server(80);

Adafruit_AS7341 as7341;

int m1[100];
int m2[100];
int m3[100];
int m4[100];
int m5[100];
int m6[100];
int m7[100];
int m8[100];

int c1 = 0;
int c2 = 0;
int c3 = 0;
int c4 = 0;
int c5 = 0;
int c6 = 0;
int c7 = 0;
int c8 = 0;
int count = 0;

int fluo_result = 0;

int flag_qPCR = 1;
int flag_qPCR_2 = 1;

int key_temp = 0;

int fluo_count = 0;
int flag_nc = 0;
int flag_set = 0;
int flag_BT_0 = 0;
int flag_BT_1 = 0;
int flag_BT_2 = 0;
int set_indx = 0;
int num_set_total = 0;
int count_set = 0;
int flag_read = 1;
int flag_fluo = 0;
char mega_var = 0;
//char BT_temp_out;
char BT_temp;
int ext_temp;
unsigned long ext_time_hr;
unsigned long ext_time_min;
unsigned long ext_time_sec;
int rev_1_temp;
unsigned long rev_1_hr;
unsigned long rev_1_min;
unsigned long rev_1_sec;
int rev_2_temp;
unsigned long rev_2_hr;
unsigned long rev_2_min;
unsigned long rev_2_sec;
int rev_3_temp;
unsigned long rev_3_hr;
unsigned long rev_3_min;
unsigned long rev_3_sec;
int rev_total_steps;
int rev_current_step;
int pcr_steps;
int num_samples = 0;
int num_nc = 0;

int counter = 0;
int activ_temp;
unsigned long activ_hr;
unsigned long activ_min;
unsigned long activ_sec;
int init_temp;
unsigned long init_hr;
unsigned long init_min;
unsigned long init_sec;
int denat_temp;
unsigned long denat_hr;
unsigned long denat_min;
unsigned long denat_sec;
int anneal_temp;
unsigned long anneal_hr;
unsigned long anneal_min;
unsigned long anneal_sec;
int exten_temp;
unsigned long exten_hr;
unsigned long exten_min;
unsigned long exten_sec;
int final_temp;
unsigned long final_hr;
unsigned long final_min;
unsigned long final_sec;

int num_set[16];
int num_steps;
int num_cycles;
int cycle;

int rna_extract_temp;
unsigned long rna_extract_hr;
unsigned long rna_extract_min;
unsigned long rna_extract_sec;

int BT_status;
int BT_flag = 0;
int rubish = 0;

int flag_update = 0;
int flag_inter_start = 0;
int flag_inter_end = 0;
int flag_while = 0;
int flag_while_update = 0;
int flag_upfnc = 0;
int flag_end = 0;

int sum = 0;
int set_counter;
int type = 0;


// I/O Pins
int qPCR = 12;       //fluo read btn
int PCR_input = 13;  //reaction begin flag
//int Analysis = 19;;//analysis btn
int end_reaction = 27;  //end reaction flag
//int Fluo_set = 27;
int ON_input = 14;  //reaction in progress flag
int esp_mega = 25;
//int input3 = 13;
int Output_mega = 26;  //I/O pin
//int Light_flag = 34;//I/O pin
int light_1 = 4;
int light_2 = 32;

int fluo_data[8][16][75];

float fluo_multiplier = 10.4;


unsigned long rem_sec_serial;
unsigned long rem_hour;
unsigned long rem_min;
char rem_time[20];

String animal_IDs[] = {};
String animal_IDs_received[] = {};

unsigned long rem_sec_count;

String animal;
String disease;

int year = 7;
int month = 6;
int day = 5;
int hour = 4;
int minute = 3;

int availableBytes;

int disease_code;

hw_timer_t *timer = NULL;
volatile uint8_t led_state = 0;

File dataFile;
/* Just a little test message.  Go to http://192.168.4.1 in a web browser
   connected to this access point to see it.
*/
//String prepareHtmlPage() {
//  String htmlPage;
//  htmlPage = F( "<!DOCTYPE HTML>"
//                "<html>"
//                "<font face=Helvetica>"
//                "<h1>Optical Sensor Readings</h1>"
//                "<p>"
//                "F1 415nm :  ");
//
//  for (int i = 0; i < count; i++) {
//    htmlPage += m1[i];
//    htmlPage += " ";
//  }
//  htmlPage += "</p><p>F2 445nm :  ";
//  for (int i = 0; i < count; i++) {
//    htmlPage += m2[i];
//    htmlPage += " ";
//  }
//  htmlPage += "</p><p>F3 480nm :  ";
//  for (int i = 0; i < count; i++) {
//    htmlPage += m3[i];
//    htmlPage += " ";
//  }
//  htmlPage += "</p><p>F4 515nm :  ";
//  for (int i = 0; i < count; i++) {
//    htmlPage += m4[i];
//    htmlPage += " ";
//  }
//  htmlPage += "</p><p>F5 555nm :  ";
//  for (int i = 0; i < count; i++) {
//    htmlPage += m5[i];
//    htmlPage += " ";
//  }
//  htmlPage += "</p><p>F6 590nm :  ";
//  for (int i = 0; i < count; i++) {
//    htmlPage += m6[i];
//    htmlPage += " ";
//  }
//  htmlPage += "</p><p>F7 630nm :  ";
//  for (int i = 0; i < count; i++) {
//    htmlPage += m7[i];
//    htmlPage += " ";
//  }
//  htmlPage += "</p><p>F8 680nm :  ";
//  for (int i = 0; i < count; i++) {
//    htmlPage += m8[i];
//    htmlPage += " ";
//  }
//  htmlPage += F("</P></font></html>");
//  return htmlPage;
//}
//
//
//void handleRoot() {
//  server.send(200, "text/html", prepareHtmlPage());
//}

//// TFLite globals, used for compatibility with Arduino-style sketches
//namespace {
//tflite::ErrorReporter* error_reporter = nullptr;
//const tflite::Model* model = nullptr;
//tflite::MicroInterpreter* interpreter = nullptr;
//TfLiteTensor* model_input = nullptr;
//TfLiteTensor* model_output = nullptr;
//
//// Create an area of memory to use for input, output, and other TensorFlow
//// arrays. You'll need to adjust this by combiling, running, and looking
//// for errors.
//constexpr int kTensorArenaSize = 5 * 1024;
////  tensor_pool = (uint8_t*) heap_caps_calloc(tensor_pool_size, 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
//uint8_t tensor_arena[kTensorArenaSize];
//} // namespace

void IRAM_ATTR timer_isr() {
  if ((flag_inter_start == 1) && (flag_inter_end == 1)) {
    flag_inter_start = 0;
    flag_inter_end = 0;
  }
  if ((rem_sec_count > 0)) {
    rem_sec_count = rem_sec_count - 1;
  }
  if (flag_while == 1) {
    flag_while_update = 1;
  }
  //Serial.println(rem_sec_serial);
  flag_upfnc = 1;
}


void setup() {

  Serial.begin(9600);
  //hspi.begin();

  //Serial.print("ESP ok");
  //mySPI.begin();

  uint8_t timer_id = 0;
  uint16_t prescaler = 80;
  int threashold = 1000000;
  timer = timerBegin(timer_id, prescaler, true);
  timerAttachInterrupt(timer, &timer_isr, true);
  timerAlarmWrite(timer, threashold, true);
  timerAlarmEnable(timer);


  // Wait for Serial to connect
  //#if DEBUG
  //  while(!Serial);
  //#endif

  //  // Set up logging (will report to Serial, even within TFLite functions)
  //  static tflite::MicroErrorReporter micro_error_reporter;
  //  error_reporter = &micro_error_reporter;
  //
  //  // Map the model into a usable data structure
  //  model = tflite::GetModel(ty_model);
  //  if (model->version() != TFLITE_SCHEMA_VERSION) {
  //    error_reporter->Report("Model version does not match Schema");
  //    while (1);
  //  }


  //  static tflite::AllOpsResolver resolver;
  //
  //  static tflite::MicroInterpreter static_interpreter(
  //    model, resolver, tensor_arena, kTensorArenaSize,
  //    error_reporter);
  //  interpreter = &static_interpreter;
  //
  //  interpreter = &static_interpreter;
  //
  //  // Allocate memory from the tensor_arena for the model's tensors
  //  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  //  if (allocate_status != kTfLiteOk) {
  //    error_reporter->Report("AllocateTensors() failed");
  //    while (1);
  //  }
  //
  //  // Assign model input and output buffers (tensors) to pointers
  //  model_input = interpreter->input(0);
  //  model_output = interpreter->output(0);
  //
  //      #if DEBUG
  //  Serial.print("Number of dimensions: ");
  //  Serial.println(model_input->dims->size);
  //  Serial.print("Dim 1 size: ");
  //  Serial.println(model_input->dims->data[0]);
  //  Serial.print("Dim 2 size: ");
  //  Serial.println(model_input->dims->data[1]);
  //  Serial.print("Input type: ");
  //  Serial.println(model_input->type);
  //#endif

  //  Serial.println();
  //  Serial.print("Configuring access point...");
  /* You can remove the password parameter if you want the AP to be open. */
  //  WiFi.softAP(ssid, password);
  //
  //  IPAddress myIP = WiFi.softAPIP();
  //  Serial.print("AP IP address: ");
  //  Serial.println(myIP);
  //  server.on("/", handleRoot);
  //  server.begin();
  //  Serial.println("HTTP server started");

  //Spectrophotometer initialization
  as7341.begin(0x39);
  //  Wire.begin(D2, D1);

  //    as7341.setATIME(100);
  //    as7341.setASTEP(999);
  //    as7341.setGain(AS7341_GAIN_256X);

  as7341.setATIME(5500);
  as7341.setASTEP(5000);
  as7341.setGain(AS7341_GAIN_32X);



  //pinMode(Analysis, INPUT); //Analysis input
  pinMode(PCR_input, INPUT);     //PCR input
  pinMode(end_reaction, INPUT);  //Fluo setup input
  pinMode(ON_input, INPUT);      //Fluo setup input
  pinMode(qPCR, INPUT);          //fluo reading input
  pinMode(Output_mega, OUTPUT);
  pinMode(chipSelect, OUTPUT);
  pinMode(light_1, OUTPUT);
  pinMode(light_2, OUTPUT);
  pinMode(33, OUTPUT);
  //pinMode(Light_flag, INPUT); //fluo reading input
  digitalWrite(Output_mega, LOW);
  digitalWrite(33, HIGH);

  SerialBT.register_callback(Bt_Status);

  SerialBT.begin(device_name);  //Bluetooth device name
                                //Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());

  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    //while (1);
  }
}

void loop() {
  set_counter = 0;
  sum = 0;
  update_func();
  //  ext_temp = 0;
  //  ext_time_hr = 0;
  //  ext_time_min = 0;
  //  ext_time_sec = 0;



  //  if (Serial.available() > 0){
  //    Serial.read();
  //  }

  //Serial.println(BT_temp);
  // delay(20);

  //  if (BT_status == 1)
  //    BT_flag = 1;
  //
  //  if ((BT_status == 0) && (BT_flag == 1)){
  //    SerialBT.end();
  //    delay(10);
  //    BT_flag = 0;
  //    SerialBT.begin(device_name);
  //  }

  //  if ((digitalRead(ON_input) == 1) && (digitalRead(Fluo_set) == 0) && (BT_status == 1) && (flag_BT_1 == 0)) {
  //    SerialBT.write(1);
  //    flag_BT_1 = 1;
  //  }
  //  else
  //    flag_BT_1 = 0;
  //
  //  if ((digitalRead(ON_input) == 0) && (BT_status == 1) && (flag_BT_0 == 0)) {
  //    SerialBT.write(0);
  //    flag_BT_0 = 1;
  //  }
  //  else
  //    flag_BT_0 = 0;
  //
  //  if ((digitalRead(ON_input) == 1) && (digitalRead(Fluo_set) == 1) && (BT_status == 1) && (flag_BT_2 == 0)) {
  //    SerialBT.write(2);
  //    flag_BT_2 = 1;
  //  }
  //  else
  //    flag_BT_2 = 0;

  if (SerialBT.available()) {
    if ((digitalRead(ON_input) == 0) && (digitalRead(PCR_input) == 0) && (digitalRead(end_reaction) == 0)) {
      count = 0;
      BT_temp = SerialBT.read();
      //String mp = SerialBT.readString();

      if (BT_temp == 123) {
        while ((BT_temp != 127) && (BT_temp != 126) && (digitalRead(end_reaction) == 0)) {
          flag_while = 1;
          if (flag_while_update == 1) {
            SerialBT.write(127);
            flag_while_update = 0;
          }

          if (SerialBT.available()) {
            BT_temp = SerialBT.read();
            if (count == 0) {
              ext_temp = BT_temp;
              count = count + 1;
            } else if (count == 1) {
              ext_time_hr = BT_temp;
              count = count + 1;
            } else if (count == 2) {
              ext_time_min = BT_temp;
              count = count + 1;
            } else if (count == 3) {
              ext_time_sec = BT_temp;
              count = count + 1;
            }
          }
        }
        flag_while = 0;
        if (BT_temp == 127) {
          digitalWrite(Output_mega, HIGH);
          delay(100);
          Serial.write(1);
          delay(50);
          Serial.write(ext_temp);
          delay(50);
          Serial.write(ext_time_hr);
          delay(50);
          Serial.write(ext_time_min);
          delay(50);
          Serial.write(ext_time_sec);
          delay(50);
          // animal = " Extract.";
          // disease = " ";
          // Serial.write(animal.c_str());
          // delay(100);
          // Serial.write('\0');
          // delay(50);
          // Serial.write(disease.c_str());
          // delay(100);
          // Serial.write('\0');
          // delay(50);
          Serial.write(127);
          delay(50);
          digitalWrite(Output_mega, LOW);
        }
      }

      else if (BT_temp == 124) {

        while ((BT_temp != 127) && (BT_temp != 126)) {
          flag_while = 1;
          if (flag_while_update == 1) {
            SerialBT.write(127);
            flag_while_update = 0;
          }

          if (SerialBT.available()) {
            BT_temp = SerialBT.read();
            if (count == 0) {
              rev_total_steps = BT_temp;
              count = count + 1;
            } else if (count == 1) {
              rev_1_temp = BT_temp;
              count = count + 1;
            } else if (count == 2) {
              rev_1_hr = BT_temp;
              count = count + 1;
            } else if (count == 3) {
              rev_1_min = BT_temp;
              count = count + 1;
            } else if (count == 4) {
              rev_1_sec = BT_temp;
              count = count + 1;
            } else if (count == 5) {
              rev_2_temp = BT_temp;
              count = count + 1;
            } else if (count == 6) {
              rev_2_hr = BT_temp;
              count = count + 1;
            } else if (count == 7) {
              rev_2_min = BT_temp;
              count = count + 1;
            } else if (count == 8) {
              rev_2_sec = BT_temp;
              count = count + 1;
            } else if (count == 9) {
              rev_3_temp = BT_temp;
              count = count + 1;
            } else if (count == 10) {
              rev_3_hr = BT_temp;
              count = count + 1;
            } else if (count == 11) {
              rev_3_min = BT_temp;
              count = count + 1;
            } else if (count == 12) {
              rev_3_sec = BT_temp;
              count = count + 1;
            }
          }
        }
        flag_while = 0;

        if (BT_temp == 127) {
          digitalWrite(Output_mega, HIGH);
          delay(100);
          Serial.write(2);
          delay(50);
          if (rev_total_steps == 1) {
            Serial.write(rev_total_steps);
            delay(50);
            Serial.write(rev_1_temp);
            delay(50);
            Serial.write(rev_1_hr);
            delay(50);
            Serial.write(rev_1_min);
            delay(50);
            Serial.write(rev_1_sec);
            delay(50);
          } else if (rev_total_steps == 2) {
            Serial.write(rev_total_steps);
            delay(50);
            Serial.write(rev_1_temp);
            delay(50);
            Serial.write(rev_1_hr);
            delay(50);
            Serial.write(rev_1_min);
            delay(50);
            Serial.write(rev_1_sec);
            delay(50);
            Serial.write(rev_2_temp);
            delay(50);
            Serial.write(rev_2_hr);
            delay(50);
            Serial.write(rev_2_min);
            delay(50);
            Serial.write(rev_2_sec);
            delay(50);
          } else if (rev_total_steps == 3) {
            Serial.write(rev_total_steps);
            delay(50);
            Serial.write(rev_1_temp);
            delay(50);
            Serial.write(rev_1_hr);
            delay(50);
            Serial.write(rev_1_min);
            delay(50);
            Serial.write(rev_1_sec);
            delay(50);
            Serial.write(rev_2_temp);
            delay(50);
            Serial.write(rev_2_hr);
            delay(50);
            Serial.write(rev_2_min);
            delay(50);
            Serial.write(rev_2_sec);
            delay(50);
            Serial.write(rev_3_temp);
            delay(50);
            Serial.write(rev_3_hr);
            delay(50);
            Serial.write(rev_3_min);
            delay(50);
            Serial.write(rev_3_sec);
            delay(50);
          }
          // animal = " Rev. Tran";
          // disease = " ";
          // Serial.write(animal.c_str());
          // delay(100);
          // Serial.write('\0');
          // delay(50);
          // Serial.write(disease.c_str());
          // delay(100);
          // Serial.write('\0');
          // delay(50);
          Serial.write(127);
          delay(50);
          digitalWrite(Output_mega, LOW);
        }
      }

      else if (BT_temp == 125) {
        while ((BT_temp != 127) && (BT_temp != 126)) {
          flag_while = 1;
          if (flag_while_update == 1) {
            SerialBT.write(127);
            flag_while_update = 0;
          }

          if (SerialBT.available()) {
            BT_temp = SerialBT.read();
            if (count == 0) {
              num_samples = BT_temp;
              count = count + 1;
            } else if (count == 1) {
              num_nc = BT_temp;
              if (num_nc == 1) {
                num_set[0] = num_samples;
                count = count + 1;
              } else if (num_nc == 0) {
                count = count + 1;
              } else {
                set_counter = 0;
                sum = 0;
                while (set_counter < num_nc) {
                  if (flag_while_update == 1) {
                    SerialBT.write(127);
                    flag_while_update = 0;
                  }
                  if (SerialBT.available()) {
                    BT_temp = SerialBT.read();
                    num_set[set_counter] = BT_temp;
                    sum = sum + num_set[set_counter];
                    set_counter = set_counter + 1;
                  }
                }
                if (sum != num_samples) {
                  count = 0;
                  sum = 0;
                  set_counter = 0;
                } else {
                  count = count + 1;
                  set_counter = 0;
                }
              }
            } else if (count == 2) {
              activ_temp = BT_temp;
              count = count + 1;
            } else if (count == 3) {
              activ_hr = BT_temp;
              count = count + 1;
            } else if (count == 4) {
              activ_min = BT_temp;
              count = count + 1;
            } else if (count == 5) {
              activ_sec = BT_temp;
              count = count + 1;
            } else if (count == 6) {
              pcr_steps = BT_temp;
              count = count + 1;
            } else if (count == 7) {
              num_cycles = BT_temp;
              count = count + 1;
            } else if (count == 8) {
              init_temp = BT_temp;
              count = count + 1;
            } else if (count == 9) {
              init_hr = BT_temp;
              count = count + 1;
            } else if (count == 10) {
              init_min = BT_temp;
              count = count + 1;
            } else if (count == 11) {
              init_sec = BT_temp;
              count = count + 1;
            } else if (count == 12) {
              denat_temp = BT_temp;
              count = count + 1;
            } else if (count == 13) {
              denat_hr = BT_temp;
              count = count + 1;
            } else if (count == 14) {
              denat_min = BT_temp;
              count = count + 1;
            } else if (count == 15) {
              denat_sec = BT_temp;
              if (pcr_steps == 2)
                count = 20;
              else if (pcr_steps == 3)
                count = count + 1;
            } else if (count == 16) {
              anneal_temp = BT_temp;
              count = count + 1;
            } else if (count == 17) {
              anneal_hr = BT_temp;
              count = count + 1;
            } else if (count == 18) {
              anneal_min = BT_temp;
              count = count + 1;
            } else if (count == 19) {
              anneal_sec = BT_temp;
              count = count + 1;
            } else if (count == 20) {
              exten_temp = BT_temp;
              count = count + 1;
            } else if (count == 21) {
              exten_hr = BT_temp;
              count = count + 1;
            } else if (count == 22) {
              exten_min = BT_temp;
              count = count + 1;
            } else if (count == 23) {
              exten_sec = BT_temp;
              count = count + 1;
            } else if (count == 24) {
              final_temp = BT_temp;
              count = count + 1;
            } else if (count == 25) {
              final_hr = BT_temp;
              count = count + 1;
            } else if (count == 26) {
              final_min = BT_temp;
              count = count + 1;
            } else if (count == 27) {
              final_sec = BT_temp;
              count = count + 1;
            }
          }
        }
        flag_while = 0;

        if (BT_temp == 127) {
          digitalWrite(Output_mega, HIGH);
          delay(100);
          Serial.write(3);
          delay(50);
          Serial.write(num_samples);
          delay(50);
          Serial.write(num_nc);
          delay(50);
          for (int e = 0; e < num_nc; e++) {
            Serial.write(num_set[e]);
            delay(50);
          }
          Serial.write(activ_temp);
          delay(50);
          Serial.write(activ_hr);
          delay(50);
          Serial.write(activ_min);
          delay(50);
          Serial.write(activ_sec);
          delay(50);
          Serial.write(pcr_steps);
          delay(50);
          Serial.write(num_cycles);
          delay(50);
          Serial.write(init_temp);
          delay(50);
          Serial.write(init_hr);
          delay(50);
          Serial.write(init_min);
          delay(50);
          Serial.write(init_sec);
          delay(50);
          Serial.write(denat_temp);
          delay(50);
          Serial.write(denat_hr);
          delay(50);
          Serial.write(denat_min);
          delay(50);
          Serial.write(denat_sec);
          delay(50);
          if (pcr_steps == 3) {
            Serial.write(anneal_temp);
            delay(50);
            Serial.write(anneal_hr);
            delay(50);
            Serial.write(anneal_min);
            delay(50);
            Serial.write(anneal_sec);
            delay(50);
            Serial.write(exten_temp);
            delay(50);
            Serial.write(exten_hr);
            delay(50);
            Serial.write(exten_min);
            delay(50);
            Serial.write(exten_sec);
            delay(50);
          } else if (pcr_steps == 2) {
            Serial.write(exten_temp);
            delay(50);
            Serial.write(exten_hr);
            delay(50);
            Serial.write(exten_min);
            delay(50);
            Serial.write(exten_sec);
            delay(50);
          }
          Serial.write(final_temp);
          delay(50);
          Serial.write(final_hr);
          delay(50);
          Serial.write(final_min);
          delay(50);
          Serial.write(final_sec);
          delay(50);
          // animal = " PCR";
          // disease = " ";
          // Serial.write(animal.c_str());
          // delay(100);
          // Serial.write('\0');
          // delay(50);
          // Serial.write(disease.c_str());
          // delay(100);
          // Serial.write('\0');
          // delay(50);

          Serial.write(127);
          delay(50);
          digitalWrite(Output_mega, LOW);
        }
      } else if (BT_temp == 122) {
        while ((BT_temp != 127) && (BT_temp != 126)) {
          flag_while = 1;
          if (flag_while_update == 1) {
            SerialBT.write(127);
            flag_while_update = 0;
          }

          if (SerialBT.available()) {
            if (count == 0) {
              BT_temp = SerialBT.read();
              num_samples = BT_temp;
              count = count + 1;
            } else if (count == 1) {
              BT_temp = SerialBT.read();
              //Serial.print("Dis code");
              disease_code = BT_temp;
              count = count + 1;
            }
            else if (count == 2) {
              BT_temp = SerialBT.read();
            }
          }
        }
        flag_while = 0;

        if (BT_temp == 127) {
          Serial.println("out");
          digitalWrite(Output_mega, HIGH);
          delay(100);
          Serial.write(4);
          delay(50);
          Serial.write(num_samples);
          delay(50);
          Serial.write(disease_code);
          delay(50);

          // animal = " PCR";
          // disease = " ";
          // Serial.write(animal.c_str());
          // delay(100);
          // Serial.write('\0');
          // delay(50);
          // Serial.write(disease.c_str());
          // delay(100);
          // Serial.write('\0');
          // delay(50);
          Serial.write(127);
          delay(50);
          digitalWrite(Output_mega, LOW);
          //Serial.println(num_samples);
          //Serial.println(disease_code);
        }
      }
      count = 0;
    } else
      while (SerialBT.available()) {
        rubish = SerialBT.read();
      }
  }

  else if ((digitalRead(end_reaction) == 1) && (digitalRead(ON_input) == 1) && (digitalRead(PCR_input) == 0) && (flag_end == 0)) {
    count = 0;
    while (count < 5) {
      if (Serial.available() > 0) {
        flag_read = 1;
        if ((count == 0) && (flag_read == 1)) {
          month = Serial.read();
          count = count + 1;
          flag_read = 0;
        } else if ((count == 1) && (flag_read == 1)) {
          day = Serial.read();
          count = count + 1;
          flag_read = 0;
        } else if ((count == 2) && (flag_read == 1)) {
          year = Serial.read();
          count = count + 1;
          flag_read = 0;
        } else if ((count == 3) && (flag_read == 1)) {
          hour = Serial.read();
          count = count + 1;
          flag_read = 0;
        } else if ((count == 4) && (flag_read == 1)) {
          minute = Serial.read();
          count = count + 1;

          String message = String(month) + "/" + String(day) + "/" + String(year) + " " + String(hour) + ":" + String(minute) + " " + String(animal) + " " + String(disease) + "\n";
          appendFile(SD, "/kaldata.txt", message.c_str());

          // Serial.print("month ");
          // Serial.println(month);
          // Serial.print("day ");
          // Serial.println(day);
          // Serial.print("year ");
          // Serial.println(year);
          // Serial.print("hour ");
          // Serial.println(hour);
          // Serial.print("minute ");
          // Serial.println(minute);
          flag_read = 0;
        }
      }
    }



    if (BT_status == 1) {
      SerialBT.write(123);
      SerialBT.flush();
    }
    rem_sec_count = 0;
    count = 0;
    flag_end = 1;
  } else if ((digitalRead(end_reaction) == 0) && (digitalRead(ON_input) == 1) && (digitalRead(PCR_input) == 1)) {
    count = 0;
    //Serial.println("ON_intput PCR_input");
    flag_update = 1;
    while (digitalRead(PCR_input) == 1) {
      if (Serial.available() > 0) {
        flag_read = 1;

        if ((count == 0) && (flag_read == 1)) {
          type = Serial.read();
          //Serial.print("type ");
          //Serial.println(type);
          count = count + 1;
          flag_read = 0;
        } else if ((count != 0) && (flag_read == 1)) {
          if ((type == 2) || (type == 4)) {
            if ((count == 1) && (flag_read == 1)) {
              cycle = Serial.read();
              //Serial.print("cycle ");
              //Serial.println(cycle);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 2) && (flag_read == 1)) {
              rem_hour = Serial.read();
              //Serial.print("rem_hour ");
              //Serial.println(rem_hour);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 3) && (flag_read == 1)) {
              rem_min = Serial.read();
              //Serial.print("rem_min ");
              //Serial.println(rem_min);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 4) && (flag_read == 1)) {
              rem_sec_serial = Serial.read();
              //Serial.print("rem_sec ");
              //Serial.println(rem_sec_serial);
              count = count + 1;
              flag_read = 0;
            }
          } else if (type == 3) {
            if ((count == 1) && (flag_read == 1)) {
              rem_hour = Serial.read();
              //Serial.print("rem_hour ");
              //Serial.println(rem_hour);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 2) && (flag_read == 1)) {
              rem_min = Serial.read();
              //Serial.print("rem_min ");
              //Serial.println(rem_min);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 3) && (flag_read == 1)) {
              rem_sec_serial = Serial.read();
              //Serial.print("rem_sec ");
              //Serial.println(rem_sec_serial);
              count = count + 1;
              flag_read = 0;
            }
          } else if (type == 1) {
            if ((count == 1) && (flag_read == 1)) {
              rev_current_step = Serial.read();
              //Serial.print("rev_current_step ");
              //Serial.println(rev_current_step);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 2) && (flag_read == 1)) {
              rem_hour = Serial.read();
              //Serial.print("rem_hour ");
              //Serial.println(rem_hour);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 3) && (flag_read == 1)) {
              rem_min = Serial.read();
              //Serial.print("rem_min ");
              //Serial.println(rem_min);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 4) && (flag_read == 1)) {
              rem_sec_serial = Serial.read();
              //Serial.print("rem_sec ");
              //Serial.println(rem_sec_serial);
              count = count + 1;
              flag_read = 0;
            }
          }
        }
      } else
        flag_read = 0;
    }
    rem_sec_count = rem_hour * 3600 + rem_min * 60 + rem_sec_serial;
    flag_update = 0;
    count = 0;
  } else if ((digitalRead(PCR_input) == 1) && (digitalRead(ON_input) == 0) && (digitalRead(end_reaction) == 0)) {
    flag_end = 0;
    count = 0;
    //Serial.println("PCR_input");
    flag_update = 1;
    while (digitalRead(PCR_input) == 1) {
      if (Serial.available() > 0) {
        flag_read = 1;

        if ((count == 0) && (flag_read == 1)) {
          type = Serial.read();
          //Serial.print("type ");
          //Serial.println(type);
          if ((type == 1) || (type == 2) || (type == 3) || (type == 4)) {
            count = count + 1;
          }
          flag_read = 0;
        } else if ((count != 0) && (flag_read == 1)) {
          if (type == 3) {
            if ((count == 1) && (flag_read == 1)) {
              num_cycles = Serial.read();
              //Serial.print("num_cycles ");
              //Serial.println(num_cycles);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 2) && (flag_read == 1)) {
              cycle = Serial.read();
              //Serial.print("cycle ");
              //Serial.println(cycle);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 3) && (flag_read == 1)) {
              rna_extract_temp = Serial.read();
              //Serial.print("rna_extract_temp ");
              //Serial.println(rna_extract_temp);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 4) && (flag_read == 1)) {
              rna_extract_hr = Serial.read();
              //Serial.print("rna_extract_hr ");
              //Serial.println(rna_extract_hr);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 5) && (flag_read == 1)) {
              rna_extract_min = Serial.read();
              //Serial.print("rna_extract_min ");
              //Serial.println(rna_extract_min);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 6) && (flag_read == 1)) {
              rna_extract_sec = Serial.read();
              //Serial.print("rna_extract_sec ");
              //Serial.println(rna_extract_sec);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 7) && (flag_read == 1)) {
              rem_hour = Serial.read();
              //Serial.print("rem_hour ");
              //Serial.println(rem_hour);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 8) && (flag_read == 1)) {
              rem_min = Serial.read();
              //Serial.print("rem_min ");
              //Serial.println(rem_min);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 9) && (flag_read == 1)) {
              rem_sec_serial = Serial.read();
              //Serial.print("rem_sec_serial ");
              //Serial.println(rem_sec_serial);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 10) && (flag_read == 1)) {
              animal = Serial.readStringUntil('\0');
              disease = Serial.readStringUntil('\0');
              //Serial.print("animal ");
              //Serial.println(animal);
              //Serial.print("disease ");
              //Serial.println(disease);
              count = count + 1;
              flag_read = 0;
            }
          } else if ((type == 2) || (type == 4)) {
            if ((count == 1) && (flag_read == 1)) {
              num_samples = Serial.read();
              //Serial.print("num_samples ");
              //Serial.println(num_samples);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 2) && (flag_read == 1)) {
              num_nc = Serial.read();
              //Serial.print("num_nc ");
              //Serial.println(num_nc);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 3) && (flag_read == 1)) {
              if (num_nc == 1) {
                num_set[0] = Serial.read();
                //Serial.print("numset[0] ");
                //Serial.println(num_set[0]);
                count = count + 1;
                flag_read = 0;
              } else if (num_nc == 0) {
                activ_temp = Serial.read();
                //Serial.print("activ_temp ");
                //Serial.println(activ_temp);
                count = 5;
                flag_read = 0;
              } else if (num_nc > 1) {
                while (set_counter < num_nc) {
                  if (Serial.available() > 0) {
                    num_set[set_counter] = Serial.read();
                    //Serial.print("num_set ");
                    //Serial.println(num_set[set_counter]);
                    set_counter = set_counter + 1;
                  }
                }
                count = count + 1;
                flag_read = 0;
              }
            } else if ((count == 4) && (flag_read == 1)) {
              activ_temp = Serial.read();
              //Serial.print("activ_temp ");
              //Serial.println(activ_temp);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 5) && (flag_read == 1)) {
              activ_hr = Serial.read();
              //Serial.print("activ_hr ");
              //Serial.println(activ_hr);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 6) && (flag_read == 1)) {
              activ_min = Serial.read();
              //Serial.print("activ_min ");
              //Serial.println(activ_min);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 7) && (flag_read == 1)) {
              activ_sec = Serial.read();
              //Serial.print("activ_sec ");
              //Serial.println(activ_sec);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 8) && (flag_read == 1)) {
              num_steps = Serial.read();
              //Serial.print("num_steps ");
              //Serial.println(num_steps);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 9) && (flag_read == 1)) {
              num_cycles = Serial.read();
              //Serial.print("num_cycles ");
              //Serial.println(num_cycles);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 10) && (flag_read == 1)) {
              init_temp = Serial.read();
              //Serial.print("init_temp ");
              //Serial.println(init_temp);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 11) && (flag_read == 1)) {
              init_hr = Serial.read();
              //Serial.print("init_hr ");
              //Serial.println(init_hr);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 12) && (flag_read == 1)) {
              init_min = Serial.read();
              //Serial.print("init_min ");
              //Serial.println(init_min);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 13) && (flag_read == 1)) {
              init_sec = Serial.read();
              //Serial.print("init_sec ");
              //Serial.println(init_sec);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 14) && (flag_read == 1)) {
              denat_temp = Serial.read();
              //Serial.print("denat_temp ");
              //Serial.println(denat_temp);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 15) && (flag_read == 1)) {
              denat_hr = Serial.read();
              //Serial.print("denat_hr ");
              //Serial.println(denat_hr);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 16) && (flag_read == 1)) {
              denat_min = Serial.read();
              //Serial.print("denat_min ");
              //Serial.println(denat_min);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 17) && (flag_read == 1)) {
              denat_sec = Serial.read();
              //Serial.print("denat_sec ");
              //Serial.println(denat_sec);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 18) && (flag_read == 1)) {
              if (num_steps == 2) {
                exten_temp = Serial.read();
                //Serial.print("exten_temp ");
                //Serial.println(exten_temp);
                count = 23;
                flag_read = 0;
              } else if (num_steps == 3) {
                anneal_temp = Serial.read();
                //Serial.print("anneal_temp ");
                //Serial.println(anneal_temp);
                count = count + 1;
                flag_read = 0;
              }
            } else if ((count == 19) && (flag_read == 1)) {
              anneal_hr = Serial.read();
              //Serial.print("anneal_hr ");
              //Serial.println(anneal_hr);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 20) && (flag_read == 1)) {
              anneal_min = Serial.read();
              //Serial.print("anneal_min ");
              //Serial.println(anneal_min);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 21) && (flag_read == 1)) {
              anneal_sec = Serial.read();
              //Serial.print("anneal_sec ");
              //Serial.println(anneal_sec);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 22) && (flag_read == 1)) {
              exten_temp = Serial.read();
              //Serial.print("exten_temp ");
              //Serial.println(exten_temp);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 23) && (flag_read == 1)) {
              exten_hr = Serial.read();
              //Serial.print("exten_hr ");
              //Serial.println(exten_hr);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 24) && (flag_read == 1)) {
              exten_min = Serial.read();
              //Serial.print("exten_min ");
              //Serial.println(exten_min);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 25) && (flag_read == 1)) {
              exten_sec = Serial.read();
              //Serial.print("exten_sec ");
              //Serial.println(exten_sec);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 26) && (flag_read == 1)) {
              final_temp = Serial.read();
              //Serial.print("final_temp ");
              //Serial.println(final_temp);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 27) && (flag_read == 1)) {
              final_hr = Serial.read();
              //Serial.print("final_hr ");
              //Serial.println(final_hr);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 28) && (flag_read == 1)) {
              final_min = Serial.read();
              //Serial.print("final_min ");
              //Serial.println(final_min);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 29) && (flag_read == 1)) {
              final_sec = Serial.read();
              //Serial.print("final_sec ");
              //Serial.println(final_sec);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 30) && (flag_read == 1)) {
              cycle = Serial.read();
              //Serial.print("cycle ");
              //Serial.println(cycle);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 31) && (flag_read == 1)) {
              rem_hour = Serial.read();
              //Serial.print("rem_hour ");
              //Serial.println(rem_hour);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 32) && (flag_read == 1)) {
              rem_min = Serial.read();
              //Serial.print("rem_min ");
              //Serial.println(rem_min);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 33) && (flag_read == 1)) {
              rem_sec_serial = Serial.read();
              //Serial.print("rem_sec ");
              //Serial.println(rem_sec_serial);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 34) && (flag_read == 1)) {
              animal = Serial.readStringUntil('\0');
              disease = Serial.readStringUntil('\0');
              //Serial.print("animal ");
              //Serial.println(animal);
              //Serial.print("disease ");
              //Serial.println(disease);
              count = count + 1;
              flag_read = 0;
            }
          } else if (type == 1) {
            if ((count == 1) && (flag_read == 1)) {
              rev_total_steps = Serial.read();
              //Serial.print("rev_total_steps ");
              //Serial.println(rev_total_steps);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 2) && (flag_read == 1)) {
              rev_current_step = Serial.read();
              //Serial.print("rev_current_step ");
              //Serial.println(rev_current_step);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 3) && (flag_read == 1)) {
              rev_1_temp = Serial.read();
              //Serial.print("rev_1_temp ");
              //Serial.println(rev_1_temp);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 4) && (flag_read == 1)) {
              rev_1_hr = Serial.read();
              //Serial.print("rev_1_hr ");
              //Serial.println(rev_1_hr);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 5) && (flag_read == 1)) {
              rev_1_min = Serial.read();
              //Serial.print("rev_1_min ");
              //Serial.println(rev_1_min);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 6) && (flag_read == 1)) {
              rev_1_sec = Serial.read();
              //Serial.print("rev_1_sec ");
              //Serial.println(rev_1_sec);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 7) && (flag_read == 1)) {
              if (rev_total_steps == 1) {
                rem_hour = Serial.read();
                //Serial.print("rem_hour ");
                //Serial.println(rem_hour);
                count = 16;
                flag_read = 0;
              } else {
                rev_2_temp = Serial.read();
                //Serial.print("rev_2_temp ");
                //Serial.println(rev_2_temp);
                count = count + 1;
                flag_read = 0;
              }
            } else if ((count == 8) && (flag_read == 1)) {
              rev_2_hr = Serial.read();
              //Serial.print("rev_2_hr ");
              //Serial.println(rev_2_hr);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 9) && (flag_read == 1)) {
              rev_2_min = Serial.read();
              //Serial.print("rev_2_min ");
              //Serial.println(rev_2_min);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 10) && (flag_read == 1)) {
              rev_2_sec = Serial.read();
              //Serial.print("rev_2_sec ");
              //Serial.println(rev_2_sec);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 11) && (flag_read == 1)) {
              if (rev_total_steps == 2) {
                rem_hour = Serial.read();
                //Serial.print("rem_hour ");
                //Serial.println(rem_hour);
                count = 16;
                flag_read = 0;
              } else {
                rev_3_temp = Serial.read();
                //Serial.print("rev_3_temp ");
                //Serial.println(rev_3_temp);
                count = count + 1;
                flag_read = 0;
              }
            } else if ((count == 12) && (flag_read == 1)) {
              rev_3_hr = Serial.read();
              //Serial.print("rev_3_hr ");
              //Serial.println(rev_3_hr);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 13) && (flag_read == 1)) {
              rev_3_min = Serial.read();
              //Serial.print("rev_3_min ");
              //Serial.println(rev_3_min);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 14) && (flag_read == 1)) {
              rev_3_sec = Serial.read();
              //Serial.print("rev_3_sec ");
              //Serial.println(rev_3_sec);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 15) && (flag_read == 1)) {
              rem_hour = Serial.read();
              //Serial.print("rem_hour ");
              //Serial.println(rem_hour);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 16) && (flag_read == 1)) {
              rem_min = Serial.read();
              //Serial.print("rem_min ");
              //Serial.println(rem_min);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 17) && (flag_read == 1)) {
              rem_sec_serial = Serial.read();
              //Serial.print("rem_sec ");
              //Serial.println(rem_sec_serial);
              count = count + 1;
              flag_read = 0;
            } else if ((count == 18) && (flag_read == 1)) {
              animal = Serial.readStringUntil('\0');
              disease = Serial.readStringUntil('\0');
              //Serial.print("animal ");
              //Serial.println(animal);
              //Serial.print("disease ");
              //Serial.println(disease);
              count = count + 1;
              flag_read = 0;
            }
          }
        }
      } else
        flag_read = 0;
    }
    if ((BT_status == 1)) {
      if (type == 1) {
        SerialBT.write(125);
        SerialBT.write(rev_total_steps);
        SerialBT.write(rev_current_step);
        if (rev_total_steps == 1) {
          SerialBT.write(rev_1_temp);
          SerialBT.write(rev_1_hr);
          SerialBT.write(rev_1_min);
          SerialBT.write(rev_1_sec);
          SerialBT.write(rem_hour);
          SerialBT.write(rem_min);
          SerialBT.write(rem_sec_serial);
          SerialBT.write(127);
        } else if (rev_total_steps == 2) {
          SerialBT.write(rev_1_temp);
          SerialBT.write(rev_1_hr);
          SerialBT.write(rev_1_min);
          SerialBT.write(rev_1_sec);
          SerialBT.write(rev_2_temp);
          SerialBT.write(rev_2_hr);
          SerialBT.write(rev_2_min);
          SerialBT.write(rev_2_sec);
          SerialBT.write(rem_hour);
          SerialBT.write(rem_min);
          SerialBT.write(rem_sec_serial);
          SerialBT.write(127);
        } else if (rev_total_steps == 3) {
          SerialBT.write(rev_1_temp);
          SerialBT.write(rev_1_hr);
          SerialBT.write(rev_1_min);
          SerialBT.write(rev_1_sec);
          SerialBT.write(rev_2_temp);
          SerialBT.write(rev_2_hr);
          SerialBT.write(rev_2_min);
          SerialBT.write(rev_2_sec);
          SerialBT.write(rev_3_temp);
          //Serial.println(rev_3_temp);
          SerialBT.write(rev_3_hr);
          //Serial.println(rev_3_hr);
          SerialBT.write(rev_3_min);
          // Serial.println(rev_3_min);
          SerialBT.write(rev_3_sec);
          //Serial.println(rev_3_sec);
          SerialBT.write(rem_hour);
          //Serial.println(rem_hour);
          SerialBT.write(rem_min);
          // Serial.println(rem_min);
          SerialBT.write(rem_sec_serial);
          //Serial.println(rem_sec_serial);
          SerialBT.write(127);
        }
      } else if (type == 2) {
        SerialBT.write(126);
        SerialBT.write(num_samples);
        SerialBT.write(num_nc);
        if (num_nc == 1) {
          SerialBT.write(num_set[0]);
        } else if (num_nc > 1) {
          for (int b = 0; b < num_nc; b++) {
            SerialBT.write(num_set[b]);
          }
        }
        SerialBT.write(activ_temp);
        SerialBT.write(activ_hr);
        SerialBT.write(activ_min);
        SerialBT.write(activ_sec);
        SerialBT.write(num_steps);
        SerialBT.write(num_cycles);
        SerialBT.write(init_temp);
        SerialBT.write(init_hr);
        SerialBT.write(init_min);
        SerialBT.write(init_sec);
        SerialBT.write(denat_temp);
        SerialBT.write(denat_hr);
        SerialBT.write(denat_min);
        SerialBT.write(denat_sec);
        if (num_steps == 3) {
          SerialBT.write(anneal_temp);
          SerialBT.write(anneal_hr);
          SerialBT.write(anneal_min);
          SerialBT.write(anneal_sec);
        }
        SerialBT.write(exten_temp);
        SerialBT.write(exten_hr);
        SerialBT.write(exten_min);
        SerialBT.write(exten_sec);
        SerialBT.write(final_temp);
        SerialBT.write(final_hr);
        SerialBT.write(final_min);
        SerialBT.write(final_sec);
        SerialBT.write(cycle);
        SerialBT.write(rem_hour);
        SerialBT.write(rem_min);
        SerialBT.write(rem_sec_serial);
        SerialBT.write(127);
      }  else if (type == 4) {
        SerialBT.write(122);
        SerialBT.write(disease_code);
        SerialBT.write(num_samples);
        for (int a = 0; a< num_samples; a++){
          SerialBT.print(animal_IDs[a]);
          SerialBT.print('\t');
        }
        SerialBT.write(activ_temp);
        SerialBT.write(activ_hr);
        SerialBT.write(activ_min);
        SerialBT.write(activ_sec);
        SerialBT.write(num_steps);
        SerialBT.write(num_cycles);
        SerialBT.write(init_temp);
        SerialBT.write(init_hr);
        SerialBT.write(init_min);
        SerialBT.write(init_sec);
        SerialBT.write(denat_temp);
        SerialBT.write(denat_hr);
        SerialBT.write(denat_min);
        SerialBT.write(denat_sec);
        if (num_steps == 3) {
          SerialBT.write(anneal_temp);
          SerialBT.write(anneal_hr);
          SerialBT.write(anneal_min);
          SerialBT.write(anneal_sec);
        }
        SerialBT.write(exten_temp);
        SerialBT.write(exten_hr);
        SerialBT.write(exten_min);
        SerialBT.write(exten_sec);
        SerialBT.write(final_temp);
        SerialBT.write(final_hr);
        SerialBT.write(final_min);
        SerialBT.write(final_sec);
        SerialBT.write(cycle);
        SerialBT.write(rem_hour);
        SerialBT.write(rem_min);
        SerialBT.write(rem_sec_serial);
        SerialBT.write(127);
      } 
      else if ((type == 3)) {
        SerialBT.write(124);
        SerialBT.write(1);
        SerialBT.write(1);
        SerialBT.write(rna_extract_temp);
        SerialBT.write(rna_extract_hr);
        SerialBT.write(rna_extract_min);
        SerialBT.write(rna_extract_sec);
        SerialBT.write(rem_hour);
        SerialBT.write(rem_min);
        SerialBT.write(rem_sec_serial);
        SerialBT.write(127);
      }
    }
    rem_sec_count = rem_hour * 3600 + rem_min * 60 + rem_sec_serial;
    flag_update = 0;
    count = 0;
    SerialBT.flush();
  }

  //  if ((digitalRead(Fluo_set)) == 0) {
  //    flag_fluo = 0;
  //  }

  if (digitalRead(qPCR) == 1) {
    //Serial.print("qPCR");
    //Fluo_Read();
    // delay(5000);
    digitalWrite(light_1, HIGH);
    digitalWrite(light_2, LOW);
    delay(1000);
    digitalWrite(light_1, LOW);
    digitalWrite(light_2, HIGH);
    delay(1000);
    digitalWrite(light_1, LOW);
    digitalWrite(light_2, LOW);
  }


  /*
        for (int x = 0; x < 8; x++) {
          for (int y = 0; y <= 16; y++) {
            for (int z = 0; z < 75; z++) {
              fluo_data[x][y][z] = 0;
            }
          }
        }
        counter = 0;

      }

      //  else if ((digitalRead(qPCR) == 1) && (digitalRead(PCR_input) == 0) && (flag_qPCR == 1)) {
      //    flag_qPCR = 0;
      //    Fluo_Read();
      //  }
      //  else if ((digitalRead(qPCR) == 1) && (digitalRead(PCR_input) == 1) && (flag_qPCR == 1)) {
      //    flag_qPCR = 0;
      //    while (counter < (num_samples + num_nc)) {
      //      for (int a = 0; a < num_nc; a++) {
      //        for (int b = 0; b <= num_set[a]; b++) {
      //          display.clearDisplay();
      //          display.setTextSize(1);
      //          display.setTextColor(WHITE);
      //          display.setCursor(0, 0);
      //          if (b == 0) {
      //            display.print("Meas. N.C ");
      //            display.println(a + 1);
      //          }
      //          else {
      //            display.println("Meas. Tube");
      //            display.print(a + 1);
      //            display.print(": ");
      //            display.println(b);
      //          }
      //          display.display();
      //          delay(5);
      //          while (digitalRead(qPCR) == 0);
      //          if ((digitalRead(qPCR) == 1) && (flag_qPCR_2 == 1)) {
      //            flag_qPCR_2 = 0;
      //            Fluo_Read();
      //            fluo_data[a][b][fluo_count] = c5;
      //          }
      //          flag_qPCR_2 = 1;
      //          counter = counter + 1;
      //        }
      //      }
      //      fluo_count = fluo_count + 1;
      //    }
      //    counter = 0;
      //  }


      else {
        Disp();
        flag_qPCR = 1;
      }

      if (digitalRead(PCR_input) == 1) {
        count = 0;
      }
      // server.handleClient();
      Serial.println(count);
      delay(50);              // wait for a second

  */
}

void Read_Chan() {
  if (!as7341.readAllChannels()) {
    //Serial.println("Error reading all channels!");
    return;
  }
  //  c1 = as7341.getChannel(AS7341_CHANNEL_415nm_F1);
  //  c2 = as7341.getChannel(AS7341_CHANNEL_445nm_F2);
  //  c3 = as7341.getChannel(AS7341_CHANNEL_480nm_F3);
  if (count == 0) {
    c4 = as7341.getChannel(AS7341_CHANNEL_515nm_F4);
    c5 = as7341.getChannel(AS7341_CHANNEL_555nm_F5);
  } else if (count == 1) {
    c6 = as7341.getChannel(AS7341_CHANNEL_590nm_F6);
    c7 = as7341.getChannel(AS7341_CHANNEL_630nm_F7);
  }
  //  c8 = as7341.getChannel(AS7341_CHANNEL_680nm_F8);

  // Print out the stored values for each channel
  //  Serial.print("F1 415nm : ");
  //  Serial.println(as7341.getChannel(AS7341_CHANNEL_415nm_F1));
  //  Serial.print("F2 445nm : ");
  //  Serial.println(as7341.getChannel(AS7341_CHANNEL_445nm_F2));
  //  Serial.print("F3 480nm : ");
  //  Serial.println(as7341.getChannel(AS7341_CHANNEL_480nm_F3));
  //  Serial.print("F4 515nm : ");
  //  Serial.println(as7341.getChannel(AS7341_CHANNEL_515nm_F4));
  //  Serial.print("F5 555nm : ");
  //  Serial.println(as7341.getChannel(AS7341_CHANNEL_555nm_F5));
  //  Serial.print("F6 590nm : ");
  //  Serial.println(as7341.getChannel(AS7341_CHANNEL_590nm_F6));
  //  Serial.print("F7 630nm : ");
  //  Serial.println(as7341.getChannel(AS7341_CHANNEL_630nm_F7));
  //  Serial.print("F8 680nm : ");
  //  Serial.println(as7341.getChannel(AS7341_CHANNEL_680nm_F8));
}



void Fluo_Read() {
  if (counter == 2) {
    counter = 0;
  }
  //Serial.println("Reading...");

  Read_Chan();
  //m1[count] = c1;
  //m2[count] = c2;
  //m3[count] = c3;
  //m4[count] = c4;
  //m5[count] = c5;
  //m6[count] = c6;
  //m7[count] = c7;
  //m8[count] = c8;
  counter = counter + 1;
  if (counter == 2) {
    //fluo_result = ((c4*0.9) + (c5*0.56)) - ((c6*0.7) + (c7*0.49));
    fluo_result = c5 * fluo_multiplier - c7;
    //Serial.println(fluo_result);
  } else {
    fluo_result = 0;
  }
  //Serial.println("Success!");
  delay(500);
}

void Bt_Status(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {

  if (event == ESP_SPP_SRV_OPEN_EVT) {
    BT_status = 1;
  }

  else if (event == ESP_SPP_CLOSE_EVT) {
    BT_status = 0;
  }
}

void update_func() {
  if ((digitalRead(ON_input) == 1) && (digitalRead(end_reaction) == 0) && (flag_update == 0)) {
    if ((BT_status == 1) && (flag_upfnc == 1)) {
      //  if ((flag_inter_start == 0) && (flag_inter_end == 0)) {
      //   flag_inter_start = 1;
      if (type == 1) {
        SerialBT.write(125);
        SerialBT.write(rev_total_steps);
        SerialBT.write(rev_current_step);
        if (rev_total_steps == 1) {
          SerialBT.write(rev_1_temp);
          SerialBT.write(rev_1_hr);
          SerialBT.write(rev_1_min);
          SerialBT.write(rev_1_sec);
          SerialBT.write(rem_hour);
          SerialBT.write(rem_min);
          SerialBT.write(rem_sec_serial);
          SerialBT.write(127);
        } else if (rev_total_steps == 2) {
          SerialBT.write(rev_1_temp);
          SerialBT.write(rev_1_hr);
          SerialBT.write(rev_1_min);
          SerialBT.write(rev_1_sec);
          SerialBT.write(rev_2_temp);
          SerialBT.write(rev_2_hr);
          SerialBT.write(rev_2_min);
          SerialBT.write(rev_2_sec);
          SerialBT.write(rem_hour);
          SerialBT.write(rem_min);
          SerialBT.write(rem_sec_serial);
          SerialBT.write(127);
        } else if (rev_total_steps == 3) {
          SerialBT.write(rev_1_temp);
          SerialBT.write(rev_1_hr);
          SerialBT.write(rev_1_min);
          SerialBT.write(rev_1_sec);
          SerialBT.write(rev_2_temp);
          SerialBT.write(rev_2_hr);
          SerialBT.write(rev_2_min);
          SerialBT.write(rev_2_sec);
          SerialBT.write(rev_3_temp);
          SerialBT.write(rev_3_hr);
          SerialBT.write(rev_3_min);
          SerialBT.write(rev_3_sec);
          SerialBT.write(rem_hour);
          SerialBT.write(rem_min);
          SerialBT.write(rem_sec_serial);
          SerialBT.write(127);
        }
      } 
       else if (type == 4) {
        SerialBT.write(122);
        SerialBT.write(disease_code);
        SerialBT.write(num_samples);
        for (int a = 0; a< num_samples; a++){
          SerialBT.print(animal_IDs[a]);
          SerialBT.print('\t');
        }
        SerialBT.write(activ_temp);
        SerialBT.write(activ_hr);
        SerialBT.write(activ_min);
        SerialBT.write(activ_sec);
        SerialBT.write(num_steps);
        SerialBT.write(num_cycles);
        SerialBT.write(init_temp);
        SerialBT.write(init_hr);
        SerialBT.write(init_min);
        SerialBT.write(init_sec);
        SerialBT.write(denat_temp);
        SerialBT.write(denat_hr);
        SerialBT.write(denat_min);
        SerialBT.write(denat_sec);
        if (num_steps == 3) {
          SerialBT.write(anneal_temp);
          SerialBT.write(anneal_hr);
          SerialBT.write(anneal_min);
          SerialBT.write(anneal_sec);
        }
        SerialBT.write(exten_temp);
        SerialBT.write(exten_hr);
        SerialBT.write(exten_min);
        SerialBT.write(exten_sec);
        SerialBT.write(final_temp);
        SerialBT.write(final_hr);
        SerialBT.write(final_min);
        SerialBT.write(final_sec);
        SerialBT.write(cycle);
        SerialBT.write(rem_hour);
        SerialBT.write(rem_min);
        SerialBT.write(rem_sec_serial);
        SerialBT.write(127);
      } 
      
      else if (type == 2) {
        SerialBT.write(126);
        SerialBT.write(num_samples);
        SerialBT.write(num_nc);
        if (num_nc == 1) {
          SerialBT.write(num_set[0]);
        } else if (num_nc > 1) {
          for (int a = 0; a < num_nc; a++) {
            SerialBT.write(num_set[a]);
          }
        }
        SerialBT.write(activ_temp);
        SerialBT.write(activ_hr);
        SerialBT.write(activ_min);
        SerialBT.write(activ_sec);
        SerialBT.write(num_steps);
        SerialBT.write(num_cycles);
        SerialBT.write(init_temp);
        SerialBT.write(init_hr);
        SerialBT.write(init_min);
        SerialBT.write(init_sec);
        SerialBT.write(denat_temp);
        SerialBT.write(denat_hr);
        SerialBT.write(denat_min);
        SerialBT.write(denat_sec);
        if (num_steps == 3) {
          SerialBT.write(anneal_temp);
          SerialBT.write(anneal_hr);
          SerialBT.write(anneal_min);
          SerialBT.write(anneal_sec);
        }
        SerialBT.write(exten_temp);
        SerialBT.write(exten_hr);
        SerialBT.write(exten_min);
        SerialBT.write(exten_sec);
        SerialBT.write(final_temp);
        SerialBT.write(final_hr);
        SerialBT.write(final_min);
        SerialBT.write(final_sec);
        SerialBT.write(cycle);
        SerialBT.write(rem_hour);
        SerialBT.write(rem_min);
        SerialBT.write(rem_sec_serial);
        SerialBT.write(127);
      }
       else if ((type == 3)) {
        SerialBT.write(124);
        SerialBT.write(1);
        SerialBT.write(1);
        SerialBT.write(rna_extract_temp);
        SerialBT.write(rna_extract_hr);
        SerialBT.write(rna_extract_min);
        SerialBT.write(rna_extract_sec);
        SerialBT.write(rem_hour);
        SerialBT.write(rem_min);
        SerialBT.write(rem_sec_serial);
        SerialBT.write(127);
      }
      flag_inter_end = 1;
      //   }
    }
  } else if ((digitalRead(ON_input) == 0)) {
    if (BT_status == 1) {
      if (flag_upfnc == 1) {
        SerialBT.write(127);
      }
    }
  } else if ((digitalRead(end_reaction) == 1) && (digitalRead(ON_input) == 1) && (digitalRead(PCR_input) == 0) && (flag_end == 1)) {
    if (BT_status == 1) {
      if (flag_upfnc == 1) {
        SerialBT.write(123);
      }
    }
  } else {
    if (BT_status == 1) {
      if (flag_upfnc == 1) {
        SerialBT.write(127);
      }
    }
  }
  rem_hour = rem_sec_count / 3600;
  rem_min = (rem_sec_count % 3600) / 60;
  rem_sec_serial = rem_sec_count % 60;
  flag_upfnc = 0;
  SerialBT.flush();
}

void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.path(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void createDir(fs::FS &fs, const char *path) {
  Serial.printf("Creating Dir: %s\n", path);
  if (fs.mkdir(path)) {
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}

void removeDir(fs::FS &fs, const char *path) {
  Serial.printf("Removing Dir: %s\n", path);
  if (fs.rmdir(path)) {
    Serial.println("Dir removed");
  } else {
    Serial.println("rmdir failed");
  }
}

void readFile(fs::FS &fs, const char *path) {
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void renameFile(fs::FS &fs, const char *path1, const char *path2) {
  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
    Serial.println("File renamed");
  } else {
    Serial.println("Rename failed");
  }
}

void deleteFile(fs::FS &fs, const char *path) {
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) {
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}

void testFileIO(fs::FS &fs, const char *path) {
  File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;
  if (file) {
    len = file.size();
    size_t flen = len;
    start = millis();
    while (len) {
      size_t toRead = len;
      if (toRead > 512) {
        toRead = 512;
      }
      file.read(buf, toRead);
      len -= toRead;
    }
    end = millis() - start;
    Serial.printf("%u bytes read for %u ms\n", flen, end);
    file.close();
  } else {
    Serial.println("Failed to open file for reading");
  }


  file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  size_t i;
  start = millis();
  for (i = 0; i < 2048; i++) {
    file.write(buf, 512);
  }
  end = millis() - start;
  Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
  file.close();
}