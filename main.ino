//////////////////// Mavlink to PPM Sum //////////////////////////
/*  
  Gregory Dymarek / 1st November 2016

  Designed & tested on TeensyLC

  This is a transparent Mavlink bi-directional proxy that converts
  any incoming MANUAL_CONTROL messages into PPM signal;
*/
//////////////////////////////////////////////////////////////////


#include "PulsePosition.h"
#include "mavlink_v2/common/mavlink.h"


//////////////////////CONFIGURATION///////////////////////////////
#define DEBUG_SERIAL_SPEED 115200 
#define QGC_SERIAL_SPEED 115200 //115200
#define LRS_SERIAL_SPEED 115200 //115200
#define LED_PIN 13
#define PPM_PIN 20  //set PPM out pin. For TeensyLC: 6, 9, 10, 20, 22, 23
#define PPM_CHANNELS 16

#define S1 Serial //source
#define S2 Serial2  //PPM target (LRS)
#define DEBUG Serial1
////////////////////////////////////

PulsePositionOutput ppmOut;

#define PPM_MIN 1000
#define PPM_MID 1500
#define PPM_MAX 2000

int ppm[PPM_CHANNELS];
uint16_t i;
char buf[MAVLINK_MAX_PACKET_LEN];
uint16_t c = 0;

mavlink_status_t status;
mavlink_message_t msg;


void toggleLED(uint16_t ms) {
  digitalWrite(LED_PIN, HIGH);
  delay(ms);
  digitalWrite(LED_PIN, LOW);
}

void setup()
{
  DEBUG.begin(DEBUG_SERIAL_SPEED);
  pinMode(LED_PIN, OUTPUT);

  S1.begin(QGC_SERIAL_SPEED);
  S2.begin(LRS_SERIAL_SPEED, SERIAL_8N1);

  for (i = 0; i < PPM_CHANNELS; i++)
    ppm[i] = PPM_MID;

  ppmOut.begin(PPM_PIN); //all channels will default to 1500;
  DEBUG.println("");
  DEBUG.println("Started.");

  toggleLED(1000);
      for (i = 0; i < PPM_CHANNELS; i++) {
        ppmOut.write(i + 1, 1500);
  }
}

/*
void printBIN(uint16_t v) {
  for (int i = 0; i < 16; i++) {
      if (v < pow(2, i))
            DEBUG.print(B0);
  }
  if (v) DEBUG.print(v,BIN);
}
*/

void toggleChannel(uint8_t i) {
  if (ppm[i] == PPM_MIN) ppm[i] = PPM_MID;
  else if (ppm[i] == PPM_MID) ppm[i] = PPM_MAX;
  else ppm[i] = PPM_MIN;
}

void buttonsToPPM(uint16_t b) {
  static uint16_t prev = b;
  uint8_t i;

  if (prev!=b) {
    for (i = 0; i < PPM_CHANNELS - 4; i++) {
      if (((b >> i) & 1) //button pressed now
          && (!(prev >> i) & 1)) //but not pressed before
        toggleChannel(4 + i);
    }

    prev = b;
  }
}

void loop() {
  uint16_t b = 0;
  if (S1.available()) {

    buf[c++] = S1.read();

    if (c >= MAVLINK_MAX_PACKET_LEN) {
      DEBUG.println("MAX!!");
      mavlink_reset_channel_status(MAVLINK_COMM_0);
      buf[0] = buf[c-1];
      c = 1;
    }

    uint8_t ret = mavlink_parse_char(MAVLINK_COMM_0, buf[c - 1], &msg, &status);
    if (ret==0) return;

    if (msg.msgid == MAVLINK_MSG_ID_MANUAL_CONTROL) {
      digitalWrite(LED_PIN, HIGH);
      ppm[0] = 1500 + mavlink_msg_manual_control_get_x(&msg) / 2;
      ppm[1] = 1500 + mavlink_msg_manual_control_get_y(&msg) / 2;
      ppm[2] = 1500 + mavlink_msg_manual_control_get_z(&msg) / 2; //throttle
      ppm[3] = 1500 + mavlink_msg_manual_control_get_r(&msg) / 2;

      DEBUG.println(mavlink_msg_manual_control_get_z(&msg) / 2);

      buttonsToPPM(mavlink_msg_manual_control_get_buttons(&msg));

      for (i = 0; i < PPM_CHANNELS; i++) {
        ppmOut.write(i + 1, ppm[i]);
      }      
      digitalWrite(LED_PIN, LOW);   
    } else {
      for (i = 0; i < c; i++) S2.write(buf[i]);
    }

    c = 0;

  }

  if (S2.available()) //there is nothing to parse from LRS (flight controller)
    S1.write(S2.read());
}

