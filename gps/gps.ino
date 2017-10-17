#include "Adafruit_FONA.h"
#include <SoftwareSerial.h>

#define FONA_RX  9
#define FONA_TX  8
#define FONA_RST 4
#define FONA_RI  7
#define INTR 2
#define LED 13

#define _URL "gps-tracker.herokuapp.com/api/v1/device/registerdevice"
// this is a large buffer for replies
char replybuffer[255];
volatile bool state;

SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

uint8_t type;


void setup() {
  while (!Serial);
  Serial.begin(115200);
  Serial.println(F("FONA basic test"));

  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    while (1);
  }
  type = fona.type();
  fona.setGPRSNetworkSettings(F("internet"));
  state = false;
}


void loop() {
  Serial.print(F("FONA> "));
  while (! Serial.available() ) {
    if (fona.available()) {
      Serial.write(fona.read());
    }
  }

  // while(!fona.available());
  // while(getNetworkStatus()!=1);
  // while(!enableGPRS()){
  //     delay(1000);
  // }

  // char *time = getNetworkTime();
  // char data[200];
  // sprintf(data, "{ \"device_id\":%s }", time);

  // postData(_URL, data);

  // while(1);

  while(1){
    Serial.println(readTilt());
    delay(50);
  }

  flushSerial();
  while (fona.available()) {
    Serial.write(fona.read());
  }
}

void flushSerial() {
  while (Serial.available())
    Serial.read();
}

void blink(){
  if(state == false) digitalWrite(LED, HIGH);
  else digitalWrite(LED, LOW);
  state = !state;
}

float readTilt(){
  float value;
  value = pulseIn(TILT_IN, HIGH);
  return value;
}


void postData(char *URL, char *data){
  uint16_t statuscode;
  int16_t length;

  if (!fona.HTTP_POST_start(URL, F("application/json"), (uint8_t *) data, strlen(data), &statuscode, (uint16_t *)&length)) {
  }
  while (length > 0) {
    while (fona.available()) {
      char c = fona.read();
      length--;
      if (! length) break;
    }
  }
  fona.HTTP_POST_end();
}

uint8_t getNetworkStatus(){
    uint8_t n = fona.getNetworkStatus();
    return n;    
}

const char *getNetworkTime(){
    char buffer[23];
    fona.getTime(buffer, 23);  // make sure replybuffer is at least 23 bytes!
    return buffer;
}

boolean enableGPRS(){
    if (!fona.enableGPRS(true)){
        return false;
    }
    return true;
}
