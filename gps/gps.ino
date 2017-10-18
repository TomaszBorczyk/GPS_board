#include "Adafruit_FONA.h"
#include <avr/sleep.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>

#define FONA_RX  9
#define FONA_TX  8
#define FONA_RST 4
#define FONA_RI  7

#define INTR 2
#define LED 13
#define TILT_IN 18

#define IMEI "862877033359769"
#define _URL "gps-tracker.herokuapp.com/api/v1/device/updatelocation"

enum deviceMode {SLEEP, TRACK, ENTER_TRACK};

volatile enum deviceMode mode;
volatile bool state;
char replybuffer[255];
float lat, lon;

TinyGPS gps;

SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

uint8_t type;


void setup() {
  pinMode(LED, OUTPUT);
  pinMode(INTR, INPUT_PULLUP);
  attachTiltIntr();

  while (!Serial);
  Serial.begin(9600);
  while(!Serial1);
  Serial1.begin(9600);
  Serial.println(F("FONA basic test"));

  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    while (1);
  }
  type = fona.type();
  fona.setGPRSNetworkSettings(F("internet"));
  state = false;
  mode = SLEEP;
  lat = 0.11;
  lon = 0.11;
  basicSetup();
}


void loop() {
  // while (! Serial.available() ) {
    // if (fona.available()) {
    //   Serial.write(fona.read());
    // }
  // }
      
  // sleepMCU();

  // main code idea 
  // while(mode == TRACK){
  //   if(gps.encode(Serial1.read())){ 
  //     gps.f_get_position(&lat, &lon);
  //     char * response = postData(createJSONData(lat, lon));
  //     checkIfPositionNotChanging();
  //     checkResponse(response);
  //     if(shouldStop) enterSleepMode();
  //     delay(15000);
  //   }
  // }
  // else if(mode == SLEEP){
  // }
  // end of main code idea 


  if(mode == SLEEP){
    Serial.println(F("Sleep mode"));
    delay(100);
  }
  else if(mode == TRACK || mode == ENTER_TRACK){
    if(mode == ENTER_TRACK){
      // setupGPRS();

    }
    while(!Serial1.available());
    while(Serial1.available()){ // check for gps data
      if(gps.encode(Serial1.read())){ 
        gps.f_get_position(&lat,&lon);
        Serial.println(createJSONData(IMEI, lat, lon));
        postData(_URL, createJSONData(IMEI, lat, lon));
        delay(15000);
      }
    }
  }

}

void basicSetup(){
  while(!fona.available()){
    // Serial.println("fona not av");
  }
  while(getNetworkStatus()!=1){
    // Serial.println("no netwrok");
  }
  while(!enableGPRS()){
    // Serial.println("enabloong gprs");
    delay(1000);
  }
}

void flushSerial() {
  while (Serial.available())
    Serial.read();
}

void attachInterruptForTilt(){
  attachInterrupt(digitalPinToInterrupt(INTR), blink, CHANGE);
}

void attachTiltIntr(){
  attachInterrupt(digitalPinToInterrupt(INTR), enterTrackingMode, RISING);
}

void enterTrackingMode(){
  sleep_disable();
  detachInterrupt(digitalPinToInterrupt(INTR));
  mode = TRACK;
  // fona.enableGPRS(true);
}

void enterSleepMode(){
  mode = SLEEP;
  attachTiltIntr();
  fona.enableGPRS(false);
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

const char *createJSONData(const char* id, float lat, float lon){
  char buffer[250];
  char lat_c[12], lon_c[13];
  dtostrf(lat, 9, 7, lat_c);
  dtostrf(lon, 10, 7, lon_c);
  lat_c[11]='\0';
  lon_c[12]='\0';
  
  sprintf(buffer, "{\"device_id\":\"%s\",\"lat\":%s,\"lon\":%s}%c", id, lat_c, lon_c, '\0');
  return buffer;
}


void postData(char *URL, char *data){
  uint16_t statuscode;
  int16_t length;

  if (!fona.HTTP_POST_start(URL, F("application/json"), (uint8_t *) data, strlen(data), &statuscode, (uint16_t *)&length)) {
  }
  while (length > 0) {
    while (fona.available()) {
      char c = fona.read();
      // Serial.write(c);
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

boolean enableGPRS(){
    if (!fona.enableGPRS(true)){
        return false;
    }
    return true;
}

void sleepMCU(){
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); 
  sleep_enable();  
  attachTiltIntr();
  sleep_mode(); 
  // sleep_disable();  
}
