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

#define RADIUS 6373
#define TO_RAD (3.1415926536 / 180)
#define MAX_DISTANCE 20
#define MAX_ITERATIONS 3
#define GPS_SLEEP_TIME 5000

enum deviceMode {SLEEP, TRACK, ENTER_TRACK, ENTER_SLEEP};

volatile enum deviceMode mode;
float lat, lon;
float startLat, startLon;
int trackIterations;
TinyGPS gps;

boolean debug;

SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);


void setup() {
  debug = false;
  pinMode(LED, OUTPUT);
  pinMode(INTR, INPUT_PULLUP);
  
  basicSetup();
  serialSetup();
  fonaSetup();
  resetTrackIterations();
  initialWait();
  attachTiltIntr();
}


void loop() {
  // while (! Serial.available() ) {
    // if (fona.available()) {
    //   Serial.write(fona.read());
    // }
  // }

  if(mode == SLEEP){
    if(debug) Serial.println(F("Sleep mode"));
    delay(1000);
  }
  else if(mode == TRACK || mode == ENTER_TRACK){
    if(mode == ENTER_TRACK){
      while(Serial1.available()){ // check for gps data
        if(gps.encode(Serial1.read())){ 
          gps.f_get_position(&startLat, &startLon);
          if(debug) Serial.println(createJSONData(IMEI, startLat, startLon, "alert"));
          postData(_URL, createJSONData(IMEI, startLat, startLon, "alert"));
          mode = TRACK;
          break;
        }
      }
    }
    if(mode == TRACK){
      while(!Serial1.available());
      while(Serial1.available()){ // check for gps data
        if(gps.encode(Serial1.read())){ 
          gps.f_get_position(&lat,&lon);
          if(debug) Serial.println(createJSONData(IMEI, lat, lon, "update"));
          postData(_URL, createJSONData(IMEI, lat, lon, "update"));
          if(trackIterations > MAX_ITERATIONS && isCloseToStart(calculateDistance(lat, lon, startLat, startLon))){
            mode = ENTER_SLEEP;
            break;
          }
          trackIterations++;
          delay(GPS_SLEEP_TIME);
        }
      }

    }

  }
  if(mode == ENTER_SLEEP){
    if(debug) Serial.println("Entering sleep...");
    enterSleepMode();
    resetTrackIterations();
  }

}

void fonaSetup(){
  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    while (1);
  }
  while(!fona.available());
  while(getNetworkStatus()!=1);
  fona.setGPRSNetworkSettings(F("internet"));
  while(!enableGPRS()){
    delay(1000);
  }
  fona.enableNetworkTimeSync(true);
}

void basicSetup(){
  mode = SLEEP;
}

void serialSetup(){
  if(debug) {
    while (!Serial);
    Serial.begin(9600);
  }
  while(!Serial1);
  Serial1.begin(9600);
}

void initialWait(){
  delay(10000);
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

void resetTrackIterations(){
  trackIterations = 0;
}

void attachTiltIntr(){
  // zmien na CHANGE
  attachInterrupt(digitalPinToInterrupt(INTR), enterTrackingMode, CHANGE);
}

void enterTrackingMode(){
  // sleep_disable();
  detachInterrupt(digitalPinToInterrupt(INTR));
  mode = ENTER_TRACK;
  // fona.enableGPRS(true);
}

void enterSleepMode(){
  mode = SLEEP;
  attachTiltIntr();
  // fona.enableGPRS(false);
}

void sleepMCU(){
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); 
  sleep_enable();  
  attachTiltIntr();
  sleep_mode(); 
  // sleep_disable();  
}



bool isCloseToStart(float distance){
  return distance < MAX_DISTANCE;
}

float calculateDistance(float lat1, float lon1, float lat2, float lon2){
  float dLat, dLon, a, c, distance;

  dLat = toRadians(lat2 - lat1);
  dLon = toRadians(lon2 - lon1);
  a = pow(sin(dLat/2), 2) + cos(lat1) * cos(lat2) * pow(sin(dLon/2), 2);
  // c = 2 * atan2( sqrt(a), sqrt(1-a));
  // distance = RADIUS * c;
  distance = 2 * RADIUS * asin(sqrt(a));

  return distance*1000;
}

float toRadians(float value){
  return TO_RAD * value;
}


const char *createJSONData(const char* id, float lat, float lon, const char* type){
  char buffer[250];
  char lat_c[12], lon_c[13];
  dtostrf(lat, 9, 7, lat_c);
  dtostrf(lon, 10, 7, lon_c);
  lat_c[11]='\0';
  lon_c[12]='\0';
  
  sprintf(buffer, "{\"deviceId\":\"%s\",\"lat\":%s,\"lon\":%s, \"type\":\"%s\"}%c", id, lat_c, lon_c, type, '\0');
  return buffer;
}

const char getTime() {
  char buffer[250];
  fona.getTime(*buffer, 240);
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
