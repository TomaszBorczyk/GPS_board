
#include "Adafruit_FONA.h"

#define FONA_RX  9
#define FONA_TX  8
#define FONA_RST 4
#define FONA_RI  7

#define _URL "gps-tracker.herokuapp.com/api/v1/device/registerdevice"
// this is a large buffer for replies
char replybuffer[255];

#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

// uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);

uint8_t type;



void setup() {
  // while (!Serial);

  // Serial.begin(115200);
  // Serial.println(F("FONA basic test"));
  // Serial.println(F("Initializing....(May take 3 seconds)"));

  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
  //   Serial.println(F("Couldn't find FONA"));
    while (1);
  }
  type = fona.type();
  // Serial.println(F("FONA is OK"));
  fona.setGPRSNetworkSettings(F("internet"));
}

// void setup() {
//   while (!Serial);

//   Serial.begin(115200);
//   Serial.println(F("FONA basic test"));
//   Serial.println(F("Initializing....(May take 3 seconds)"));

//   fonaSerial->begin(4800);
//   if (! fona.begin(*fonaSerial)) {
//     Serial.println(F("Couldn't find FONA"));
//     while (1);
//   }
//   type = fona.type();
//   Serial.println(F("FONA is OK"));
  
// //   char imei[16] = {0}; // MUST use a 16 character buffer for IMEI!
// //   uint8_t imeiLen = fona.getIMEI(imei);
// //   if (imeiLen > 0) {
// //     Serial.print("Module IMEI: "); Serial.println(imei);
// //   }

//   fona.setGPRSNetworkSettings(F("internet"));

//   // Optionally configure HTTP gets to follow redirects over SSL.
//   // Default is not to follow SSL redirects, however if you uncomment
//   // the following line then redirects over SSL will be followed.
//   //fona.setHTTPSRedirect(true);
// }


void loop() {
  // Serial.print(F("FONA> "));
  // while (! Serial.available() ) {
  //   if (fona.available()) {
  //     Serial.write(fona.read());
  //   }
  // }

  while(!fona.available());
  while(getNetworkStatus()!=1);
  while(!enableGPRS()){
      delay(1000);
  }
  // char time[23] = { 0 };
  char *time = getNetworkTime();
  // char *data = "{ \"device_id\":\"%s\" }";
  char data[200];
  sprintf(data, "{ \"device_id\":%s }", time);

  postData(_URL, data);

  while(1);
//   char command = Serial.read();
//   Serial.println(command);


  // flush input
  // flushSerial();
  // while (fona.available()) {
  //   Serial.write(fona.read());
  // }

}

void postData(char *URL, char *data){
  uint16_t statuscode;
  int16_t length;

  // flushSerial();
  // Serial.println(URL);
  // Serial.println(data);

  if (!fona.HTTP_POST_start(URL, F("application/json"), (uint8_t *) data, strlen(data), &statuscode, (uint16_t *)&length)) {
    // Serial.println("Failed!");
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

const char *getNetworkTime(){
    char buffer[23];
    fona.getTime(buffer, 23);  // make sure replybuffer is at least 23 bytes!
    return buffer;
}

boolean enableGPRS(){
    if (!fona.enableGPRS(true)){
        // Serial.println(F("Failed to turn on"));
        return false;
    }
    return true;
}



// void flushSerial() {
//   while (Serial.available())
//     Serial.read();
// }

// char readBlocking() {
//   while (!Serial.available());
//   return Serial.read();
// }

// uint16_t readnumber() {
//   uint16_t x = 0;
//   char c;
//   while (! isdigit(c = readBlocking())) {
//     //Serial.print(c);
//   }
//   Serial.print(c);
//   x = c - '0';
//   while (isdigit(c = readBlocking())) {
//     Serial.print(c);
//     x *= 10;
//     x += c - '0';
//   }
//   return x;
// }

// uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout) {
//   uint16_t buffidx = 0;
//   boolean timeoutvalid = true;
//   if (timeout == 0) timeoutvalid = false;

//   while (true) {
//     if (buffidx > maxbuff) {
//       //Serial.println(F("SPACE"));
//       break;
//     }

//     while (Serial.available()) {
//       char c =  Serial.read();

//       //Serial.print(c, HEX); Serial.print("#"); Serial.println(c);

//       if (c == '\r') continue;
//       if (c == 0xA) {
//         if (buffidx == 0)   // the first 0x0A is ignored
//           continue;

//         timeout = 0;         // the second 0x0A is the end of the line
//         timeoutvalid = true;
//         break;
//       }
//       buff[buffidx] = c;
//       buffidx++;
//     }

//     if (timeoutvalid && timeout == 0) {
//       //Serial.println(F("TIMEOUT"));
//       break;
//     }
//     delay(1);
//   }
//   buff[buffidx] = 0;  // null term
//   return buffidx;
// }
