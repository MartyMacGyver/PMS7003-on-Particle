//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Demo: interfacing a Plantower PMS7003 air quality sensor to a
//  Particle IoT microcontroller and an SSD1306 128x32 OLED display
/*
    Copyright (c) 2016-2018 Martin F. Falatic
    
    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at
    
        http://www.apache.org/licenses/LICENSE-2.0
    
    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#include "Adafruit_SSD1306.h"
#include "Adafruit_GFX.h"

SYSTEM_MODE(SEMI_AUTOMATIC);

const bool DEBUG = false;
const int printbufLen = 256;
char printbuf[printbufLen];

typedef struct {
    char     deviceType[32];
    uint8_t  frameHeader[2];
    uint16_t frameLen;
    uint16_t concPM1_0_CF1;
    uint16_t concPM2_5_CF1;
    uint16_t concPM10_0_CF1;
    uint16_t concPM1_0_amb;
    uint16_t concPM2_5_amb;
    uint16_t concPM10_0_amb;
    uint16_t rawGt0_3um;
    uint16_t rawGt0_5um;
    uint16_t rawGt1_0um;
    uint16_t rawGt2_5um;
    uint16_t rawGt5_0um;
    uint16_t rawGt10_0um;
    uint8_t  version;
    uint8_t  errorCode;
    uint16_t checksum;
    bool     dataGood;
    uint32_t readCount;
} PMS7003_framestruct;


bool pms7003_read(PMS7003_framestruct *thisFrame, char * printbuf, int printbufLen) {
    // Particle.publish("PMS7003", printbuf, 60, PRIVATE);
    // send data only when you receive data:
    //sprintf(thisFrame->deviceType, "PMS7003");

    uint16_t calcChecksum = 0;
    const int MAX_FRAME_LEN = 64;
    int incomingByte = 0;  // for incoming serial data
    char frameBuf[MAX_FRAME_LEN];
    int detectOff = 0;
    int frameLen = MAX_FRAME_LEN;
    bool inFrame = false;

    thisFrame->frameLen = MAX_FRAME_LEN;

    strcpy(thisFrame->deviceType, "PMS7003");
    Serial.println("-- Reading PMS7003");
    Serial1.begin(9600);
    bool packetReceived = false;
    while (!packetReceived) {
        if (Serial1.available() > 32) {
            int drain = Serial1.available();
            if (DEBUG) {
                Serial.print("-- Draining buffer: ");
                Serial.println(Serial1.available(), DEC);
            }
            for (int i = drain; i > 0; i--) {
                Serial1.read();
            }
        }
        if (Serial1.available() > 0) {
            if (DEBUG) {
                Serial.print("-- Available: ");
                Serial.println(Serial1.available(), DEC);
            }
            incomingByte = Serial1.read();
            if (DEBUG) {
                Serial.print("-- READ: ");
                Serial.println(incomingByte, HEX);
            }
            if (!inFrame) {
                if (incomingByte == 0x42 && detectOff == 0) {
                    frameBuf[detectOff] = incomingByte;
                    thisFrame->frameHeader[0] = incomingByte;
                    calcChecksum = incomingByte; // Checksum init!
                    detectOff++;
                }
                else if (incomingByte == 0x4D && detectOff == 1) {
                    frameBuf[detectOff] = incomingByte;
                    thisFrame->frameHeader[1] = incomingByte;
                    calcChecksum += incomingByte;
                    inFrame = true;
                    detectOff++;
                }
                else {
                    Serial.print("-- Frame syncing... ");
                    Serial.print(incomingByte, HEX);
                    if (DEBUG) {
                    }
                    Serial.println();
                }
            }
            else {
                frameBuf[detectOff] = incomingByte;
                calcChecksum += incomingByte;
                detectOff++;
                uint16_t val = frameBuf[detectOff-1]+(frameBuf[detectOff-2]<<8);
                switch (detectOff) {
                    case 4:
                        thisFrame->frameLen = val;
                        frameLen = val + detectOff;
                        break;
                    case 6:
                        thisFrame->concPM1_0_CF1 = val;
                        break;
                    case 8:
                        thisFrame->concPM2_5_CF1 = val;
                        break;
                    case 10:
                        thisFrame->concPM10_0_CF1 = val;
                        break;
                    case 12:
                        thisFrame->concPM1_0_amb = val;
                        break;
                    case 14:
                        thisFrame->concPM2_5_amb = val;
                        break;
                    case 16:
                        thisFrame->concPM10_0_amb = val;
                        break;
                    case 18:
                        thisFrame->rawGt0_3um = val;
                        break;
                    case 20:
                        thisFrame->rawGt0_5um = val;
                        break;
                    case 22:
                        thisFrame->rawGt1_0um = val;
                        break;
                    case 24:
                        thisFrame->rawGt2_5um = val;
                        break;
                    case 26:
                        thisFrame->rawGt5_0um = val;
                        break;
                    case 28:
                        thisFrame->rawGt10_0um = val;
                        break;
                    case 29:
                        val = frameBuf[detectOff-1];
                        thisFrame->version = val;
                        break;
                    case 30:
                        val = frameBuf[detectOff-1];
                        thisFrame->errorCode = val;
                        break;
                    case 32:
                        thisFrame->checksum = val;
                        calcChecksum -= ((val>>8)+(val&0xFF));
                        break;
                    default:
                        break;
                }
    
                if (detectOff >= frameLen) {
                    snprintf(printbuf, printbufLen, "PMS7003 ");
                    snprintf(printbuf, printbufLen, "%s[%02x %02x] (%04x) ", printbuf,
                        thisFrame->frameHeader[0], thisFrame->frameHeader[1], thisFrame->frameLen);
                    snprintf(printbuf, printbufLen, "%sCF1=[%04x %04x %04x] ", printbuf,
                        thisFrame->concPM1_0_CF1, thisFrame->concPM2_5_CF1, thisFrame->concPM10_0_CF1);
                    snprintf(printbuf, printbufLen, "%samb=[%04x %04x %04x] ", printbuf,
                        thisFrame->concPM1_0_amb, thisFrame->concPM2_5_amb, thisFrame->concPM10_0_amb);
                    snprintf(printbuf, printbufLen, "%sraw=[%04x %04x %04x %04x %04x %04x] ", printbuf,
                        thisFrame->rawGt0_3um, thisFrame->rawGt0_5um, thisFrame->rawGt1_0um,
                        thisFrame->rawGt2_5um, thisFrame->rawGt5_0um, thisFrame->rawGt10_0um);
                    snprintf(printbuf, printbufLen, "%sver=%02x err=%02x ", printbuf,
                        thisFrame->version, thisFrame->errorCode);
                    snprintf(printbuf, printbufLen, "%scsum=%04x %s xsum=%04x", printbuf,
                        thisFrame->checksum, (calcChecksum == thisFrame->checksum ? "==" : "!="), calcChecksum);
                    Serial.println(printbuf);
                    Particle.publish("Data1", printbuf, 60, PRIVATE);
                    packetReceived = true;
                    detectOff = 0;
                    inFrame = false;
                }
            }
        }
    }
    Serial1.end();
    thisFrame->dataGood = calcChecksum == thisFrame->checksum;
    thisFrame->readCount++;
    return thisFrame->dataGood;
}


#define OLED_RESET D7
Adafruit_SSD1306 display(OLED_RESET);
int wifiSwitch = D4;
int flipSwitch = D5;
int holdSwitch = D6;
bool flipState = false;

void updateOrientation(Adafruit_SSD1306 * pDisplay, bool * flipState, int flipSwitch) {
    bool newState = bool(digitalRead(flipSwitch));
    if (newState != *flipState) {
        *flipState = newState;
        if (newState) {
            pDisplay->setRotation(0);
        }
        else {
            pDisplay->setRotation(2);
        }
        pDisplay->clearDisplay();   // clears the screen and buffer
    }
}


PMS7003_framestruct currFrame;

void setup() {
    pinMode(wifiSwitch, INPUT_PULLUP);
    pinMode(flipSwitch, INPUT_PULLUP);
    pinMode(holdSwitch, INPUT_PULLUP);

    Serial.begin(57600);

    if (digitalRead(wifiSwitch) && !Particle.connected()) {
        Particle.connect();
    }

    // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
    updateOrientation(&display, &flipState, flipSwitch);
    display.clearDisplay();   // clears the screen and buffer
    display.display();
    currFrame.readCount = 0;
    delay(1000);
    Serial.println("-- Initializing...");
}


void loop () {
    if (digitalRead(holdSwitch)) { 
        pms7003_read(&currFrame, printbuf, printbufLen);
    }

    updateOrientation(&display, &flipState, flipSwitch);

    display.setTextColor(BLACK, WHITE); // 'inverted' text
    display.fillRect(0, 0, display.width(), 11, WHITE);

    display.setCursor(1, 2);
    display.setTextSize(1);
    if (currFrame.readCount > 9999999) {
        currFrame.readCount = 0;
    }
    snprintf(printbuf, sizeof(printbuf), "%s #%07d", currFrame.deviceType, currFrame.readCount);
    display.print(printbuf);
    
    display.setCursor(102, 2);
    if (digitalRead(holdSwitch)) {
        if (currFrame.dataGood) {
            display.drawRect(100, 1, 27, 9, BLACK);
            display.setTextColor(BLACK);
            snprintf(printbuf, sizeof(printbuf), "%s", " OK ");
        }
        else {
            display.fillRect(100, 1, 27, 9, BLACK);
            display.setTextColor(WHITE);
            snprintf(printbuf, sizeof(printbuf), "%s", "FAIL");
        }
    }
    else {
        display.fillRect(100, 1, 27, 9, BLACK);
        display.setTextColor(WHITE);
        snprintf(printbuf, sizeof(printbuf), "%s", "HOLD");
    }
    display.print(printbuf);

    if (currFrame.dataGood) {
        display.fillRect(0, 15, display.width(), 31, BLACK);
    
        display.setTextColor(WHITE);
        display.setCursor(1, 15);
        snprintf(printbuf, sizeof(printbuf), "PM1.0:%4d", currFrame.concPM1_0_amb);
        display.print(printbuf);
    
        display.setTextColor(WHITE);
        display.setCursor(66, 15);
        snprintf(printbuf, sizeof(printbuf), "PM2.5:%4d", currFrame.concPM2_5_amb);
        display.print(printbuf);
    
        display.setTextColor(WHITE);
        display.setCursor(1, 24);
        snprintf(printbuf, sizeof(printbuf), "PM10 :%4d", currFrame.concPM10_0_amb);
        display.print(printbuf);
    
        display.setTextColor(WHITE);
        display.setCursor(66, 24);
        snprintf(printbuf, sizeof(printbuf), " (ug/m^3)");
        display.print(printbuf);
    }
    display.display();

    delay(1000);
}



