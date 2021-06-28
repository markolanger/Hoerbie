/*========================================================================
  Bibliotheken
  ========================================================================*/
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Arduino.h>
#include "DFRobotDFPlayerMini.h"
#include <SPI.h>
#include <MFRC522.h>
#include <EEPROM.h>
#include <Preferences.h>
#include "Wire.h"
#include <FastLED.h>

/*========================================================================
  Pinout
  ========================================================================*/

#define ProjectName "Hoerbie"
const int resetPin = 22; // Reset pin NFC
const int ssPin = 21;    // Slave select pin NFC
#define DATA_PIN 2        //WS2812b
#define busyPin 4
//#define headphonePin 25
#define dfpMute 33
unsigned int last_Volume = 20;
unsigned int last_max_Volume = 28;
unsigned int max_Volume = last_max_Volume;
unsigned int akt_Volume = last_Volume;
/*========================================================================
  Debug
  ========================================================================*/

bool            debug = 0; // Auf 1 setzen um debug Informationen über die Serielle Schnittstelle zu erhalten.
bool debugTimeCompare = 0;
bool        debugWifi = 1;
bool        debugBoot = 0;
bool        debugGyro = 0;
bool       debugSetup = 0;
bool         debugLed = 0;
bool      debugWifiAP = 0;
bool             flag = 0;
bool   Hoerspielmodus = 0;

/*========================================================================
  Globale Variablen
  ========================================================================*/
Preferences preferences;
int timeout = 20;


//MPU-6050
const int MPU_addr = 0x68;                              // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int setupAcX = 10000;                                   // Next / Prev
int setupAcY = 15000;                                   // Pause/Config
int setupGyZ = 15000;                                   // Volume


//Webserver
unsigned long last_color = 0xFFFFFF;
int success2 = 0;
int success = 0;
int oldfolder = 1;
String TimerOFF = "00:00";
String TimerON = "00:00";
uint8_t TMR_OFF_HH, TMR_OFF_MM, TMR_ON_HH, TMR_ON_MM;
int TMR_OFF_REP = 0;
int TMR_ON_REP = 0;

bool TMP_OFFTIME = true;
bool TMP_ONTIME = true;
bool WakeUpLight = false;
bool SleepLight = false;
bool startSR = false;


//Sunrise
static uint16_t heatIndex = 0; // start out at 0
DEFINE_GRADIENT_PALETTE( sunrise_gp ) {
  0,     0,   0,   0,   //schwarz
  128,   240,   0,   0,   //rot
  224,   240, 240,   0,   //gelb
  255,   128, 128, 240    //sehr helles Blau
};

volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;

//NTP
WebServer server(80);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);


//NFC
// this object stores nfc tag data
struct nfcTagObject {
  uint32_t cookie;
  uint8_t version;
  uint8_t folder;
  uint8_t mode;
  uint8_t special;
  uint32_t color;
};

nfcTagObject myCard;
void resetCard(void);
bool readCard(nfcTagObject *nfcTag);
void setupCard(void);
static void nextTrack();
void startTimer(void);
void stoppTimer(void);
int voiceMenu(int numberOfOptions, int startMessage, int messageOffset, bool preview = false, int previewFromFolder = 0);
bool knownCard = false;
uint16_t numTracksInFolder;
uint16_t track;
MFRC522::MIFARE_Key key;
bool successRead;
byte sector = 1;
byte blockAddr = 4;
byte trailerBlock = 7;
MFRC522::StatusCode status;




//LED WS2812b
#define NUM_LEDS 1        //number of LEDs WS2812b
#define BRIGHTNESS 32     //brightness  (max 254)
#define LED_TYPE WS2812B   // I know we are using ws2812, but it's ok!
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];


//Buttons
bool ignorePauseButton = false;
bool ignoreUpButton = false;
bool ignoreDownButton = false;
bool ignoreConfigButton = false;
uint8_t numberOfCards = 0;

/*========================================================================
  Builder
  ========================================================================*/

MFRC522 mfrc522 = MFRC522(ssPin, resetPin); // Create instance
unsigned int Brightness = BRIGHTNESS;
hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  isrCounter++;
  lastIsrAt = millis();
  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  // It is safe to use digitalRead/Write here if you want to toggle an output
}



//DFR MP3========================================================================================================================
class Mp3Notify {
  public:
    static void OnError(uint16_t errorCode) {
      // see DfMp3_Error for code meaning
      Serial.println();
      Serial.print("Com Error ");
      Serial.println(errorCode);
    }
    static void OnPlayFinished(uint16_t track) {
      Serial.print("Track beendet");
      Serial.println(track);
      delay(100);
      nextTrack();
    }
    static void OnCardOnline(uint16_t code) {
      Serial.println(F("SD card online "));
    }
    static void OnCardInserted(uint16_t code) {
      Serial.println(F("SD card ready "));
    }
    static void OnCardRemoved(uint16_t code) {
      Serial.println(F("SD card ejected "));
    }
};
HardwareSerial mySoftwareSerial(2);
DFRobotDFPlayerMini myDFPlayer;
// void printDetail(uint8_t type, int value) {Serial.println(F("Blubberd ")};
// Leider kann das Modul keine Queue abspielen.
static void nextTrack() {
  if (knownCard == false)
    // Wenn eine neue Karte angelernt wird soll das Ende eines Tracks nicht
    // verarbeitet werden
    return;

  if (myCard.mode == 1) {
    Serial.println(F("Hörspielmodus ist aktiv"));;
  }
  if (myCard.mode == 2) {
    if (track >= numTracksInFolder || track < 0) {
      track = 0;
      Serial.println("track zurücksetzten ");
    }
    if (track != numTracksInFolder) {
      track = track + 1;
      myDFPlayer.playFolder(myCard.folder, track);
      flag = 1;
      Serial.print(F("Albummodus ist aktiv -> nächster Track: "));
      Serial.print(track);
    }
    else track = 1;
  }
  if (myCard.mode == 3) {
    track = random(1, numTracksInFolder + 1);
    Serial.print(F("Party Modus ist aktiv -> zufälligen Track spielen: "));
    Serial.println(track);
    flag = 1;
    myDFPlayer.playFolder(myCard.folder, track);
  }
  if (myCard.mode == 4) {
    Serial.println(F("Einzel Modus aktiv"));
  }
  if (myCard.mode == 5) {
    if (track != numTracksInFolder) {
      track = track + 1;
      Serial.print(F("Hörbuch Modus ist aktiv -> nächster Track und "
                     "Fortschritt speichern"));
      Serial.println(track);
      myDFPlayer.playFolder(myCard.folder, track);
      flag = 1;
      // Fortschritt im EEPROM abspeichern
      EEPROM.write(myCard.folder, track);
    } else
      // Fortschritt zurück setzen
      EEPROM.write(myCard.folder, 1);
  }
  delay(500);
}
static void previousTrack() {
  if (myCard.mode == 1) {
    Serial.println(F("Hörspielmodus ist aktiv -> Track von vorne spielen"));
    myDFPlayer.playFolder(myCard.folder, track);
    flag = 1;
  }
  if (myCard.mode == 2) {
    Serial.println(F("Albummodus ist aktiv -> vorheriger Track"));
    if (track != 1) {
      track = track - 1;
    }
    myDFPlayer.playFolder(myCard.folder, track);
    flag = 1;
  }
  if (myCard.mode == 3) {
    Serial.println(F("Party Modus ist aktiv -> Track von vorne spielen"));
    myDFPlayer.playFolder(myCard.folder, track);
    flag = 1;
  }
  if (myCard.mode == 4) {
    Serial.println(F("Einzel Modus aktiv -> Track von vorne spielen"));
    myDFPlayer.playFolder(myCard.folder, track);
    flag = 1;
  }
  if (myCard.mode == 5) {
    Serial.println(F("Hörbuch Modus ist aktiv -> vorheriger Track und "
                     "Fortschritt speichern"));
    if (track != 1) {
      track = track - 1;
    }
    myDFPlayer.playFolder(myCard.folder, track);
    flag = 1;
    // Fortschritt im EEPROM abspeichern
    EEPROM.write(myCard.folder, track);
  }
}
bool isPlaying() {
  return !digitalRead(busyPin);
}



// HTML Functions========================================================================================================================
//Funktion um die Antworten der HTML Seite auszuwerten
void handleRestart() {
  preferences.end();
  ESP.restart();
}
void handleSetup() {
  server.send ( 200, "text/html", SetupPage());
  if (server.args() > 0 ) { // Arguments were received
    for ( uint8_t i = 0; i < server.args(); i++ ) {

      Serial.print("Vom Server wurde folgendes empfangen: "); // Display the argument
      Serial.print(server.argName(i)); // Display the argument
      Serial.print("=");
      Serial.println(server.arg(i));

      if (server.argName(i) == "ssid" ) {
        preferences.clear();
        Serial.print("Speichere SSID: ");
        Serial.println(server.arg(i));
        preferences.putString("SSID", server.arg(i));
      }
      else if (server.argName(i) == "pw") {
        Serial.print("Speichere PW: ");
        Serial.println(server.arg(i));
        preferences.putString("Password", server.arg(i));
      }
    }
  }
}
void httpTimeJson() {
  //  String output = "{ \"uptime\":" + String(millis()); + "\","\"Volumenow\":" + akt_Volume; + " }";
  String output  = "{\"uptime\":\"" + String(millis()) + "\",\"VolumeNow\":\"" + akt_Volume + "\",\"VolumeMax\":\"" + max_Volume + "\",\"Card_Cookie\":\"" + myCard.cookie + "\",\"Card_Serial\":\"" + mfrc522.PICC_ReadCardSerial() + "\",\"Card_Folder\":\"" + myCard.folder + "\",\"Card_Mode\":\"" + myCard.mode + "\",\"Tmp\":\"" + Tmp + "\"}";
  server.send(200, "application/json", output);
}
void httpNotFound() {
  server.send(404, "text/plain", "404: Not found");
}
void handleRoot() {
  server.send ( 200, "text/html", getPage() );
  if (server.args() > 0 ) { // Arguments were received
    for ( uint8_t i = 0; i < server.args(); i++ ) {

      Serial.print("Vom Server wurde folgendes empfangen: "); // Display the argument
      Serial.print(server.argName(i)); // Display the argument
      Serial.print("=");
      Serial.println(server.arg(i));

      if (server.argName(i) == "appt-time-off" ) {

        TimerOFF = server.arg(i);
        char charBuf[TimerOFF.length() + 1];
        TimerOFF.toCharArray(charBuf, TimerOFF.length() + 1);
        TMR_OFF_HH = atof(strtok(charBuf, ":"));
        TMR_OFF_MM = atof(strtok(NULL, ":"));
        Serial.print("Die eingelesene Zeit für TimerOFF: ");
        Serial.print(TMR_OFF_HH);
        Serial.print(":");
        Serial.println(TMR_OFF_MM);
        TMR_OFF_REP = 0;
        TMP_OFFTIME = false;

      }
      else if (server.argName(i) == "cb_tmr_off") {
        TMR_OFF_REP = 1;
        Serial.println("Für den TimerOFF wurde eine Wiederholung eingestellt");
      }
      else if (server.argName(i) == "appt-time-on") {
        TimerON = server.arg(i);
        char charBuf[TimerON.length() + 1];
        TimerON.toCharArray(charBuf, TimerON.length() + 1);
        TMR_ON_HH = atof(strtok(charBuf, ":"));
        TMR_ON_MM = atof(strtok(NULL, ":"));
        Serial.print("Die eingelesene Zeit für TimerON: ");
        Serial.print(TMR_ON_HH);
        Serial.print(":");
        Serial.println(TMR_ON_MM);
        TMR_ON_REP = 0;
        TMP_ONTIME = false;
      }
      else if (server.argName(i) == "cb_tmr_on") {
        TMR_ON_REP = 1;
        Serial.println("Für den TimerON wurde eine Wiederholung eingestellt");
      }
      else if (server.argName(i) == "max_volume") {
        max_Volume = server.arg(i).toInt();
        Serial.println("Die maximale Lautstärke wurde geändert");
      }
      else if (server.argName(i) == "akt_volume") {
        if (server.arg(i).toInt() < max_Volume) {
          myDFPlayer.volume(server.arg(i).toInt());
          delay(200);
          akt_Volume = myDFPlayer.readVolume();
          server.send(200, "text/html", getPage());
        } else if (server.arg(i).toInt() > max_Volume) {
          myDFPlayer.volume(max_Volume);
          akt_Volume = max_Volume;
          myDFPlayer.advertise(902);
          delay(1650);
          myDFPlayer.stopAdvertise();
          delay(40);
          server.send(200, "text/html", getPage());
          Serial.println("!MAXVol+");
        } else
        {
          myDFPlayer.volume(max_Volume);
          akt_Volume = max_Volume;
          myDFPlayer.advertise(902);
          delay(1650);
          myDFPlayer.stopAdvertise();
          delay(40);
          server.send(200, "text/html", getPage());
          Serial.println("!MAXVol+");
        }
        delay(200);
        myDFPlayer.advertise(akt_Volume);
        delay(1400);
        Serial.println("Die aktuelle Lautstärke wurde geändert");
      }
      else if (server.argName(i) == "LED_color") {
        Serial.println("Die Farbe der LEDs wird geändert: " + server.arg(i));
        String Color = server.arg(i);
        Color.remove(0, 1);
        char *ptr;
        char charBuf[Color.length() + 1];
        Color.toCharArray(charBuf, Color.length() + 1);
        unsigned long col = strtol(charBuf, &ptr, 16);
        fill_solid(leds, NUM_LEDS, col); // Farbe aller LEDs ändern
        FastLED.show();
        if (debugLed) {
          Serial.print("Color: ");  //?
          Serial.println(col);
          Serial.println(charBuf);
          Serial.println(Color);
        }
      }
      else if (server.argName(i) == "LED_bri") {
        Serial.println("Die Helligkeit der LEDs wird geändert ");
        Serial.println("Helligkeit = " + server.arg(i));
        FastLED.setBrightness(server.arg(i).toInt());
        FastLED.show();
      }
      else if (server.argName(i) == "cb_SleepLight_on") {
        //CODE HERE
        if (server.arg(i).toInt() == 1) {
          SleepLight = true;
        }
      }
      else if (server.argName(i) == "cb_SleepLight_off") {
        if (server.arg(i).toInt() == 0) {
          SleepLight = false;
        }
      }
      else if (server.argName(i) == "cb_WakeUpLight_on") {
        if (server.arg(i).toInt() == 1) {
          WakeUpLight = true;
        }

      }
      else if (server.argName(i) == "cb_WakeUpLight_off") {
        if (server.arg(i).toInt() == 0) {
          WakeUpLight = false;
        }
      }
      else if (server.argName(i) == "rfid-dir") {
        myCard.folder = server.arg(i).toInt();
      }
      else if (server.argName(i) == "rfid-mode") {
        myCard.mode = server.arg(i).toInt();
      }
      else if (server.argName(i) == "rfid-file") {
        myCard.special = server.arg(i).toInt();
      }
      else if (server.argName(i) == "rfid-color") {
        myCard.color = server.arg(i).toInt();
        success2 = 0;
        FastLED.setBrightness(BRIGHTNESS);  //Helligkeit einstellen
        fill_solid(leds, NUM_LEDS, CRGB::Red); // Farbe aller LEDs ändern
        if (debugLed) {
          Serial.println("LED: Red");
        }
        FastLED.show();
        while (success2 == 0) {
          if (  mfrc522.PICC_IsNewCardPresent()) {
            if (  mfrc522.PICC_ReadCardSerial()) {
              MFRC522::MIFARE_Key key;
              for (byte i = 0; i < 6; i++) key.keyByte[i] = 0xFF;
              status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, trailerBlock, &key, &(mfrc522.uid));
              if (status != MFRC522::STATUS_OK) {
                Serial.print(F("PCD_Authenticate() failed: "));
                Serial.println(mfrc522.GetStatusCodeName(status));
              }
              else Serial.println(F("PCD_Authenticate() success: "));
              Serial.print(F("Reading data from block "));
              Serial.print(blockAddr);
              Serial.println(F(" ..."));
              byte buffer[20];
              byte size = sizeof(buffer);
              bool returnValue = true;
              status = (MFRC522::StatusCode)mfrc522.MIFARE_Read(blockAddr, buffer, &size);
              if (status != MFRC522::STATUS_OK) {
                returnValue = false;
                Serial.print(F("MIFARE_Read() failed: "));
                Serial.println(mfrc522.GetStatusCodeName(status));
              }
              Serial.print(F("Data in block "));
              Serial.print(blockAddr);
              Serial.println(F(":"));
              dump_byte_array(buffer, 20);
              Serial.println();
              Serial.println();
              switch (myCard.color) {
                case 4:
                  myCard.color = CRGB::LawnGreen;
                  break;
                case 3:
                  myCard.color = CRGB::Yellow;
                  break;
                case 6:
                  myCard.color = CRGB::White;
                  break;
                case 7:
                  myCard.color = CRGB::Pink;
                  break;
                case 2:
                  myCard.color = CRGB::Orange;
                  break;
                case 5:
                  myCard.color = CRGB::LightSkyBlue;
                  break;
                case 1:
                  myCard.color = CRGB::Black;
                  break;
              }
              EEPROM.write(myCard.folder, 1);
              uint8_t color[4];
              color[0] = (myCard.color >> 0)  & 0xFF;
              color[1] = (myCard.color >> 8)  & 0xFF;
              color[2] = (myCard.color >> 16) & 0xFF;
              color[3] = (myCard.color >> 24) & 0xFF;
              byte Buffer[20] = {0x13, 0x37, 0xb3, 0x47, // 0x1337 0xb347 magic cookie to identify our nfc tags
                                 0x01,                   // version 1
                                 myCard.folder,          // the folder picked by the user
                                 myCard.mode,            // the playback mode picked by the user
                                 myCard.special,         // track or function for admin cards
                                 color[3],
                                 color[2],
                                 color[1],
                                 color[0],
                                 0x00,
                                 0x00,
                                 0x00,
                                 0x00
                                };

              status = mfrc522.MIFARE_Write(blockAddr,  Buffer, 16);
              if (status != MFRC522::STATUS_OK) {
                Serial.print(F("MIFARE_Write() failed! "));
                Serial.println(mfrc522.GetStatusCodeName(status));
              } else {
                Serial.print(F("MIFARE_Write() "));
                Serial.println(mfrc522.GetStatusCodeName(status));
              }
              status = (MFRC522::StatusCode)mfrc522.MIFARE_Read(blockAddr, buffer, &size);
              if (status != MFRC522::STATUS_OK) {
                returnValue = false;
                Serial.print(F("MIFARE_Read() failed: "));
                Serial.println(mfrc522.GetStatusCodeName(status));
              }
              Serial.print(F("Data in block "));
              Serial.print(blockAddr);
              Serial.println(F(":"));
              dump_byte_array(buffer, 20);
              Serial.println();
              Serial.println();


              mfrc522.PICC_HaltA();
              mfrc522.PCD_StopCrypto1();
              success2 = +2;
              fill_solid(leds, NUM_LEDS, CRGB::Green); // Farbe aller LEDs ändern
              if (debugLed) {
                Serial.println("LED: Green");
              }
              FastLED.show();
            }
          }
        }
      }
    }
  }
}

//HTML Actions========================================================================================================================
//Ab hier Aktionen welche bei drücken eines Button auf der HTML Seite ausgelöst werden
void handlePrev() {
  Serial.println("handlePrev");
  myDFPlayer.previous();
  flag = 1;
  server.send(200, "text/html", getPage());
}
void handlePlay() {
  Serial.println("handlePlay");
  myDFPlayer.start();
  flag = 1;
  server.send(200, "text/html", getPage());
}
void handlePause() {
  Serial.println("handlePause");
  myDFPlayer.pause();
  server.send(200, "text/html", getPage());
}
void handleNext() {
  Serial.println("handleNext");
  myDFPlayer.next();
  flag = 1;
  server.send(200, "text/html", getPage());
}
void handleVol_up() {
  if (akt_Volume < max_Volume) {
    Serial.println("handleVol+");
    myDFPlayer.volumeUp();
    delay(1000);
    akt_Volume = myDFPlayer.readVolume();
    server.send(200, "text/html", getPage());
  } else {
    akt_Volume = myDFPlayer.readVolume();
    myDFPlayer.advertise(902);
    delay(1650);
    myDFPlayer.stopAdvertise();
    delay(40);
    server.send(200, "text/html", getPage());
    Serial.println("!MAXVol+");
  }
  delay(200);
  myDFPlayer.advertise(akt_Volume);
}
void handleVol_down() {
  Serial.println("handleVol-");
  myDFPlayer.volumeDown();
  delay(1000);
  akt_Volume = myDFPlayer.readVolume();
  delay(200);
  myDFPlayer.advertise(akt_Volume);
  server.send(200, "text/html", getPage());
}
void handleEQ_NORM() {
  Serial.println("handleEQ_Norm");
  myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
  server.send(200, "text/html", getPage());
}
void handleEQ_POP() {
  Serial.println("handleEQ_POP");
  myDFPlayer.EQ(DFPLAYER_EQ_POP);
  server.send(200, "text/html", getPage());
}
void handleEQ_ROCK() {
  Serial.println("handleEQ_ROCK");
  myDFPlayer.EQ(DFPLAYER_EQ_ROCK);
  server.send(200, "text/html", getPage());
}
void handleEQ_CLASSIC() {
  Serial.println("handleEQ_CLASSIC");
  myDFPlayer.EQ(DFPLAYER_EQ_CLASSIC);
  server.send(200, "text/html", getPage());
}
void handleEQ_BASS() {
  Serial.println("handleEQ_BASE");
  myDFPlayer.EQ(DFPLAYER_EQ_BASS);
  server.send(200, "text/html", getPage());
}
void handleEQ_JAZZ() {
  Serial.println("handleEQ_JAZZ");
  myDFPlayer.EQ(DFPLAYER_EQ_JAZZ);
  server.send(200, "text/html", getPage());
}


//Sonnenaufgang Simulation========================================================================================================================
void sunrise() {
  if (debug) {
    Serial.println("sunrise() wird ausgeführt");
  }
  CRGBPalette256 sunrisePal = sunrise_gp;
  CRGB color = ColorFromPalette(sunrisePal, heatIndex);
  // fill the entire strip with the current color
  fill_solid(leds, NUM_LEDS, color);
  FastLED.setBrightness(200);
  FastLED.show();
  if (debugLed) {
    Serial.print("heatIndex: ");
    Serial.println(heatIndex);
  }
  heatIndex++;
  if (heatIndex == 255) {
    heatIndex = 0;
    startSR = false;
  }


}





//Timer function========================================================================================================================
//Funktion um die Ein- /Ausschalttimer auszuwerten


void TimeCompare() {

  if (debugTimeCompare) Serial.println("TimeCompare() wird ausgeführt");
  timeClient.update();
  int NTP_HH = timeClient.getHours();
  int NTP_MM = timeClient.getMinutes();
  if (debugTimeCompare) Serial.println("Die aktuelle NTP Zeit:" + String(NTP_HH) + ":" + String(NTP_MM) + " | Schlafen gehen: " +  TMR_OFF_MM + ":" + TMR_OFF_HH);



  if ((TMR_OFF_MM == NTP_MM) and (TMP_OFFTIME == false) and (TMR_OFF_HH == NTP_HH)) {
    //Abschalttimer
    myDFPlayer.pause();
    delay(100);
    myDFPlayer.playMp3Folder(261); //Verabschiedung spielen
    delay(10000);
    while (isPlaying())
      myDFPlayer.stop();
    myDFPlayer.pause();
    if (TMR_OFF_REP == 0) {
      TMP_OFFTIME = true;
    }
    Serial.println("Die Wiedergabe wurde durch den OFF-Timer gestoppt.");
    myDFPlayer.pause();
    delay(1000);
    myDFPlayer.pause();
  }
  else if ((TMR_ON_MM == NTP_MM) and (TMP_ONTIME == false) and (TMR_ON_HH == NTP_HH)) {
    myDFPlayer.playMp3Folder(262); //Begrüßung spielen
    if (TMR_ON_REP == 0) {
      TMP_ONTIME = true;
    }
    if (WakeUpLight == true) {
      startSR = true;
    }
    Serial.println("Die Wiedergabe wurde durch den ON-Timer gestartet.");
  }
}
//WIFI implementation ========================================================================================================================
int WiFi_RouterNetworkConnect(char* txtSSID, char* txtPassword)
{
  int success = 1;
  WiFi.begin(txtSSID, txtPassword);
  int WiFiConnectTimeOut = 0;
  while ((WiFi.status() != WL_CONNECTED) && (WiFiConnectTimeOut < 10))
  {
    delay(1000);
    WiFiConnectTimeOut++;
  }

  if (WiFi.status() != WL_CONNECTED)
  {
    success = -1;
  }
  MDNS.setInstanceName(ProjectName);
  Serial.println(WiFi.localIP());
  if (debugWifi) Serial.print("WiFi Connected ");
  if (debugWifi) Serial.println("Start mDNS");
  if (!MDNS.begin(ProjectName)) {
    Serial.println("Error setting up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  if (debugWifi) Serial.print("mDNS responder started: ");
  if (debugWifi) Serial.print(ProjectName);
  if (debugWifi) Serial.println(".local");
  return success;
}

// Disconnect from router network and return 1 (success) or -1 (no success)
int WiFi_RouterNetworkDisconnect()
{
  int success = -1;
  WiFi.disconnect();
  int WiFiConnectTimeOut = 0;
  while ((WiFi.status() == WL_CONNECTED) && (WiFiConnectTimeOut < 10))
  {
    delay(1000);
    WiFiConnectTimeOut++;
  }
  // not connected
  if (WiFi.status() != WL_CONNECTED)
  {
    success = 1;
  }
  if (debugWifi)Serial.println("Disconnected.");
  return success;
}


// Initialize Soft Access Point with ESP32
// ESP32 establishes its own WiFi network, one can choose the SSID
int WiFi_AccessPointStart(char* AccessPointNetworkSSID)
{
  WiFi.mode(WIFI_AP);
  IPAddress apIP(192, 168, 4, 1);    // Hier wird IP bestimmt
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP("Hoerbie_Setup");  // Name of the Access Point
  delay(500);
  //Serial.println("Start AP - 192.168.4.1");
  if (debugWifi) {
    Serial.println("Start AP");
  }
  if (debugWifi) {
    Serial.print("IP Adress: ");
  }
  if (debugWifi) {
    Serial.println(WiFi.softAPIP());
  }


  myDFPlayer.playMp3Folder(263);              //Ansage das ein Access-Point geöffnet wird

  server.on("/", handleSetup);                // INI wifimanager Index Webseite senden
  server.on("/restart", []() {                // INI wifimanager Index Webseite senden
    server.send(200, "text/plain", "Reset on going");
    handleRestart();
  });

  server.begin();
  if (debugWifi)  Serial.println("HTTP Server started");
  MDNS.addService("http", "tcp", 80);
  while (1) {
    server.handleClient();                    // Wird endlos ausgeführt damit das WLAN Setup erfolgen kann
    if (AcY > setupAcY)break;                 //Bricht die Warteschleife ab sobald die Play/Pause Taste gedrückt wurde
  }
  return 1;
}


// SETUP ====================================================================================================================================================================================
void setup() {
  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();
  timer = timerBegin(0, 80, true);
  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);
  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter)
  timerAlarmWrite(timer, 1000000, true);

  //===================================================================================
  FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);  //Helligkeit einstellen
  fill_solid(leds, NUM_LEDS, CRGB::Green); // Farbe aller LEDs ändern
  if (debugLed) {
    Serial.println("LED: Green");
  }
  FastLED.show();
  //===================================================================================

  Serial.begin(115200);
  SPI.begin();
  Wire.begin(0, 15);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);                                                                   // set to zero (wakes up the MPU-6050)
  mySoftwareSerial.begin(9600, SERIAL_8N1, 16, 17);  // speed, type, RX, TX
  randomSeed(analogRead(32)); // Zufallsgenerator initialisieren
  mfrc522.PCD_Init();
  mfrc522.PCD_DumpVersionToSerial();
  pinMode(dfpMute, OUTPUT);
  digitalWrite(dfpMute, LOW);
  //  pinMode(headphonePin, INPUT_PULLUP;
  pinMode(busyPin, INPUT);

  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }
  Serial.println(ProjectName);
  Serial.println("Marko Langer");

  Serial.println();
  if (debugBoot) {
    Serial.println(F("DFRobot DFPlayer Mini"));
  }
  if (debugBoot) {
    Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  }
  preferences.clear();
  myDFPlayer.begin(mySoftwareSerial, false, true);

  myDFPlayer.setTimeOut(500); //Set serial communictaion time out 500ms
  delay(200);
  myDFPlayer.volume(akt_Volume);
  delay(100);
  myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
  delay(100);
  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);
  myDFPlayer.outputSetting(true, 15); //output setting, enable the output and set the gain to 15
  //----Read imformation----
  if (debugBoot) Serial.println("");
  if (debugBoot) Serial.print(F("readState: "));
  if (debugBoot) Serial.println(myDFPlayer.readState()); //read mp3 state
  if (debugBoot) Serial.print(F("readVolume: "));
  if (debugBoot) Serial.println(myDFPlayer.readVolume()); //read current volume
  if (debugBoot) Serial.print(F("readEQ: "));
  if (debugBoot) Serial.println(myDFPlayer.readEQ()); //read EQ setting
  if (debugBoot) Serial.print(F("readFileCounts: "));
  if (debugBoot) Serial.println(myDFPlayer.readFileCounts()); //read all file counts in SD card
  if (debugBoot) Serial.print(F("readCurrentFileNumber: "));
  if (debugBoot) Serial.println(myDFPlayer.readCurrentFileNumber()); //read current play file number
  if (debugBoot) Serial.println(F("readFileCountsInFolder: "));
  if (debugBoot) Serial.println(myDFPlayer.readFileCountsInFolder(3)); //read fill counts in folder SD:/03
  delay(200);
  myDFPlayer.volume(akt_Volume);
  delay(200);
  myDFPlayer.playMp3Folder(260);


  preferences.begin("my-wifi", false);
  if (debugWifiAP) {
    WiFi.mode(WIFI_AP_STA);
  }
  // takeout 2 Strings out of the Non-volatile storage
  String strSSID = preferences.getString("SSID", "");
  String strPassword = preferences.getString("Password", "");
  // convert it to char*
  char* txtSSID = const_cast<char*>(strSSID.c_str());
  char* txtPassword = const_cast<char*>(strPassword.c_str());   // https://coderwall.com/p/zfmwsg/arduino-string-to-char
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to SSID: ");
  Serial.println(txtSSID);
  if (debugWifi)   Serial.print(" with the following PW:  ");
  if (debugWifi)   Serial.println(txtPassword);
  if (debugWifi)   Serial.println(WiFi.localIP());
  
  // try to connect to the LAN
  success = WiFi_RouterNetworkConnect(txtSSID, txtPassword);
  if (success == 1)
  {
    fill_solid(leds, NUM_LEDS, CRGB::Blue); // Farbe aller LEDs ändern
    if (debugLed) {
      Serial.println("LED: blue");
    }
  }
  else
  {
    fill_solid(leds, NUM_LEDS, CRGB::Red); // Farbe aller LEDs ändern
    if (debugLed) {
      Serial.println("LED: RED");
    }
  }
  FastLED.show();

  // Start access point"
  if (success == -1) {
    WiFi_AccessPointStart(ProjectName);
    Serial.print(success);
    Serial.println(" | AP  started");
  }

  if (debugWifi) Serial.println ( "HTTP server started" );


  //NTP Client starten, das Offset zur Empfangenen Zeit einstellen
  if (success == 1)timeClient.begin();
  if (success == 1)timeClient.setTimeOffset(+3600); //+1h Offset
  if (success == 1)timeClient.update();

  //Verweise für den Empfang von HTML Client informationen
  server.on ( "/", handleRoot );
  server.on ("/play", handlePlay);
  server.on ("/pause", handlePause);
  server.on ("/prev", handlePrev);
  server.on ("/next", handleNext);
  server.on ("/vol+", handleVol_up);
  server.on ("/vol-", handleVol_down);
  server.on ("/eq_base", handleEQ_BASS);
  server.on ("/eq_pop", handleEQ_POP);
  server.on ("/eq_rock", handleEQ_ROCK);
  server.on ("/eq_classic", handleEQ_CLASSIC);
  server.on ("/eq_jazz", handleEQ_JAZZ);
  server.on ("/eq_norm", handleEQ_NORM);
  server.on("/setup", handleSetup);
  server.on("/time.json", httpTimeJson);
  server.onNotFound(httpNotFound);
  //  server.on("/akt_Volume", handleVolume);
  server.on("/restart", []() {
    server.send(200, "text/plain", "Reset on going");
    handleRestart();
  });


  server.begin();
  if (success == 1)startTimer();
  if (debugBoot) Serial.println ( "===============////////////// ================" );
  if (debugBoot) Serial.println ( "=============== / ENDE / ================" );
  if (debugBoot) Serial.println ( "===============////////////// ================" );
}


//==============SETUP ENDE================================

// LOOP ====================================================================================================================================================================================
void loop() {
  Serial.println("Loop");
  do {


    if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE) { //Timer Interrupt Routine
      uint32_t isrCount = 0, isrTime = 0;
      // Read the interrupt count and time
      portENTER_CRITICAL(&timerMux);
      isrCount = isrCounter;
      isrTime = lastIsrAt;
      portEXIT_CRITICAL(&timerMux);
      //Ab hier Funktionen für Timer

      if (success == 1) TimeCompare(); //Abfrage der Zeit im Sekundentakt
      if (startSR == true) sunrise();
    }
    server.handleClient();


//      Serial.print("myDFPlayer.available: ");
//      Serial.print(myDFPlayer.available());
//      Serial.print(" | isPlaying:");
//      Serial.print(isPlaying());
//      Serial.print(" | oldfolder: ");
//      Serial.println(oldfolder);
    


    if (!isPlaying()) {
      if (myDFPlayer.available()) {
        printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
      }
    }
    if (isPlaying()) {
      if (!flag == 0) {
//        Serial.println("Advertising Track");
//        delay(10);
//        myDFPlayer.advertise(track);
        flag = 0;
      }
    }



    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers

    AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    //    if (debugGyro) {
    //      Serial.print("AcX = "); Serial.print(AcX);
    //      Serial.print(" | AcX = "); Serial.print(AcX);
    //      Serial.print(" | AcZ = "); Serial.print(AcZ);
    //      Serial.print(" | Tmp = "); Serial.print(Tmp / 340.00 + 36.53); //equation for temperature in degrees C from datasheet
    //      Serial.print(" | GyX = "); Serial.print(GyX);
    //      Serial.print(" | GyY = "); Serial.print(GyY);
    //      Serial.print(" | GyZ = "); Serial.print(GyZ);
    //      Serial.print(" | Tmp = "); Serial.println(Tmp / 340.00 + 32.53);
    //    }
    if (AcX > setupAcX && ignoreUpButton == 0) {
      ignoreUpButton = 1;
      if (debugGyro) {
        Serial.print("Left: Locked | AcX = ");
        Serial.println(AcX);
      }
      Serial.println(F("previousTrack"));
      previousTrack();
    }
    if (AcX < (setupAcX / 2) && AcX > 1 && ignoreUpButton == 1) {
      ignoreUpButton = 0;
      delay(10);
      if (debugGyro) {
        Serial.print("Left: Unlocked | AcX = ");
        Serial.println(AcX);
      }
    }
    if (AcX < (setupAcX * -1) && ignoreDownButton == 0) {
      ignoreDownButton = 1;
      if (debugGyro) {
        Serial.print("Right: Locked | AcX = ");
        Serial.println(AcX);
      }
      Serial.println(F("nextTrack"));
      nextTrack();
    }
    if (AcX > ((setupAcX * -1) / 2) && AcX < -10 && ignoreDownButton == 1) {
      ignoreDownButton = 0;
      delay(10);
      if (debugGyro) {
        Serial.print("Right: Unlocked | AcX = ");
        Serial.println(AcX);
      }
    }
    if (AcY < (setupAcY * -1) && ignorePauseButton == 0) {
      ignorePauseButton = 1;
      if (debug) {
        Serial.print("Forward: Locked | AcY = ");
        Serial.println(AcY);
      }
      if (isPlaying()) {
        myDFPlayer.pause();
        Serial.println("Pause");
      }
      if (!isPlaying()) {
        myDFPlayer.start();
        Serial.println("Start");
      }
    }
    if (AcY > ((setupAcY * -1) / 2) && AcY < -10 && ignorePauseButton == 1) {
      ignorePauseButton = 0;
      if (debugGyro) {
        Serial.print("Forward: Unlocked | AcY = ");
        Serial.println(AcY);
      }
    }
    if (GyZ > setupGyZ) {
      if (myDFPlayer.readVolume() <= max_Volume) {
        delay(200);
        myDFPlayer.volumeUp();
        delay(200);
        akt_Volume = myDFPlayer.readVolume();
        Serial.print("volumeUp | akt_Volume: ");
        Serial.println(akt_Volume);
        delay(200);
        myDFPlayer.advertise(akt_Volume);
        delay(200);
        if (debugGyro) {
          Serial.print("volumeUp | GyZ = ");
          Serial.println(GyZ);
        }
      }



      if (myDFPlayer.readVolume() > max_Volume) {
        myDFPlayer.volume(max_Volume);
        akt_Volume = max_Volume;
        myDFPlayer.advertise(902);
        delay(1650);
        myDFPlayer.stopAdvertise();
        delay(300);
        myDFPlayer.advertise(akt_Volume);
        if (debugGyro) {
          Serial.print("Volume at max!: ");
          Serial.println(myDFPlayer.readVolume());
        }
      }
    }


    if (GyZ < (setupGyZ * -1)) {
      if (myDFPlayer.readVolume() != 0) {
        myDFPlayer.volumeDown();
        myDFPlayer.volumeDown();
        delay(50);
        akt_Volume = myDFPlayer.readVolume();
        myDFPlayer.advertise(akt_Volume);
        delay(200);
        if (debugGyro) {
          Serial.print("volumeDown | GyZ = ");
          Serial.println(GyZ);
        }
      }
    }

  } while (!mfrc522.PICC_IsNewCardPresent()); //wenn keine RFID Karte aufgelegt wurde

  // --> RFID Karte wurde aufgelegt

  if (!mfrc522.PICC_ReadCardSerial())
    return;

  if (readCard(&myCard) == true) {
    if (myCard.cookie == 322417479 && myCard.folder != 0 && myCard.mode != 0) {

      knownCard = true;
      myDFPlayer.readFileCountsInFolder(myCard.folder);
      numTracksInFolder = myDFPlayer.readFileCountsInFolder(myCard.folder);

      Serial.print("num Tracks Folder: ");
      Serial.print(numTracksInFolder);
      Serial.print(" | myCard.mode: ");
      Serial.println(myCard.mode);


//      // Hörspielmodus: eine zufällige Datei aus dem Ordner
//      if (myCard.mode == 1) {
//        Hoerspielmodus = 0;
//        Serial.print(F("Hörspielmodus -> zufälligen Track wiedergeben "));
//        track = random(1, numTracksInFolder + 1);
//        Serial.print(myCard.folder);
//        Serial.print(" , ");
//        Serial.println(track);
//        Serial.println(" ");
//        myDFPlayer.playFolder(myCard.folder, track);
//        flag = 1;
//        fill_solid(leds, NUM_LEDS, myCard.color); // Farbe aller LEDs ändern
//        FastLED.show();
//        if (debugLed) {
//          Serial.print("myCard.color: ");
//          Serial.println(myCard.color);
//        }
//      }
//      // Album Modus: kompletten Ordner spielen
//      if (myCard.mode == 2) {
//        if (Hoerspielmodus == 0)
//        {
//          track = 0;
//          Hoerspielmodus = 1;
//        }
//        if (myCard.folder != oldfolder)
//        {
//          track = 0;
//          oldfolder = myCard.folder; // erkenne ob sich der ordner geändert hat und resete den track counter.
//          Serial.print("hier!");
//        }
//        if (track >= numTracksInFolder || track < 0) {
//          track = 0;
//          Serial.println("track zurücksetzten ");
//        }
//        oldfolder = myCard.folder; // erkenne ob sich der ordner geändert hat und resete den track counter.
//
//
//        Serial.println(F("Album Modus -> kompletten Ordner wiedergeben"));
//        track = track + 1;
//        myDFPlayer.playFolder(myCard.folder, track);
//        flag = 1;
//
//        fill_solid(leds, NUM_LEDS, myCard.color); // Farbe aller LEDs ändern
//        FastLED.show();
//        if (debugLed) {
//          Serial.print("myCard.color: ");
//          Serial.println(myCard.color);
//        }
//      }
//      // Party Modus: Ordner in zufälliger Reihenfolge
//      if (myCard.mode == 3) {
//        Hoerspielmodus = 0;
//        Serial.println(
//          F("Party Modus -> Ordner in zufälliger Reihenfolge wiedergeben"));
//        track = random(1, numTracksInFolder + 1);
//        Serial.print(myCard.folder);
//        Serial.print(" , ");
//        Serial.println(track);
//        Serial.println(" ");
//        myDFPlayer.playFolder(myCard.folder, track);
//        flag = 1;
//        fill_solid(leds, NUM_LEDS, myCard.color); // Farbe aller LEDs ändern
//        if (debugLed) {
//          Serial.print("myCard.color: ");
//          Serial.println(myCard.color);
//        }
//      }
      // Einzel Modus: eine Datei aus dem Ordner abspielen
      if (myCard.mode == 4) {
        Hoerspielmodus = 0;
        Serial.println(
          F("Einzel Modus -> eine Datei aus dem Odrdner abspielen"));
        track = myCard.special;
        myDFPlayer.playFolder(myCard.folder, track);
        flag = 1;
        Serial.println(track);
        Serial.print("myDFPlayer.playFolder(myCard.folder, track)");
        Serial.print(myCard.folder);
        Serial.print(" , ");
        Serial.println(track);
        Serial.println(" ");
        fill_solid(leds, NUM_LEDS, myCard.color); // Farbe aller LEDs ändern
        FastLED.show();
        if (debugLed) {
          Serial.print("myCard.color: ");
          Serial.println(myCard.color);
        }
      }
      /* // Hörbuch Modus: kompletten Ordner spielen und Fortschritt merken
        if (myCard.mode == 5) {
         Serial.println(F("Hörbuch Modus -> kompletten Ordner spielen und "
                          "Fortschritt merken"));
         if (Hoerspielmodus == 0)
         {
           track = 0;
           Hoerspielmodus = 1;
         }
         track = track + 1;
         myDFPlayer.playFolder(myCard.folder, track);
         flag = 1;
         fill_solid(leds, NUM_LEDS, myCard.color); // Farbe aller LEDs ändern
         FastLED.show();
         if (debugLed) {
           Serial.print("myCard.color: ");
           Serial.println(myCard.color);
         }
        }*/
    }

    // Neue Karte konfigurieren
    else {
      knownCard = false;
      myDFPlayer.pause();
      myDFPlayer.playMp3Folder(300);
      //setupCard();

    }
  }
  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();

}

bool readCard(nfcTagObject * nfcTag) {
  bool returnValue = true;
  // Show some details of the PICC (that is: the tag/card)
  Serial.print(F("Card UID:"));
  dump_byte_array(mfrc522.uid.uidByte, mfrc522.uid.size);
  Serial.println();
  Serial.print(F("PICC type: "));
  MFRC522::PICC_Type piccType = mfrc522.PICC_GetType(mfrc522.uid.sak);
  Serial.println(mfrc522.PICC_GetTypeName(piccType));

  byte buffer[18];
  byte size = sizeof(buffer);

  // Authenticate using key A
  Serial.println(F("Authenticating using key A..."));
  status = (MFRC522::StatusCode)mfrc522.PCD_Authenticate(
             MFRC522::PICC_CMD_MF_AUTH_KEY_A, trailerBlock, &key, &(mfrc522.uid));
  if (status != MFRC522::STATUS_OK) {
    returnValue = false;
    Serial.print(F("PCD_Authenticate() failed: "));
    Serial.println(mfrc522.GetStatusCodeName(status));
    return returnValue;
  }

  // Show the whole sector as it currently is
  Serial.println(F("Current data in sector:"));
  mfrc522.PICC_DumpMifareClassicSectorToSerial(&(mfrc522.uid), &key, sector);
  Serial.println();

  // Read data from the block
  Serial.print(F("Reading data from block "));
  Serial.print(blockAddr);
  Serial.println(F(" ..."));
  status = (MFRC522::StatusCode)mfrc522.MIFARE_Read(blockAddr, buffer, &size);
  if (status != MFRC522::STATUS_OK) {
    returnValue = false;
    Serial.print(F("MIFARE_Read() failed: "));
    Serial.println(mfrc522.GetStatusCodeName(status));
  }
  Serial.print(F("Data in block "));
  Serial.print(blockAddr);
  Serial.println(F(":"));
  dump_byte_array(buffer, 20);
  Serial.println();
  Serial.println();

  uint32_t tempCookie;
  tempCookie = (uint32_t)buffer[0] << 24;
  tempCookie += (uint32_t)buffer[1] << 16;
  tempCookie += (uint32_t)buffer[2] << 8;
  tempCookie += (uint32_t)buffer[3];

  uint32_t tempColor;
  tempColor = (uint32_t)buffer[8] << 24;
  tempColor += (uint32_t)buffer[9] << 16;
  tempColor += (uint32_t)buffer[10] << 8;
  tempColor += (uint32_t)buffer[11];

  nfcTag->cookie = tempCookie;
  nfcTag->version = buffer[4];
  nfcTag->folder = buffer[5];
  nfcTag->mode = buffer[6];
  nfcTag->special = buffer[7];
  nfcTag->color = tempColor;

  return returnValue;
}
/**
   Helper routine to dump a byte array as hex values to Serial.
*/
void dump_byte_array(byte * buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
  }
}

void startTimer() {
  // Start an alarm
  timerAlarmEnable(timer);
}

void stoppTimer() {
  timerEnd(timer);
  timer = NULL;
}

void printDetail(uint8_t type, int value) {
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      Mp3Notify::OnPlayFinished(track);
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}
String getPage() {
  String page = "";
  page += "<!DOCTYPE html>";
  page += "<html lang='de'>";
  page += "<head>";
  page += "  <title>";
  page += ProjectName;
  page += "   Interface</title>";
  page += "<meta charset='UTF-8'>";
  page += "<meta name='theme-color' content='#4CAF50'>";
  page += "<meta name='viewport' content='width=device-width, initial-scale=1.0, user-scalable=yes'>";
  page += "<meta name='author' content='Marko Langer'>";
  page += "<link href='https://fonts.googleapis.com/icon?family=Material+Icons' rel='stylesheet'>";
  page += "    <script type='application/javascript'>";
  page += "    var xmlhttp = new XMLHttpRequest();";
  page += "    var url = '/time.json'; function updateStart()";
  page += "    {";
  page += "      xmlhttp.onreadystatechange = function()";
  page += "      {";
  page += "        if (this.readyState == 4)";
  page += "        {";
  page += "          if(this.status == 200)";
  page += "      {";
  page += "        try";
  page += "      {";
  page += "    var data = JSON.parse(this.responseText); updateView(data); } catch (e)";
  page += "    {";
  page += "    console.log('Update failed: ' + e); console.log(this.responseText);";
  page += "  }";
  page += "  }";
  page += "    window.setTimeout(updateStart, 500);";
  page += "  }";
  page += "  }; xmlhttp.open('GET', url, true);";
  page += "    xmlhttp.send(); } function updateView(data)";
  page += "    {";
  page += "    document.getElementById('time-ms').innerHTML = Math.round(data.uptime/1000);";
  page += "    document.getElementById('VolumeN').innerHTML = (data.VolumeNow);";
  page += "    document.getElementById('VolumeM').innerHTML = (data.VolumeMax);";
  page += "    document.getElementById('CCookie').innerHTML = (data.Card_Cookie);";
  page += "    document.getElementById('CSerial').innerHTML = (data.Card_Serial);";
  page += "    document.getElementById('CFolder').innerHTML = (data.Card_Folder);";
  page += "    document.getElementById('CMode').innerHTML = (data.Card_Mode);";
  page += "    document.getElementById('TTmp').innerHTML = Math.round((data.Tmp / 340.00 + 32.53));";
  page += "  }";


  page += "    updateStart(); </script> <script type='text/javascript'> function";
  page += "    einblenden(){ var select =";
  page += "    document.getElementById('rfid-mode').selectedIndex; if(select == 3)";
  page += "    document.getElementById('rfidfile').style.display = 'block'; else";
  page += "    document.getElementById('rfidfile').style.display = 'none'; } </script>";
  page += "<noscript>  <center style='color:red;'>In diesem Browser wurde JavaScript deaktiviert. <br> Das Interface wird möglicherweise nicht korrekt funktionieren.<hr size='1'> <br> <br> <br> </center> </noscript>";
  page += "</head>";


  page += "  <style>";
  page += " .accordion { background-color: #eee; color: #444; cursor: pointer;  padding: 18px;  width: 100%;  border: 1px solid;  border-radius: 8px;  border-color: #000;  text-align: left;  outline: none; font-size: 15px;  transition: 0.4s; }";
  page += " .active, .accordion:hover {  background-color: #ccc; }";
  page += " .accordion:after {  content: '+';  color: #777;  font-weight: bold;  float: right;  margin-left: 5px;}";
  page += " .active:after {  content: '-';}";
  page += " .panel {  padding: 0 18px;  width: 96%;  margin-left: 8px;  margin-right: 8px;  background-color: white;  max-height: 0;  overflow: hidden;  transition: max-height 0.2s ease-out;}";
  page += " .btn {  border: 2px solid black;  border-radius: 5px;  background-color: white;  color: black;  padding: 10px 22px;  font-size: 14px;  cursor: pointer;}";
  page += " .success {  border-color: #4CAF50;  color: green;}";
  page += " .success:hover {  background-color: #4CAF50;  color: white;}";
  page += " .slidecontainer { width: 80%; }";
  page += " .slider { -webkit-appearance: none;    width: 100%;    height: 15px;    border-radius: 5px;    background: #d3d3d3;    outline: none;    opacity: 0.7;    -webkit-transition: .2s;    transition: opacity .2s; }";
  page += " .slider:hover {    opacity: 1;}";
  page += " .slider::-webkit-slider-thumb {    -webkit-appearance: none;    appearance: none;    width: 25px;    height: 25px;  border-radius: 50%;    background: #4CAF50;    cursor: pointer;}";
  page += " .slider::-moz-range-thumb {    width: 25px;    height: 25px;    border-radius: 50%;    background: #4CAF50;    cursor: pointer; }";
  page += " .btn-group .button {  background-color: #4CAF50;  border: 1px solid green;  color: white;  padding: 5px 10px;  text-align: center;  text-decoration: none;  display: inline-block;  font-size: 14px;  cursor: pointer;  border-radius: 2px;  }";
  page += " .btn-group .button:not(:last-child) {  border-right: none;  }";
  page += " .btn-group .button:hover {  background-color: #3e8e41; }";
  page += " </style>";
  page += " <body style='font-size: 12px; font-family: Verdana, sans-serif'>  <h1>";
  page += ProjectName;
  page += " Interface</h1>          <h3>";
  page += ProjectName;
  page += " konfigurieren und steuern</h3><br> <br>";
  page += " <button class='accordion'>Steuerung</button><div style='padding: 0 18px; background-color: white;'><p>";
  page += " <div class='btn-group'>";
  page += "             <a href='play' class='button'><i class='material-icons'>play_arrow</i></a>";
  page += "             <a href='pause' class='button'><i class='material-icons'>pause</i></a>";
  page += "             <a href='prev' class='button'><i class='material-icons'>skip_previous</i></a>";
  page += "             <a href='next' class='button'><i class='material-icons'>skip_next</i></a>";
  page += "             <a href='vol-' class='button'><i class='material-icons'>volume_down</i></a>";
  page += "             <a href='vol+' class='button'><i class='material-icons'>volume_up</i></a>";
  page += "           </div> <p><br></p>";
  page += "           <form action='/'>";
  page += "            <div class='slidecontainer'>";

  page += "            <p>Maximale Lautstärke einstellen. Aktuell:";
  page += "            <span id='VolumeM'>??</span>";
  page += "            <input type='range' min='0' max='30' value='";
  page +=              max_Volume;
  page += "' name='max_volume' class='slider' id='myMaxVolRange' oninput='y.value=parseInt(max_volume.value)'>Neue max. Lautstärke: <output name='y' for='max_volume'></output></p>";

  page += "             <p><br>Lautstärke einstellen. Aktuell:";
  page += "             <span id='VolumeN'>??</span>";
  page += "             <input type='range' min='0' max='30' value='";

//  page +=              akt_Volume;
  page += "' name='akt_volume' class='slider' id='myAktVolRange' oninput='x.value=parseInt(akt_volume.value)'>Neue Lautstärke: <output name='x' for='akt_volume'></output></p>";
  page += "            <input type='submit' value='Speichern' class='btn success'>";
  page += "           </div>";
  page += "         <script>";
  page += "           var slider = document.getElementById('myAktVolRange');";
  page += "           var output = document.getElementById('aktValue');";
  page += "           var slider2 = document.getElementById('myMaxVolRange');";
  page += "           var output2= document.getElementById('maxValue');";
  page += "           output.innerHTML = slider.value;";
  page += "           output2.innerHTML = slider2.value;";
  page += "           slider2.oninput = function() {";
  page += "             output2.innerHTML = this.value;";
  page += "           }";
  page += "           slider.oninput = function() {";
  page += "             output.innerHTML = this.value;";
  page += "           }";
  page += "         </script>";
  page += "         </form>";
  page += "    </p></div><br>";
//  page += "<button class='accordion'>Equalizer</button><div class='panel'><p>";
//  page += "          <div class='btn-group'>";
//  page += "            <a href='eq_norm' class='button' Style='font-size: 10px;'>Normal</a>";
//  page += "            <a href='eq_pop' class='button' Style='font-size: 10px;'>Pop</a>";
//  page += "            <a href='eq_rock' class='button' Style='font-size: 10px;'>Rock</a>";
//  page += "            <a class='button' href='eq_jazz' Style='font-size: 10px;'>Jazz</a>";
//  page += "            <a class='button' href='eq_classic' Style='font-size: 10px;'>Classic</a>";
//  page += "            <a class='button' href='eq_base' Style='font-size: 10px;'>Base</a>";
//  page += "        </div>";
//  page += "   </p></div><br>";

//  page += "<button class='accordion'>Zeitsteuerung</button><div class='panel'><p>";
//  page += "        <form></form>";
//  page += "          <table style='margin-left: auto; margin-right: auto;' border='0' width='100%' cellspacing='0' cellpadding='0'>";
//  page += "            <tbody>";
//  page += "              <tr>";
//  page += "                <td>";
//  page += "          <form action='/'><br><u>Ausschaltzeit festlegen:</u><br>";
//  page += "          <input id='appt-time-off' type='time' name='appt-time-off'value='";
//  page +=             TimerOFF;
//  page += "'> Uhr &emsp; &emsp; &emsp; &emsp; &emsp; &emsp;";
//  page += "           <input type='submit' value='Speichern' class='btn success'><br>";
//  page += "           <input type='checkbox' name='cb_tmr_off' value='1' id='cb_tmr_off'>";
//  page += "           Jeden Tag zu dieser Uhrzeit ausschalten<BR>";
//  page += "           <input type='checkbox' name='cb_SleepLight_on' value='1' id='cb_SleepLight_on'>";
//  page += "           Nachtlicht einschalten<BR>";
//  page += "           <input type='checkbox' name='cb_SleepLight_off' value='1' id='cb_SleepLight_off'>";
//  page += "           Nachtlicht ausschalten <BR><br>";
//  page += "           </form>";
//  page += "          <br>";
//  page += "        </td>";
//  page += "      </Tr>";
//  page += "      <tr>";
//  page += "      <td style='border-top:1px dotted'>";
//  page += "         <form action='/'><br><br><u>Weckzeit festlegen:</u><br>";
//  page += "           <input id='appt-time-on' type='time' name='appt-time-on' value='";
//  page +=               TimerON;
//  page += "'> Uhr &emsp; &emsp; &emsp; &emsp; &emsp; &emsp;";
//  page += "           <input type='submit' value='Speichern' class='btn success'>";
//  page += "           <br>";
//  page += "           <input type='checkbox' name='cb_tmr_on' value='1' id='cb_tmr_on'>";
//  page += "           Jeden Tag um diese Uhrzeit Wecken <BR>";
//  page += "           <input type='checkbox' name='cb_WakeUpLight_on' value='1' id='cb_WakeUpLight_on'>";
//  page += "           WakeUp Light einschalten <BR>";
//  page += "           <input type='checkbox' name='cb_WakeUpLight_off' value='1' id='cb_WakeUpLight_on'>";
//  page += "           WakeUp Light ausschalten<BR>";
//  page += "         </form>";
//  page += "       </td></TR>";
//  page += "                 </tbody></table>";
//  page += "       </p></div><br>";

  page += "<button class='accordion'>RFID Verwaltung</button><div class='panel'><p>";
  page += "          <p><br><u>Zuletzt eingelesene Karte:</u></p>";
  page += "          <center><table style='margin-left: auto; margin-right: auto;' border='0' width='100%' cellspacing='0' cellpadding='0'>";
  page += "            <tbody>";
  page += "              <tr>";
  page += "                <td>ID</td>";
  page += "                 <td>Ordner</td>";
  page += "                 <td>Wiedergabemodus</td>";
  page += "               </tr>";
  page += "               <tr>";
  page += "                 <td>";
  page += "                      <span id='CCookie'>";
  page +=                         myCard.cookie;
  page += "                      </span> /";
  page += "                      <span id='CSerial'>";
  page +=                         mfrc522.PICC_ReadCardSerial();
  page += "                      </span>";
  page += "                 </td>";
  page += "                 <td>";
  page += "                      <span id='CFolder'>";
  page +=                         myCard.folder;
  page += "                      </span>";
  page += "                 </td>";
  page += "                 <td>";
  page += "                      <span id='CMode'>";
  page +=                          myCard.mode;
  page += "                        </span>";
  switch (myCard.mode) {
    case 1: page += " --> Hörspielmodus: eine zufällige Datei aus dem Ordner"; break;
    case 2: page += " --> Album Modus: kompletten Ordner spielen"; break;
    case 3: page += " --> Party Modus: Ordner in zufälliger Reihenfolgen"; break;
    case 4: page += " --> Einzel Modus: eine Datei aus dem Ordner abspielen"; break;
    default: break;
  };
  // case 5: page += " --> Hörbuch Modus: kompletten Ordner spielen und Fortschritt merken"; break;
  page += "                </td>";
  page += "              </tr>";
  page += "     <tr>";
  page += "       <td colspan='3' style='border-top:1px dotted;'>";
  page += "             <p><br><u>Karte neu Konfiguieren:</u></p>";
  page += "       </td>";
  page += "       </tr>";
  page += "       <tr>";
  page += "       <td>";
  page += "           <form action='/'>";
  page += "             Ordner Nr.: </td>";
  page += "              <td colspan='2'>";
  page += "       <input type='number' size='1' min='1' max='99' name='rfid-dir'></td>";
  page += "     </tr>";
  page += "     <tr>";
  page += "       <td valign=top>Wiedergabemodus:</td>";
  page += "         <td colspan='2' valign=top>";
  page += "           <select id='rfid-mode' name='rfid-mode' onchange='einblenden()'>";
// page += "             <option value='1'>1 - Zufallstitel: eine zufällige Datei aus dem Ordner</option>";
//  page += "             <option value='2'>2 - Album sortiert: Ordner kompletten spielen</option>";
//  page += "             <option value='3'>3 - Party Modus: Ordner kompletten in zufälliger Reihenfolge spielen</option>";
  page += "             <option value='4'>Titel Modus: eine Datei aus dem Ordner abspielen</option>";
  //page += "             <option value='5'>5 - Hörbuch Modus: kompletten Ordner spielen und Fortschritt merken</option>";
  page += "           </select>";
  page += "           <div id='rfidfile'>Dateinummer: &nbsp; &nbsp; <input type='number' size='1' min='1' max='999' name='rfid-file' value='1'></div>";
  page += "     </td>";
  page += "     </tr>";

  page += "           <tr>";
  page += "           <td>Farbe:</td>";
  page += "           <td colspan='2'>";
  page += "                   <select id='rfid-color' name='rfid-color' onchange='einblenden()'>";
  page += "                 <option value='1'>Schwarz/Aus</option>";
  page += "                 <option value='2'>Orange</option>";
  page += "                 <option value='3'>Gelb</option>";
  page += "                 <option value='4'>Grün</option>";
  page += "                 <option value='5'>Hellblau</option>";
  page += "                 <option value='6'>Weiss</option>";
  page += "                 <option value='7'>Pink</option>";
  page += "               </select>";
  page += "           </TD>";
  page += "     </tr>";
  page += "     <TR><td colspan='3'><br>";
  page += "       <input type='submit' value='Speichern' class='btn success'> <br> <br> <br>";
  page += "     </form></td></TR>";
  page += "              </tbody>";
  page += "          </table></center></p></div> <br>";

  page += "<button class='accordion'>LED-Farbsteuerung</button>";
  page += "<div class='panel'><p>";
  page += "       <form>";
  page += "         <br> <input type='color' id='color' style='width: 100px; height: 40px; cursor: zoom-in;border: 1px; border-radius: 3px;' name='LED_color' value='#00ab00'> <p><br>Helligkeit einstellen.";
  page += "         <p><input type='range' min='0' max='255' value='64' name='LED_bri' class='slider' id='LED_bri' oninput='w.value=parseInt(LED_bri.value)'>Neue Helligkeit: <output name='w' for='LED_bri'>64</output></p>";
  page += "         <br><input type='submit' value='Speichern' class='btn success'>";
  page += "       </form>";
  page += "    </p></div> <br> <br>";

  page += "<button class='accordion'>Systeminfos</button>";
  page += "<div class='panel'><p><br>";
  page += ProjectName;
  page += " ist seit <span id='time-ms'>??</span> Sekunden aktiv.<br>";
  page += " Die interne Betriebstemperatur beträgt: <span id='TTmp'>??</span> &deg;C.<br>";
  page += " WiFi-Einstellungen: <a href='/setup'>konfigurieren</a>";
  page += " <form action='/restart'> <input type='submit' value='System-Reboot'> </form>";
  page += "      </p></div><br>";


  page += "<script>";
  page += "var acc = document.getElementsByClassName('accordion');";
  page += "var i;";
  page += "for (i = 0; i < acc.length; i++) {";
  page += "  acc[i].addEventListener('click', function() {";
  page += "    this.classList.toggle('active');";
  page += "    var panel = this.nextElementSibling;";
  page += "    if (panel.style.maxHeight) {";
  page += "      panel.style.maxHeight = null;";
  page += "    } else {";
  page += "      panel.style.maxHeight = panel.scrollHeight + 'px';";
  page += "    }";
  page += "   });";
  page += "}";
  page += "</script>";
  page += "</body>";
  page += "</html>";
  return page;
}


String SetupPage() {
  String page = "<!DOCTYPE html>";
  page += "<html>";
  page += " <head>";
  page += "<title>";
  page += ProjectName;
  page += " WiFi-Einstellungen";
  page += "</title>";
  page += "<meta charset=UTF-8>";
  page += "<meta name=theme-color content=#4CAF50>";
  page += "<meta name=viewport content=width=device-width, initial-scale=1.0, user-scalable=yes>";
  page += "<meta name='author' content='Marko Langer'>";
  page += "<style>";
  page += ".btn {  border: 2px solid black;  border-radius: 5px;  background-color: white;  color: black;  padding: 10px 22px;  font-size: 14px;  cursor: pointer;}";
  page += ".success:hover {  background-color: #4CAF50;  color: white;}";
  page += ".success {  border-color: #4CAF50;  color: green;}";
  page += "</style>";
  page += "</head>";
  page += "<body style='font-size: 12px; font-family: Verdana, sans-serif'><h1>";
  page += ProjectName;
  page += "</h1>  <hr/><h2>  WiFi Setup</h2>";
  page += " <table border='0'><tr>";
  page += "    <td style='text-align:right;'>Netzwerkname: <form action='/'></td>";
  page += "    <td><input type='text' name='ssid'></td>";
  page += "    </tr><tr>";
  page += "     <td style='text-align:right;'>Passwort: </td>";
  page += "     <td><input type='text' name='pw'></td>";
  page += "   </tr><tr>    <td> &nbsp;</td>     <td> &nbsp;</td>";
  page += "   </tr><tr>";
  page += "     <td><input type='submit' value='Speichern' class='btn success'></form></td>";
  page += "     <td style='text-align:right;'><form action='/restart'> <input type='submit' value='Neustart' class='btn success'></form></td>";
  page += "   </tr></table>";
  page += "</html>";
  return page;
}
