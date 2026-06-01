

#pragma once

#include <WiFi.h>
#include <WiFiMulti.h>

#include <HTTPClient.h>

#include <AsyncUDP.h>

#include <ESPUI.h>

#include <Wire.h>

#include <SoftwareSerial.h>

extern AsyncUDP udpRoof;
extern IPAddress ipDestination; //IP address to send UDP data to
extern time_t lastHelloReceivedMillis;

extern uint16_t labelLoad;
extern uint16_t labelHeading;
extern uint16_t labelAgOpenGpsAddress;
extern uint16_t labelGpsReceivers;
extern uint16_t labelGpsMessages;
extern uint16_t labelPwmOut;
extern uint16_t labelGpsMessageHz;

extern uint64_t RelPosNedCount;
extern time_t RelPosNedMillis;
extern uint64_t NavPvtCount;
extern time_t NavPvtMillis;
extern time_t previousRelPosNedMillis;
extern time_t previousNavPvtMillis;

extern uint32_t mphPwm;

//Kalman filter roll
extern double rollK, rollPc, rollG, rollXp, rollZp, rollXe;
extern double rollP;
extern double rollVar;
extern double rollVarProcess;
//Kalman filter heading
extern double headK, headPc, headG, headXp, headZp, headXe;
extern double headP;
extern double headVar;
extern double headVarProcess;
//Kalman filter heading
extern double headVTGK, headVTGPc, headVTGG, headVTGXp, headVTGZp, headVTGXe;
extern double headVTGP;
extern double headVTGVar;
extern double headVTGVarProcess;
//Kalman filter heading
extern double headMixK, headMixPc, headMixG, headMixXp, headMixZp, headMixXe;
extern double headMixP;
extern double headMixVar;
extern double headMixVarProcess;
//Kalman filter lat
extern double latK, latPc, latG, latXp, latZp, latXe;
extern double latP;
extern double latVar;
extern double latVarProcess;
//Kalman filter lon
extern double lonK, lonPc, lonG, lonXp, lonZp, lonXe;
extern double lonP;
extern double lonVar;
extern double lonVarProcess;

extern double VarProcessVeryFast;
extern double VarProcessFast;
extern double VarProcessMedi;
extern double VarProcessSlow;
extern double VarProcessVerySlow;
extern bool filterGPSpos;

extern uint16_t nmeaMessageDelay;

extern double HeadingQuotaVTG;

extern double virtLat, virtLon;

extern bool bNMEAstarted, bGGAexists;
extern String sNMEA;
extern int16_t i, iPos;
extern int8_t cFixQualGGA;

//heading + roll
extern double HeadingRelPosNED, cosHeadRelPosNED, HeadingVTG, HeadingVTGOld, cosHeadVTG, HeadingMix, cosHeadMix;
extern double HeadingDiff, HeadingMax, HeadingMin, HeadingMixBak, HeadingQualFactor;
extern uint8_t noRollCount,  drivDirect;
constexpr double PI180 = PI / 180;
extern bool dualGPSHeadingPresent, rollPresent, virtAntPosPresent, add360ToRelPosNED, add360ToVTG;
extern double roll, rollToAOG;
extern uint8_t dualAntNoValueCount, dualAntNoValueMax;
extern int16_t antDistDeviation;

extern bool powerUnstable;
extern time_t powerUnstableMillis;
extern time_t gpsHzMaxMillis;
extern time_t gpsHzMinMillis;
extern time_t gpsHzCurrentMillis;

struct NAV_PVT {
    uint8_t cls;
    uint8_t id;
    uint16_t len;
    uint32_t iTOW;  //GPS time ms
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    uint32_t tAcc;
    int32_t nano;
    uint8_t fixType;//0 no fix....
    uint8_t flags;
    uint8_t flags2;
    uint8_t numSV; //number of sats
    int32_t lon;   //deg * 10^-7
    int32_t lat;   //deg * 10^-7
    int32_t height;
    int32_t hMSL;  //heigth above mean sea level mm
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    int32_t gSpeed; //Ground Speed mm/s
    int32_t headMot;
    uint32_t sAcc;
    uint32_t headAcc;
    uint16_t pDOP;
    uint8_t flags3;
    uint8_t reserved1;
    int32_t headVeh;
    int32_t magDec;//doesnt fit, checksum was 4 bytes later, so changes from int16_t to int32_t
    uint32_t magAcc;
    uint8_t CK0;
    uint8_t CK1;
};
constexpr uint8_t sizeOfUBXArray = 3;
extern NAV_PVT UBXPVT1[sizeOfUBXArray];

//NMEA
extern uint8_t OGIBuffer[90], HDTBuffer[24], VTGBuffer[55], GGABuffer[80], RMCBuffer[80];
extern bool newOGI, newHDT, newGGA, newVTG;
extern uint8_t OGIdigit, GGAdigit, VTGdigit, HDTdigit, RMCdigit;
extern uint8_t GGABufferLength, VTGBufferLength, HDTBufferLength, RMCBufferLength;

extern SoftwareSerial NmeaTransmitter;
extern QueueHandle_t HDTQueue;
extern QueueHandle_t VTGQueue;
extern QueueHandle_t GGAQueue;
extern QueueHandle_t RMCQueue;

//UBX
extern uint8_t UBXRingCount1, UBXRingCount2, UBXDigit1, UBXDigit2, OGIfromUBX;
extern int16_t UBXLength1, UBXLength2;
constexpr uint8_t UBX_HEADER[] = { 0xB5, 0x62 };//all UBX start with this
extern bool isUBXPVT1,  isUBXRelPosNED, existsUBXRelPosNED;

struct NAV_RELPOSNED {
    uint8_t cls;
    uint8_t id;
    uint16_t len;
    uint8_t ver;
    uint8_t res1;
    uint16_t refStID;
    uint32_t iTOW;
    int32_t relPosN;
    int32_t relPosE;
    int32_t relPosD;
    int32_t relPosLength;
    int32_t relPosHeading;
    int32_t res2;//    uint8_t res2;
    int8_t relPosHPN;
    int8_t relPosHPE;
    int8_t relPosHPD;
    int8_t relPosHPLength;
    uint32_t accN;
    uint32_t accE;
    uint32_t accD;
    uint32_t accLength;
    uint32_t accHeading;
    int32_t res3; // uint8_t res3;
    uint32_t flags;
    uint8_t CK0;
    uint8_t CK1;
};
extern NAV_RELPOSNED UBXRelPosNED[sizeOfUBXArray];

///////////////////////////////////////////////////////////////////////////
// Diagnostics
///////////////////////////////////////////////////////////////////////////

struct Diagnostics{
    uint16_t badChecksumNavPVTCount;
    uint16_t wrongLengthNavPVTCount;
    uint16_t badChecksumRelPosNEDCount;
    uint16_t wrongLengthRelPosNEDCount;
};
extern Diagnostics diagnostics;

extern void loadDiagnostics();
extern void saveDiagnostics();

///////////////////////////////////////////////////////////////////////////
// Configuration
///////////////////////////////////////////////////////////////////////////

struct GPS_Config {

  enum class SpeedUnits : int8_t {
    MilesPerHour      = 0,
    KilometersPerHour = 1
  } speedUnits = SpeedUnits::MilesPerHour;

  char ssid[24] = "AOG hub";
  char password[24] = "password";
  char hostname[24] = "Dual GPS";

  uint8_t gpioWifiLed = 32;
  uint8_t WifiLedOnLevel = HIGH;    //HIGH = LED on high, LOW = LED on low

  uint8_t gpioDcPowerGood = 5;

  uint32_t baudrate = 115200;

  //connection plan:
  // ESP32--- Right F9P GPS pos --- Left F9P Heading-----Sentences
  //  RX1-27-------TX1--------------------------------UBX-Nav-PVT out   (=position+speed)
  //  TX1-16-------RX1--------------------------------RTCM in           (NTRIP comming from AOG to get absolute/correct postion
  //  RX2-25-----------------------------TX1----------UBX-RelPosNED out (=position relative to other Antenna)
  //  TX2-17-----------------------------RX1----------
  //               TX2-------------------RX2----------RTCM 1077+1087+1097+1127+1230+4072.0+4072.1 (activate in right F9P = NTRIP for relative positioning)

  // IO pins ----------------------------------------------------------------------------------------
  uint8_t gpioGpsRX1 = 27;              //right F9P TX1 GPS pos
  uint8_t gpioGpsTX1 = 16;              //right F9P RX1 GPS pos

  uint8_t gpioGpsRX2 = 25;              //left F9P TX1 Heading
  uint8_t gpioGpsTX2 = 17;              //left F9P RX1 Heading

  uint8_t gpioSerialNmea = 19;
  bool sendSerialNmeaVTG = 0;                     //1: send Nmea message 0: off
  bool sendSerialNmeaGGA = 1;                     //1: send Nmea message 0: off
  bool sendSerialNmeaHDT = 0;                     //1: send Nmea message 0: off
  bool sendSerialNmeaRMC = 0;                     //1: send Nmea message 0: off
  uint16_t serialNmeaBaudrate = 38400;
  uint8_t serialNmeaMessagesPerSec = 5;

  uint8_t gpioVelocityPWM = 15;      // Velocity (MPH speed) PWM pin
  int velocityHzPerMPH = 50; // PWM (MPH * multiplier)

  //Antennas position
  double AntDist = 140.0;                //cm distance between Antennas
  double AntHeight = 280.0;              //cm height of Antenna
  double virtAntRight = 70.0;           //cm to move virtual Antenna to the right
  double virtAntForew = 0.0;            //cm to move virtual Antenna foreward
  double headingAngleCorrection = 90;
  double AntDistDeviationFactor = 1.2;  // factor (>1), of whom length vector from both GPS units can max differ from AntDist before stop heading calc
  uint8_t MaxHeadChangPerSec = 30;         // degrees that heading is allowed to change per second

  uint8_t checkUBXFlags = 1;               //UBX sending quality flags, when used with RTK sometimes 
  uint8_t filterGPSposOnWeakSignal = 1;    //filter GPS Position on weak GPS signal
  uint8_t GPSPosCorrByRoll = 1;            // 0 = off, 1 = correction of position by roll (AntHeight must be > 0)
  double rollAngleCorrection = 0.0; 

  bool sendOGI = 1;                     //1: send NMEA message 0: off
  bool sendVTG = 0;                     //1: send NMEA message 0: off
  bool sendGGA = 0;                     //1: send NMEA message 0: off
  bool sendHDT = 0;                     //1: send NMEA message 0: off

  bool debugmode = false;
  bool debugmodeUBX = false;
  bool debugmodeHeading = false;
  bool debugmodeVirtAnt = false;
  bool debugmodeFilterPos = false;
  bool debugmodeRAW = false;

  bool enableOTA = false;

  uint16_t aogPortSendFrom = 5544;
  uint16_t aogPortListenTo = 8888;
  uint16_t aogPortSendTo = 9999;

  bool retainWifiSettings = true;
};
extern GPS_Config gpsConfig, gpsConfigDefaults;

struct Initialisation {
  uint16_t portSendFrom = 5544;
  uint16_t portListenTo = 8888;
  uint16_t portSendTo = 9999;

};
extern Initialisation initialisation;


///////////////////////////////////////////////////////////////////////////
// Global Data
///////////////////////////////////////////////////////////////////////////

extern ESPUIClass ESPUI;

// extern AsyncUDP udpLocalPort;
// extern AsyncUDP udpRemotePort;
extern AsyncUDP udpSendFrom;

///////////////////////////////////////////////////////////////////////////
// Helper Classes
///////////////////////////////////////////////////////////////////////////
extern portMUX_TYPE mux;
class TCritSect {
    TCritSect() {
      portENTER_CRITICAL( &mux );
    }
    ~TCritSect() {
      portEXIT_CRITICAL( &mux );
    }
};

///////////////////////////////////////////////////////////////////////////
// Threads
///////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////
// Helper Functions
///////////////////////////////////////////////////////////////////////////
extern void setResetButtonToRed();

extern void initESPUI();
extern void initIdleStats();
extern void initWiFi();
extern void initHeadingAndPosition();
extern void initSerialUbxReceivers();
extern void initNmeaOut();
extern void initSpeedPWM();
extern void initDiagnosticDisplay();

extern void buildGGA();
extern void buildVTG();
extern void buildHDT();
extern void buildOGI();
extern void buildRMC();
extern void getUBX();
extern void NmeaOut();
