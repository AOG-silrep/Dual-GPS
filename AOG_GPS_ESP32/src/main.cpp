

#include <stdio.h>
#include <string.h>

#include "jsonFunctions.hpp"
#include "main.hpp"

#include <ESPmDNS.h>
#include <WiFi.h>

#include <ESPUI.h>

#include <AsyncElegantOTA.h>

///////////////////////////////////////////////////////////////////////////
// global data
///////////////////////////////////////////////////////////////////////////

GPS_Config gpsConfig, gpsConfigDefaults;
Initialisation initialisation;
Diagnostics diagnostics;
void loadDiagnostics();
void saveDiagnostics();

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

const byte DNS_PORT = 53;
IPAddress wifiIP( 192, 168, 1, 1 ); //IP address for access point
IPAddress ipDestination( 192, 168, 5, 255 ); //IP address to send UDP data to
time_t lastHelloReceivedMillis;

///////////////////////////////////////////////////////////////////////////
// external Libraries
///////////////////////////////////////////////////////////////////////////

AsyncUDP udpRoof;
SoftwareSerial NmeaTransmitter;

///////////////////////////////////////////////////////////////////////////
// helper functions
///////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////
// Application
///////////////////////////////////////////////////////////////////////////
void setup( void ) {
  Serial.begin( 115200 );

  WiFi.disconnect( true );

  if( !LittleFS.begin( true ) ) {
    Serial.println( "LittleFS Mount Failed" );
    return;
  }

  loadSavedConfig();

  Serial1.begin( 115200, SERIAL_8N1, gpsConfig.gpioGpsRX1, gpsConfig.gpioGpsTX1 );
  delay( 10);
  Serial2.begin( 115200, SERIAL_8N1, gpsConfig.gpioGpsRX2, gpsConfig.gpioGpsTX2 );

  Serial.updateBaudRate( gpsConfig.baudrate );
  Serial.println( "Welcome to ESP32 Dual GPS.\nTo configure, please open the WebUI." );

  NmeaTransmitter.begin( gpsConfig.serialNmeaBaudrate, SWSERIAL_8N1, -1, gpsConfig.gpioSerialNmea );
  NmeaTransmitter.enableIntTx( false );
  nmeaMessageDelay = 1000 / gpsConfig.serialNmeaMessagesPerSec;

  pinMode( gpsConfig.gpioWifiLed, OUTPUT );
  pinMode( gpsConfig.gpioDcPowerGood, INPUT );

  initWiFi();

  Serial.println( "\n\nWiFi parameters:" );
  Serial.print( "Mode: " );
  if( WiFi.getMode() == WIFI_AP_STA ){
    Serial.println( "access point" );
    wifiIP = WiFi.softAPIP();
  } else {
    Serial.println( "client" );
    wifiIP = WiFi.localIP();
  }
  Serial.print( "IP address: " );
  Serial.println( wifiIP );

  initESPUI();

  if( gpsConfig.enableOTA ) {
    AsyncElegantOTA.begin( ESPUI.WebServer() );
  }

  if ( udpRoof.listen( gpsConfig.aogPortListenTo )){
    udpRoof.onPacket([](AsyncUDPPacket packet){
      uint8_t* data = packet.data();
      if ( data[1] + ( data[0] << 8 ) != 0x8081 ) {
        return;
      }
			uint8_t len = packet.length();
			uint16_t pgn = data[3] + ( data[2] << 8 );
      switch( pgn ){
        case 32712: {
          // PGN32712, Hello from AgIO to module
          IPAddress address = packet.remoteIP();
          if( ipDestination == address ){ // only send GPS to current AgOpenGPS
            lastHelloReceivedMillis = millis();
          } else if ( millis() - lastHelloReceivedMillis > 4000 ){ // AgOpenGPS Hello timed out
            ipDestination = address; // switch to new AgOpenGPS address
            lastHelloReceivedMillis = millis();
          }
        }
        break;

        default:
          break;
      }
    });
  }
  {
    Serial.print( "UDP writing to IP: " );
    Serial.println( ipDestination );
    Serial.print( "UDP writing to port: " );
    Serial.println( gpsConfig.aogPortSendTo );
    Serial.print( "UDP writing from port: " );
    Serial.println( gpsConfig.aogPortSendFrom );
  }

  if( WiFi.status() == WL_CONNECTED ) { // digitalWrite doesn't work in Wifi callbacks
    digitalWrite( gpsConfig.gpioWifiLed, gpsConfig.WifiLedOnLevel );
  }

  loadDiagnostics();
  
  initIdleStats();

  initHeadingAndPosition();
  initSerialUbxReceivers();
  initNmeaOut();
  initSpeedPWM();
  initDiagnosticDisplay();

  if( !MDNS.begin( "gps" )){
    Serial.println( "Error starting mDNS" );
  }
}

void loop( void ) {
  vTaskDelay( 100 );
}
