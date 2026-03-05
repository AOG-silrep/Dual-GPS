

#include "main.hpp"
#include "esp_wifi.h"
#include <WiFi.h>

IPAddress softApIP( 192, 168, 1, 1 );
String apName;
bool WiFiWasConnected = false;

void WiFiStationGotIP( WiFiEvent_t event, WiFiEventInfo_t info ){
  IPAddress myIP = WiFi.localIP();
  if( myIP == IPAddress( 0, 0, 0, 0 )) {
    Serial.print("\nCollecting valid IP address ");
    uint8_t timeout = 100;
    while( timeout && myIP == IPAddress( 0, 0, 0, 0 )){
      delay( 10 );
      myIP = WiFi.localIP();
      timeout--;
      Serial.print(".");
    }
    if( timeout > 0 ){
      Serial.println( ". done" );
    } else {
      Serial.println( "\nDHCP failed, GPS will not work, restarting..." );
      ESP.restart();
      delay( 100 );
    }
  }
  if( myIP[3] != 79 ){
      myIP[3] = 79;
      IPAddress gwIP = WiFi.gatewayIP();
      if ( !WiFi.config( myIP, gwIP, IPAddress( 255, 255, 255, 0 ), gwIP )) {
        Serial.println( "STA Failed to configure" );
      }
      Serial.println( "Switching off AP, station only" );
      WiFi.softAPdisconnect ( true );
      WiFi.mode( WIFI_MODE_STA );
  }
  delay( 10 );
  digitalWrite( gpsConfig.gpioWifiLed, gpsConfig.WifiLedOnLevel );
  WiFiWasConnected = true;
}

void WiFiStationDisconnected( WiFiEvent_t event, WiFiEventInfo_t info ){
  digitalWrite( gpsConfig.gpioWifiLed, !gpsConfig.WifiLedOnLevel );
  if( WiFiWasConnected == true ){
    WiFi.disconnect( true );
    if( !WiFi.config( INADDR_NONE, INADDR_NONE, INADDR_NONE ) ) {
      Serial.println( "STA Failed to unset configuration" );
    }
    delay( 25 );
    WiFi.begin( gpsConfig.ssid, gpsConfig.password );
    Serial.println( "reconnecting" );
  } else {
    WiFi.reconnect();
  }
}

void WiFiStationConnected( WiFiEvent_t event, WiFiEventInfo_t info ){
  WiFi.config( INADDR_NONE, INADDR_NONE, INADDR_NONE );
  WiFi.setHostname( gpsConfig.hostname );
  delay( 10 );
  digitalWrite( gpsConfig.gpioWifiLed, gpsConfig.WifiLedOnLevel );
}

void WiFiAPStaConnected( WiFiEvent_t event, WiFiEventInfo_t info ){
  Serial.println( "Switching off station mode, AP only" );
  WiFi.disconnect( true );
  delay( 100 );
  WiFi.mode( WIFI_MODE_AP );
}

void initWiFi( void ){
  delay( 50 );
  WiFi.config( INADDR_NONE, INADDR_NONE, INADDR_NONE );
  delay( 50 );
  WiFi.onEvent( WiFiStationConnected, ARDUINO_EVENT_WIFI_STA_CONNECTED );
  WiFi.onEvent( WiFiStationDisconnected, ARDUINO_EVENT_WIFI_STA_DISCONNECTED );
  WiFi.onEvent( WiFiStationGotIP, ARDUINO_EVENT_WIFI_STA_GOT_IP );
  WiFi.onEvent( WiFiAPStaConnected, ARDUINO_EVENT_WIFI_AP_STACONNECTED );
  // try to connect to existing network
  WiFi.begin( gpsConfig.ssid, gpsConfig.password );
  WiFi.setAutoReconnect ( false );
  Serial.print( "\n\nTry to connect to existing network \"" );
  Serial.print( gpsConfig.ssid );
  Serial.print( "\" with password \"" );
  Serial.print( gpsConfig.password );
  Serial.println( "\"" );

  uint8_t timeout = 10;
  // Wait for connection, 5.0s timeout
  do {
    delay( 500 );
    Serial.print( "." );
    digitalWrite( gpsConfig.gpioWifiLed, ! digitalRead( gpsConfig.gpioWifiLed ));
    timeout--;
  } while( timeout && WiFi.status() != WL_CONNECTED );
  // not connected -> create hotspot
  if( WiFi.status() != WL_CONNECTED ) {
      if( WiFi.disconnect( true )){
        Serial.println( "Wifi reset successful" );
      } else Serial.println( "Wifi reset failed" );

      digitalWrite( gpsConfig.gpioWifiLed, !gpsConfig.WifiLedOnLevel );

      WiFi.begin(); // WiFi needs to be running to retrieve the MAC address
      apName = String( "Dual GPS " );
      apName += WiFi.macAddress();
      apName.replace( ":", "" );

      Serial.print( "\n\nCreating hotspot \"" );
      Serial.print( apName.c_str() );
      Serial.println( "\"" );
      if( WiFi.mode( WIFI_MODE_APSTA )){
        delay( 25 );
        if( WiFi.softAPConfig( softApIP, softApIP, IPAddress( 255, 255, 255, 0 ))){
          delay( 25 );
          WiFi.softAP( apName.c_str() );
        } else Serial.println( "Wifi softAPConfig failed" );
      } else Serial.println( "Wifi APSTA mode failed" );

      WiFi.begin( gpsConfig.ssid, gpsConfig.password );
      while ( !WIFI_EVENT_AP_START ){ // wait until AP has started
          delay( 100 );
          Serial.print( "." );
      }
    }
}
