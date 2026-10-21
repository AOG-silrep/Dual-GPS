

#include <memory>

#include <FS.h>
#include <LittleFS.h>


#include "main.hpp"
#include "jsonFunctions.hpp"

void loadSavedConfig() {
  {
    if( LittleFS.exists( "/gps.json" ) ) {
      auto j = loadJsonFromFile( "/gps.json" );
      parseJsonToGpsConfig( j, gpsConfig );
    }
    else if( LittleFS.exists( "/config.json" ) ){ // import from old config name
      auto j = loadJsonFromFile( "/config.json" );
      parseJsonToGpsConfig( j, gpsConfig );
      saveJsonToFile( j, "/gps.json" ); // auto save file for next boot
      LittleFS.remove( "/config.json" ); // remove config.json ->
      //deprecated since it was a universal file with a risk of sharing wrong config to other PCBs
    }
    else{
      json j;
      parseJsonToGpsConfig( j, gpsConfig );
    }
  }
}

void saveConfig() {
  {
    const auto j = parseGpsConfigToJson( gpsConfig );
    saveJsonToFile( j, "/gps.json" );
  }
}

json loadJsonFromFile( const char* fileName ) {
  json j;

  if( LittleFS.exists( fileName ) ) {
    File file = LittleFS.open( fileName, "r" );

    if( file ) {
      std::vector<uint8_t> data;
      data.resize( file.size() );

      file.read( data.data(), file.size() );

      try {
        j = json::parse( data/*, nullptr, false*/ );
      } catch( json::exception& e ) {
        // output exception information
        Serial.print( "message: " );
        Serial.println( e.what() );
        Serial.print( "exception id: " );
        Serial.println( e.id );
      }
    } else {
      Serial.print( "Could not open file for reading: " );
      Serial.println( fileName );
      Serial.flush();
    }

    file.close();
  }

  return j;
}

void saveJsonToFile( const json& json, const char* fileName ) {
  // pretty print with 2 spaces indentation
  auto data = json.dump( 2 );

  File file = LittleFS.open( fileName, "w" );

  if( file && !file.isDirectory() ) {
    file.write( ( uint8_t* )data.c_str(), data.size() );
  } else {
    Serial.print( "Could not open file for writing: " );
    Serial.println( fileName );
    Serial.flush();
  }

  file.close();
}

json parseGpsConfigToJson( const GPS_Config& config ) {
  json j;

  j["wifi"]["ssid"] = config.ssid;
  j["wifi"]["password"] = config.password;
  j["wifi"]["hostname"] = config.hostname;
  j["wifi"]["retainSettings"] = config.retainWifiSettings;
  j["wifi"]["WifiLedOnLevel"] = config.WifiLedOnLevel;

  j["gps"]["AntennaDist"] = int( config.AntDist );
  j["gps"]["AntennaHeight"] = config.AntHeight;
  j["gps"]["virtualAntennaRight"] = config.virtAntRight;
  j["gps"]["virtualAntennaForeword"] = config.virtAntForew;
  j["gps"]["headingAngleCorrection"] = config.headingAngleCorrection;
  j["gps"]["AntennaDistanceDeviationFactor"] = config.AntDistDeviationFactor;
  j["gps"]["MaxHeadingChangePerSecond"] = config.MaxHeadChangPerSec;
  j["gps"]["checkUBXFlags"] = config.checkUBXFlags;
  j["gps"]["filterGPSpositionOnWeakSignal"] = config.filterGPSposOnWeakSignal;
  j["gps"]["GPSPositionCorrectionByRoll"] = config.GPSPosCorrByRoll;
  j["gps"]["rollAngleCorrection"] = config.rollAngleCorrection;
  
  j["serialNmea"]["VTGHz"] = config.serialNmeaVTGHz;
  j["serialNmea"]["GGAHz"] = config.serialNmeaGGAHz;
  j["serialNmea"]["HDTHz"] = config.serialNmeaHDTHz;
  j["serialNmea"]["RMCHz"] = config.serialNmeaRMCHz;
  j["serialNmea"]["serialNmeaBaudrate"] = config.serialNmeaBaudrate;

  j["messages"]["sendOGI"] = config.sendOGI;
  j["messages"]["sendVTG"] = config.sendVTG;
  j["messages"]["sendGGA"] = config.sendGGA;
  j["messages"]["sendHDT"] = config.sendHDT;

  j["PWM"]["velocityHzPerMPH"] = config.velocityHzPerMPH;

  j["debug"]["debugmode"] = config.debugmode;
  j["debug"]["debugmodeUBX"] = config.debugmodeUBX;
  j["debug"]["debugmodeHeading"] = config.debugmodeHeading;
  j["debug"]["debugmodeVirtualAntenna"] = config.debugmodeVirtAnt;
  j["debug"]["debugmodeFilterPosition"] = config.debugmodeFilterPos;
  j["debug"]["debugmodeRAW"] = config.debugmodeRAW;

  j["connection"]["baudrate"] = config.baudrate;
  j["connection"]["enableOTA"] = config.enableOTA;

  j["connection"]["aog"]["sendFrom"] = config.aogPortSendFrom;
  j["connection"]["aog"]["listenTo"] = config.aogPortListenTo;
  j["connection"]["aog"]["sendTo"] = config.aogPortSendTo;

  return j;
}

//serial NMEA rate of one message, imported from the old on/off switch and shared messages per second
static uint8_t loadSerialNmeaHz( const json& j, const char* message, uint8_t defaultHz, int legacyHz ) {
  const json::json_pointer hzKey( std::string( "/serialNmea/" ) + message + "Hz" );
  const json::json_pointer sendKey( std::string( "/serialNmea/send" ) + message );
  if( j.contains( hzKey ) ) {
    return snapSerialNmeaHz( j.value( hzKey, int( defaultHz ) ) );
  }
  if( j.contains( sendKey ) ) {
    return j.value( sendKey, 0 ) != 0 ? snapSerialNmeaHz( legacyHz ) : 0;
  }
  return defaultHz;
}

void parseJsonToGpsConfig( json& j, GPS_Config& config ) {
  if( j.is_object() ) {
    try {
      {
        std::string str = j.value( "/wifi/ssid"_json_pointer, gpsConfigDefaults.ssid );
        memset( config.ssid, 0, sizeof( config.ssid ) );
        memcpy( config.ssid, str.c_str(), str.size() );
      }
      {
        std::string str = j.value( "/wifi/password"_json_pointer, gpsConfigDefaults.password );
        memset( config.password, 0, sizeof( config.password ) );
        memcpy( config.password, str.c_str(), str.size() );
      }
      {
        std::string str = j.value( "/wifi/hostname"_json_pointer, gpsConfigDefaults.hostname );
        memset( config.hostname, 0, sizeof( config.hostname ) );
        memcpy( config.hostname, str.c_str(), str.size() );
      }
      config.retainWifiSettings = j.value( "/wifi/retainSettings"_json_pointer, gpsConfigDefaults.retainWifiSettings );
      config.WifiLedOnLevel = j.value( "/wifi/WifiLedOnLevel"_json_pointer, gpsConfigDefaults.WifiLedOnLevel );

      config.AntDist = j.value( "/gps/AntennaDist"_json_pointer, gpsConfigDefaults.AntDist );
      config.AntHeight = j.value( "/gps/AntennaHeight"_json_pointer, gpsConfigDefaults.AntHeight );
      config.virtAntRight = j.value( "/gps/virtualAntennaRight"_json_pointer, gpsConfigDefaults.virtAntRight );
      config.virtAntForew = j.value( "/gps/virtualAntennaForeword"_json_pointer, gpsConfigDefaults.virtAntForew );
      config.headingAngleCorrection = j.value( "/gps/headingAngleCorrection"_json_pointer, gpsConfigDefaults.headingAngleCorrection );
      config.AntDistDeviationFactor = j.value( "/gps/AntennaDistanceDeviationFactor"_json_pointer, gpsConfigDefaults.AntDistDeviationFactor );
      config.MaxHeadChangPerSec = j.value( "/gps/MaxHeadingChangePerSecond"_json_pointer, gpsConfigDefaults.MaxHeadChangPerSec );
      config.checkUBXFlags = j.value( "/gps/checkUBXFlags"_json_pointer, gpsConfigDefaults.checkUBXFlags );
      config.filterGPSposOnWeakSignal = j.value( "/gps/filterGPSpositionOnWeakSignal"_json_pointer, gpsConfigDefaults.filterGPSposOnWeakSignal );
      config.GPSPosCorrByRoll = j.value( "/gps/GPSPositionCorrectionByRoll"_json_pointer, gpsConfigDefaults.GPSPosCorrByRoll );
      config.rollAngleCorrection = j.value( "/gps/rollAngleCorrection"_json_pointer, gpsConfigDefaults.rollAngleCorrection );

      {
        const int legacyHz = j.value( "/serialNmea/serialNmeaMessagesPerSecond"_json_pointer, 5 );
        config.serialNmeaVTGHz = loadSerialNmeaHz( j, "VTG", gpsConfigDefaults.serialNmeaVTGHz, legacyHz );
        config.serialNmeaGGAHz = loadSerialNmeaHz( j, "GGA", gpsConfigDefaults.serialNmeaGGAHz, legacyHz );
        config.serialNmeaHDTHz = loadSerialNmeaHz( j, "HDT", gpsConfigDefaults.serialNmeaHDTHz, legacyHz );
        config.serialNmeaRMCHz = loadSerialNmeaHz( j, "RMC", gpsConfigDefaults.serialNmeaRMCHz, legacyHz );
      }
      config.serialNmeaBaudrate = j.value( "/serialNmea/serialNmeaBaudrate"_json_pointer, gpsConfigDefaults.serialNmeaBaudrate );

      config.sendOGI = j.value( "/messages/sendOGI"_json_pointer, gpsConfigDefaults.sendOGI );
      config.sendVTG = j.value( "/messages/sendVTG"_json_pointer, gpsConfigDefaults.sendVTG );
      config.sendGGA = j.value( "/messages/sendGGA"_json_pointer, gpsConfigDefaults.sendGGA );
      config.sendHDT = j.value( "/messages/sendHDT"_json_pointer, gpsConfigDefaults.sendHDT );

      config.velocityHzPerMPH = j.value( "/PWM/velocityHzPerMPH"_json_pointer, gpsConfigDefaults.velocityHzPerMPH );

      config.debugmode = j.value( "/debug/debugmode"_json_pointer, gpsConfigDefaults.debugmode );
      config.debugmodeUBX = j.value( "/debug/debugmodeUBX"_json_pointer, gpsConfigDefaults.debugmodeUBX );
      config.debugmodeHeading = j.value( "/debug/debugmodeHeading"_json_pointer, gpsConfigDefaults.debugmodeHeading );
      config.debugmodeVirtAnt = j.value( "/debug/debugmodeVirtualAntenna"_json_pointer, gpsConfigDefaults.debugmodeVirtAnt );
      config.debugmodeFilterPos = j.value( "/debug/debugmodeFilterPosition"_json_pointer, gpsConfigDefaults.debugmodeFilterPos );
      config.debugmodeRAW = j.value( "/debug/debugmodeRAW"_json_pointer, gpsConfigDefaults.debugmodeRAW );

      config.baudrate = j.value( "/connection/baudrate"_json_pointer, gpsConfigDefaults.baudrate );
      config.enableOTA = j.value( "/connection/enableOTA"_json_pointer, gpsConfigDefaults.enableOTA );

      config.aogPortSendFrom = j.value( "/connection/aog/sendFrom"_json_pointer, gpsConfigDefaults.aogPortSendFrom );
      config.aogPortListenTo = j.value( "/connection/aog/listenTo"_json_pointer, gpsConfigDefaults.aogPortListenTo );
      config.aogPortSendTo = j.value( "/connection/aog/sendTo"_json_pointer, gpsConfigDefaults.aogPortSendTo );

    } catch( json::exception& e ) {
      // output exception information
      Serial.print( "message: " );
      Serial.println( e.what() );
      Serial.print( "exception id: " );
      Serial.println( e.id );
      Serial.flush();
    }
  }
}

void loadDiagnostics() {
  {
    auto j = loadJsonFromFile( "/diagnostics.json" );
    parseJsonToDiagnostics( j, diagnostics );
  }
}

void saveDiagnostics() {
  {
    const auto j = parseDiagnosticsToJson( diagnostics );
    saveJsonToFile( j, "/diagnostics.json" );
  }
}

json parseDiagnosticsToJson( const Diagnostics& diagnostics ) {
  json j;

  j["count"]["badChecksumNavPVT"] = diagnostics.badChecksumNavPVTCount;
  j["count"]["wrongLengthNavPVT"] = diagnostics.wrongLengthNavPVTCount;
  j["count"]["badChecksumRelPosNED"] = diagnostics.badChecksumRelPosNEDCount;
  j["count"]["wrongLengthRelPosNED"] = diagnostics.wrongLengthRelPosNEDCount;

  return j;
}

void parseJsonToDiagnostics( json& j, Diagnostics& diagnostics ) {
  if( j.is_object() ) {
    try {
   
      diagnostics.badChecksumNavPVTCount = j.value( "/count/badChecksumNavPVT"_json_pointer, 0 );
      diagnostics.wrongLengthNavPVTCount = j.value( "/count/wrongLengthNavPVT"_json_pointer, 0 );
      diagnostics.badChecksumRelPosNEDCount = j.value( "/count/badChecksumRelPosNED"_json_pointer, 0 );
      diagnostics.wrongLengthRelPosNEDCount = j.value( "/count/wrongLengthRelPosNED"_json_pointer, 0 );

    } catch( json::exception& e ) {
      // output exception information
      Serial.print( "message: " );
      Serial.println( e.what() );
      Serial.print( "exception id: " );
      Serial.println( e.id );
      Serial.flush();
    }
  }
}
