
#include "main.hpp"

//messages per second over the last display period, in tenths
static uint32_t hzTenths( uint64_t count, uint64_t& previousCount, time_t elapsedMillis ) {
  uint64_t delta = count >= previousCount ? count - previousCount : count;//"Reset all" zeroes the counts
  previousCount = count;
  return elapsedMillis > 0 ? delta * 10000 / elapsedMillis : 0;
}

static void addHz( String& str, uint32_t tenths ) {
  str += tenths / 10;
  str += '.';
  str += tenths % 10;
  str += " Hz";
}

//the label keeps white-space, so the table must not contain newlines
static void addRowName( String& str, const char* name ) {
  str += "<tr><td>";
  str += name;
  str += "</td><td align=right>";
}

template<typename T>
static void addRow( String& str, const char* name, T value ) {
  addRowName( str, name );
  str += value;
  str += "</td></tr>";
}

static void addRateRow( String& str, uint32_t tenths ) {
  addRowName( str, "Rate" );
  addHz( str, tenths );
  str += "</td></tr>";
}

//time between the last two messages, or how long none arrived; returns true when none arrived for over 1s
static bool addIntervalRow( String& str, time_t lastMillis, time_t previousMillis ) {
  addRowName( str, "Interval" );
  time_t elapsedMillis = millis() - lastMillis;
  bool stale = lastMillis == 0 or elapsedMillis > 1000;
  if( lastMillis == 0 ){
    str += "none received";
  } else if( stale ){
    str += "none for ";
    str += elapsedMillis / 1000;
    str += " s";
  } else {
    str += lastMillis - previousMillis;
    str += " ms";
  }
  str += "</td></tr>";
  return stale;
}

static void updateLabel( uint16_t label, const String& str, bool alarm ) {
  Control* labelHandle = ESPUI.getControl( label );
  labelHandle->color = alarm ? ControlColor::Alizarin : ControlColor::Turquoise;
  labelHandle->value = str;
  ESPUI.updateControl( labelHandle );
}

void diagnosticDisplay ( void* z ){
  constexpr TickType_t xFrequency = 1000;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  String str;
  str.reserve( 1000 );

  time_t previousDisplayMillis = millis();
  uint64_t previousNavPvtCount = NavPvtCount;
  uint64_t previousRelPosNedCount = RelPosNedCount;
  uint64_t previousOutputCount = OutputCount;

  for( ;; ) {
    //wait first, so the first rates are over a whole period
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    time_t displayMillis = millis();
    time_t elapsedDisplayMillis = displayMillis - previousDisplayMillis;
    previousDisplayMillis = displayMillis;
    uint32_t navPvtHzTenths = hzTenths( NavPvtCount, previousNavPvtCount, elapsedDisplayMillis );
    uint32_t relPosNedHzTenths = hzTenths( RelPosNedCount, previousRelPosNedCount, elapsedDisplayMillis );
    uint32_t outputHzTenths = hzTenths( OutputCount, previousOutputCount, elapsedDisplayMillis );

    str = "Heading from Dual GPS: ";
    str += HeadingRelPosNED;
    str += "\nHeading from VTG: ";
    str += HeadingVTG;
    str += "\nHeading from Mix: ";
    str += HeadingMix;

    Control* labelHeadingHandle = ESPUI.getControl( labelHeading );
    labelHeadingHandle->value = str;
    ESPUI.updateControl( labelHeadingHandle );

    bool power = digitalRead( gpsConfig.gpioDcPowerGood );
    if( power == LOW ) {
      str = "DC power low: not running";
    } else if( powerUnstable == true ){
      str = "DC power unstable ";
      str += (( time_t ) millis() - powerUnstableMillis ) / 1000 ;
      str += " second(s) ago: running";
    } else if( power == HIGH ){
      str = "DC power good: running";
    }
    str += "\nGPS: ";
    if( !existsUBXRelPosNED ){
      str += "Single";
    } else if( relPosNedFresh ){
      str += "Dual";
    } else {
      str += "Dual, heading stale (RelPosNED ";
      str += relPosNedAgeMs;
      str += " ms old)";
    }
    str += "\nAntenna dist. deviation: ";
    str += antDistDeviation;
    str += "\nNAV-PVT to RelPosNED: ";
    str += NavPvtMillis - RelPosNedMillis;
    str += " ms apart";
    str += "\nGGA: ";
    if( GGAReceivedMillis == 0 ){
      str += "none received";
    } else {
      time_t ggaInterval = ( time_t )GGATimestampMillis - ( time_t )previousGGATimestampMillis;
      str += ggaInterval;
      str += " ms fix-to-fix";
    }
    str += "\nOutput to AOG: ";
    addHz( str, outputHzTenths );
    str += "\nOutput interval: ";
    str += gpsHzCurrentMillis;
    str += " ms (min ";
    str += gpsHzMinMillis;
    str += ", max ";
    str += gpsHzMaxMillis;
    str += ")";
    updateLabel( labelGpsReceivers, str, power == LOW or powerUnstable == true );

    str = "<table style='width:100%;text-align:left'>";
    addRateRow( str, navPvtHzTenths );
    bool navPvtStale = addIntervalRow( str, NavPvtMillis, previousNavPvtMillis );
    addRow( str, "Good checksum", NavPvtCount );
    addRow( str, "Bad checksum", diagnostics.badChecksumNavPVTCount );
    addRow( str, "Wrong length", diagnostics.wrongLengthNavPVTCount );
    addRow( str, "Dropped: invalid position", InvalidNavPvtCount );
    addRow( str, "Dropped: position jump", JumpNavPvtCount );
    str += "</table>";
    updateLabel( labelNavPvt, str, navPvtStale );

    str = "<table style='width:100%;text-align:left'>";
    addRateRow( str, relPosNedHzTenths );
    bool relPosNedStale = addIntervalRow( str, RelPosNedMillis, previousRelPosNedMillis );
    addRow( str, "Good checksum", RelPosNedCount );
    addRow( str, "Bad checksum", diagnostics.badChecksumRelPosNEDCount );
    addRow( str, "Wrong length/version", diagnostics.wrongLengthRelPosNEDCount );
    addRow( str, "Dropped: heading not valid", HeadingInvalidRelPosNedCount );
    addRow( str, "Dropped: not RTK fixed", NotFixedRelPosNedCount );
    addRow( str, "Dropped: base data extrapolated", ExtrapolatedRelPosNedCount );
    addRow( str, "Dropped: heading out of range", HeadingRangeRelPosNedCount );
    addRow( str, "Dropped: heading inaccurate", HeadingInaccurateRelPosNedCount );
    addRow( str, "Stale at NAV-PVT (> 2 epochs)", StaleRelPosNedCount );
    str += "</table>";
    updateLabel( labelRelPosNed, str, relPosNedStale );

    str = "Pwm: ";
    str += mphPwm;
    str += " Hz\ngSpeed: ";
    str += UBXPVT1[UBXRingCount1].gSpeed;

    Control* labelPwmOutHandle = ESPUI.getControl( labelPwmOut );
    labelPwmOutHandle->value = str;
    ESPUI.updateControl( labelPwmOutHandle );

    {
      str = "";
      if( gpsConfig.serialNmeaVTGHz ){
        for( uint8_t i = 0; i < sizeOfVTGBuffer; i++ ){
          str += ( char ) VTGBuffer[i];
        }
      }
      if( gpsConfig.serialNmeaGGAHz ){
        for( uint8_t i = 0; i < sizeOfGGABuffer; i++ ){
          str += ( char ) GGABuffer[i];
        }
      }
      if( gpsConfig.serialNmeaHDTHz ){
        for( uint8_t i = 0; i < sizeOfHDTBuffer; i++ ){
          str += ( char ) HDTBuffer[i];
        }
      }
      if( gpsConfig.serialNmeaRMCHz ){
        for( uint8_t i = 0; i < sizeOfRMCBuffer; i++ ){
          str += ( char ) RMCBuffer[i];
        }
      }

      Control* labelGpsMessagesHandle = ESPUI.getControl( labelGpsMessages );
      labelGpsMessagesHandle->value = str;
      ESPUI.updateControl( labelGpsMessagesHandle );

      Control* labelAgOpenGpsAddressHandle = ESPUI.getControl( labelAgOpenGpsAddress );
      time_t seconds = ( millis() - lastHelloReceivedMillis ) / 1000;
      String str;
      str.reserve( 30 );
      str = ipDestination.toString();
      str += "\n";
      str += ( String )seconds;
      str += " seconds ago";
      labelAgOpenGpsAddressHandle->value = str;
      if( seconds > 5 ){
        labelAgOpenGpsAddressHandle->color = ControlColor::Alizarin;
      } else {
        labelAgOpenGpsAddressHandle->color = ControlColor::Turquoise;
      }
      ESPUI.updateControl( labelAgOpenGpsAddressHandle );
    }
  }
}

void initDiagnosticDisplay() {
  xTaskCreate( diagnosticDisplay, "diagnosticDisplay", 4096, NULL, 2, NULL );
}