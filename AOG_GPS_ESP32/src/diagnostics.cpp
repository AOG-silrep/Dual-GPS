
#include "main.hpp"

void diagnosticDisplay ( void* z ){
  constexpr TickType_t xFrequency = 1000;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  String str;
  str.reserve( 500 );

  for( ;; ) {

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
    str += existsUBXRelPosNED ? "Dual" : "Single" ;
    str += "\nAntenna dist. deviation: ";
    str += antDistDeviation;
    str += "\nNAV-PVT message received: ";
    time_t elapsedNavPvtMillis = millis() - NavPvtMillis;
    if( elapsedNavPvtMillis > 1000 ){
      str += ( elapsedNavPvtMillis / 1000 );
      str += " seconds ago";
    } else {
      str += NavPvtMillis - previousNavPvtMillis;
      str += " millis apart";
    }
    str += "\nRelPosNED message received: ";
    time_t elapsedRelPosNedMillis = millis() - RelPosNedMillis;
    if( elapsedRelPosNedMillis > 1000 ){
      str += ( elapsedRelPosNedMillis / 1000 );
      str += " seconds ago";
    } else {
      str += RelPosNedMillis - previousRelPosNedMillis;
      str += " millis apart";
    }
    str += "\nNAV-PVT to RelPosNED difference: ";
    str += NavPvtMillis - RelPosNedMillis;
    str += " millis apart";
    str += "\nGGA: ";
    if( GGAReceivedMillis == 0 ){
      str += "none received";
    } else {
      time_t ggaInterval = ( time_t )GGATimestampMillis - ( time_t )previousGGATimestampMillis;
      str += ggaInterval;
      str += " millisecond fix-to-fix";
    }

    Control* labelGpsReceiversHandle = ESPUI.getControl( labelGpsReceivers );
    if( power == LOW or powerUnstable == true or elapsedNavPvtMillis > 1000 or elapsedRelPosNedMillis > 1000 ){
      labelGpsReceiversHandle->color = ControlColor::Alizarin;
    } else {
      labelGpsReceiversHandle->color = ControlColor::Turquoise;
    }
    labelGpsReceiversHandle->value = str;
    ESPUI.updateControl( labelGpsReceiversHandle );

    str = "Pwm: ";
    str += mphPwm;
    str += " Hz\ngSpeed: ";
    str += UBXPVT1[UBXRingCount1].gSpeed;

    Control* labelPwmOutHandle = ESPUI.getControl( labelPwmOut );
    labelPwmOutHandle->value = str;
    ESPUI.updateControl( labelPwmOutHandle );

    str = "Max millis: ";
    str += gpsHzMaxMillis;
    str += "\nMin millis: ";
    str += gpsHzMinMillis;
    str += "\nCurrent millis: ";
    str += gpsHzCurrentMillis;
    str += "\nNavPVT message count: ";
    str += NavPvtCount;
    str += "\nNavPVT bad checksum: ";
    str += diagnostics.badChecksumNavPVTCount;
    str += "\nNavPVT wrong length: ";
    str += diagnostics.wrongLengthNavPVTCount;
    str += "\nRelPosNED message count: ";
    str += RelPosNedCount;
    str += "\nRelPosNED bad checksum: ";
    str += diagnostics.badChecksumRelPosNEDCount;
    str += "\nRelPosNED wrong length: ";
    str += diagnostics.wrongLengthRelPosNEDCount;

    Control* labelGpsMessageHzHandle = ESPUI.getControl( labelGpsMessageHz );
    labelGpsMessageHzHandle->value = str;
    ESPUI.updateControl( labelGpsMessageHzHandle );

    {
      str = "";
      if( gpsConfig.sendSerialNmeaVTG ){
        for( uint8_t i = 0; i < 55; i++ ){
          str += ( char ) VTGBuffer[i];
        }
      }
      if( gpsConfig.sendSerialNmeaGGA ){
        for( uint8_t i = 0; i < 80; i++ ){
          str += ( char ) GGABuffer[i];
        }
      }
      if( gpsConfig.sendSerialNmeaHDT ){
        for( uint8_t i = 0; i < 24; i++ ){
          str += ( char ) HDTBuffer[i];
        }
      }
      if( gpsConfig.sendSerialNmeaRMC ){
        for( uint8_t i = 0; i < 80; i++ ){
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

		vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void initDiagnosticDisplay() {
  xTaskCreate( diagnosticDisplay, "diagnosticDisplay", 2048, NULL, 2, NULL );
}