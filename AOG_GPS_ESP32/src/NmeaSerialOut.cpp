
#include "main.hpp"
uint64_t previousMillis = 0;
uint16_t nmeaMessageDelay;

void NmeaOut ( void* z ){
  TickType_t xFrequency = (( uint16_t ) 1000 / gpsConfig.serialNmeaMessagesPerSec ) - 10;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  NmeaQueueMessage message;

	for( ;;  ){

    if( gpsConfig.sendSerialNmeaGGA ){
      if( xQueueReceive( GGAQueue, &message, 0 ) == pdTRUE ){
        NmeaTransmitter.write( message.data, message.length );
      }
    }
    if( gpsConfig.sendSerialNmeaVTG ){
      if( xQueueReceive( VTGQueue, &message, 0 ) == pdTRUE ){
        NmeaTransmitter.write( message.data, message.length );
      }
    }
    if( gpsConfig.sendSerialNmeaHDT ){
      if( xQueueReceive( HDTQueue, &message, 0 ) == pdTRUE ){
        NmeaTransmitter.write( message.data, message.length );
      }
    }
    if( gpsConfig.sendSerialNmeaRMC ){
      if( xQueueReceive( RMCQueue, &message, 0 ) == pdTRUE ){
        NmeaTransmitter.write( message.data, message.length );
      }
    }
    xTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void initNmeaOut( ){
  xTaskCreate( NmeaOut, "NmeaOut", 3096, NULL, 5, NULL );
}