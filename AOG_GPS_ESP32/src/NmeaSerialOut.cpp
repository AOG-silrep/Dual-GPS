
#include "main.hpp"
TaskHandle_t nmeaOutTask = NULL;

void NmeaOut ( void* z ){
  NmeaQueueMessage message;

	for( ;;  ){
    //headingAndPosition queues only the sentences due this epoch, then notifies
    ulTaskNotifyTake( pdTRUE, portMAX_DELAY );

    if( xQueueReceive( GGAQueue, &message, 0 ) == pdTRUE ){
      NmeaTransmitter.write( message.data, message.length );
    }
    if( xQueueReceive( VTGQueue, &message, 0 ) == pdTRUE ){
      NmeaTransmitter.write( message.data, message.length );
    }
    if( xQueueReceive( HDTQueue, &message, 0 ) == pdTRUE ){
      NmeaTransmitter.write( message.data, message.length );
    }
    if( xQueueReceive( RMCQueue, &message, 0 ) == pdTRUE ){
      NmeaTransmitter.write( message.data, message.length );
    }
  }
}

void initNmeaOut( ){
  xTaskCreate( NmeaOut, "NmeaOut", 3096, NULL, 5, &nmeaOutTask );
}
