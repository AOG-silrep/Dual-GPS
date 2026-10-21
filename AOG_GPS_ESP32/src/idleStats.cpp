

#include <ESPUI.h>
#include "esp_freertos_hooks.h"
#include "esp_heap_caps.h"

#include "main.hpp"

volatile uint16_t idleCtrCore0 = 0;
volatile uint16_t idleCtrCore1 = 0;

bool core0IdleWorker( void ) {
  static TickType_t xLastWakeTime0;

  if( xLastWakeTime0 != xTaskGetTickCount() ) {
    xLastWakeTime0 = xTaskGetTickCount();
    idleCtrCore0 = idleCtrCore0 + 1;
  }

  return true;
}

bool core1IdleWorker( void ) {
  static TickType_t xLastWakeTime1;

  if( xLastWakeTime1 != xTaskGetTickCount() ) {
    xLastWakeTime1 = xTaskGetTickCount();
    idleCtrCore1 = idleCtrCore1 + 1;
  }

  return true;
}

void idleStatsWorker( void* z ) {
  constexpr TickType_t xFrequency = 1000;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  String str;
  str.reserve( 500 );

  multi_heap_info_t heapInfo;
  for( ;; ) {
    heap_caps_get_info( &heapInfo, MALLOC_CAP_8BIT );

    str = "Core0: ";
    str += ( 1000 - idleCtrCore0 ) / 10;
    str += "‰<br/>";
    str += "Core1: ";
    str += ( 1000 - idleCtrCore1 ) / 10;
    str += "‰<br/>Uptime: ";
    str += millis() / 1000;
    str += "s<br/>Heap free: ";
    str += heapInfo.total_free_bytes / 1024;
    str += "kB (";
    str += heapInfo.free_blocks;
    str += "), allocated: ";
    str += heapInfo.total_allocated_bytes / 1024;
    str += "kB (";
    str += heapInfo.allocated_blocks;
    str += ")<br/>Lowest ever free Heap: ";
    str += heapInfo.minimum_free_bytes / 1024;
    str += "kB<br/>Largest free block on Heap: ";
    str += heapInfo.largest_free_block / 1024;
    str += "kB";

    Control* labelLoadHandle = ESPUI.getControl( labelLoad );
    labelLoadHandle->value = str;
    ESPUI.updateControl( labelLoadHandle );

    idleCtrCore0 = 0;
    idleCtrCore1 = 0;

//   heap_caps_print_heap_info(MALLOC_CAP_8BIT);

    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void initIdleStats() {
  esp_register_freertos_idle_hook_for_cpu( core0IdleWorker, 0 );
  esp_register_freertos_idle_hook_for_cpu( core1IdleWorker, 1 );
  xTaskCreate( idleStatsWorker, "IdleStats", 4096, NULL, 2, NULL );//display only, like diagnosticDisplay
}
