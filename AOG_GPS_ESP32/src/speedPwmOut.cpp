
#include "main.hpp"

uint32_t mphPwm;

void SpeedPWM ( void* z ){
  constexpr TickType_t xFrequency = 83; // 12hz
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for( ;; ) {

    mphPwm = 0;
    for( uint8_t i = 0; i < sizeOfUBXArray; i ++ ){
      mphPwm += UBXPVT1[i].gSpeed; // average
    }
    mphPwm /= sizeOfUBXArray;
    mphPwm *= gpsConfig.velocityHzPerMPH;
    mphPwm = (( double ) mphPwm ) * 0.22; // 1 gSpeed = 0.0036 km/h = 0.0022369363 mile/h
    mphPwm /= 100;
    if( isnan( mphPwm )){
      mphPwm = 0;
    }
    ledcWriteTone( gpsConfig.gpioVelocityPWM, mphPwm );
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void initSpeedPWM() {

  ledcAttach( gpsConfig.gpioVelocityPWM, gpsConfig.velocityHzPerMPH, LEDC_TIMER_10_BIT );
  ledcWrite( gpsConfig.gpioVelocityPWM, 512 ); // 50% duty cycle
  xTaskCreate( SpeedPWM, "SpeedPWM", 4096, NULL, 2, NULL );
}
