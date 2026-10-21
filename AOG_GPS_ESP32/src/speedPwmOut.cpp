
#include "main.hpp"

uint32_t mphPwm;

//1 mm/s = 3600 / 1609344 mph = 225 / 100584 mph; integer, so no floating point
constexpr uint64_t mphPerMmPerSecNum = 225;
constexpr uint64_t mphPerMmPerSecDen = 100584;
constexpr int32_t maxPwmSpeed = 30000; //mm/s, 30 m/s like the NAV-PVT jump check; keeps the math in 64 bit
constexpr uint32_t maxPwmHz = 78125;   //10 bit LEDC from the 80 MHz APB clock, above this ledcWriteTone fails (div_param=0)

static uint64_t clampSpeed( int32_t gSpeed ){
  return gSpeed < 0 ? 0 : ( gSpeed > maxPwmSpeed ? maxPwmSpeed : gSpeed );
}

void SpeedPWM ( void* z ){
  constexpr TickType_t xFrequency = 83; // 12hz
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t lastHz = UINT32_MAX; // write the first value

  for( ;; ) {

    //the last two accepted NAV-PVT; SerialTraffic writes the next one byte by byte into the third slot
    const uint8_t ringCount1 = UBXRingCount1;
    const uint64_t speedSum = clampSpeed( UBXPVT1[ringCount1].gSpeed )
      + clampSpeed( UBXPVT1[( ringCount1 + sizeOfUBXArray - 1 ) % sizeOfUBXArray].gSpeed );
    const int hzPerMph = gpsConfig.velocityHzPerMPH;
    uint64_t hz = 0;
    if( hzPerMph > 0 ){
      hz = ( speedSum * hzPerMph * mphPerMmPerSecNum + mphPerMmPerSecDen ) / ( 2 * mphPerMmPerSecDen ); // average of 2, rounded
    }
    mphPwm = hz > maxPwmHz ? maxPwmHz : hz;
    if( mphPwm != lastHz ){ // each call reprograms the LEDC timer
      ledcWriteTone( gpsConfig.gpioVelocityPWM, mphPwm );
      lastHz = mphPwm;
    }
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void initSpeedPWM() {

  ledcAttach( gpsConfig.gpioVelocityPWM, gpsConfig.velocityHzPerMPH, LEDC_TIMER_10_BIT );
  ledcWrite( gpsConfig.gpioVelocityPWM, 512 ); // 50% duty cycle
  xTaskCreate( SpeedPWM, "SpeedPWM", 4096, NULL, 2, NULL );
}
