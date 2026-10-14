
#include <AsyncUDP.h>
#include "main.hpp"

//Kalman filter roll
double rollK, rollPc, rollG, rollXp, rollZp, rollXe;
double rollP = 1.0;
double rollVar = 0.1; // variance, smaller: faster, less filtering
double rollVarProcess = 0.3;// 0.0005;// 0.0003;  bigger: faster, less filtering// replaced by fast/slow depending on GPS quality
//Kalman filter heading
double headK, headPc, headG, headXp, headZp, headXe;
double headP = 1.0;
double headVar = 0.1; // variance, smaller, more faster filtering
double headVarProcess = 0.1;// 0.001;//  bigger: faster, less filtering// replaced by fast/slow depending on GPS quality
//Kalman filter heading
double headVTGK, headVTGPc, headVTGG, headVTGXp, headVTGZp, headVTGXe;
double headVTGP = 1.0;
double headVTGVar = 0.1; // variance, smaller, more faster filtering
double headVTGVarProcess = 0.01;// 0.001;//  bigger: faster, less filtering// replaced by fast/slow depending on GPS quality
//Kalman filter heading
double headMixK, headMixPc, headMixG, headMixXp, headMixZp, headMixXe;
double headMixP = 1.0;
double headMixVar = 0.1; // variance, smaller, more faster filtering
double headMixVarProcess = 0.1;// 0.001;//  bigger: faster, less filtering// replaced by fast/slow depending on GPS quality
//Kalman filter lat
double latK, latPc, latG, latXp, latZp, latXe;
double latP = 1.0;
double latVar = 0.1; // variance, smaller, more faster filtering
double latVarProcess = 0.3;//  replaced by fast/slow depending on GPS qaulity
//Kalman filter lon
double lonK, lonPc, lonG, lonXp, lonZp, lonXe;
double lonP = 1.0;
double lonVar = 0.1; // variance, smaller, more faster filtering
double lonVarProcess = 0.3;// replaced by fast/slow depending on GPS quality

double VarProcessVeryFast = 0.2;//0,3  used, when GPS signal is weak, no roll, but heading OK
double VarProcessFast = 0.08;//0,15  used, when GPS signal is weak, no roll, but heading OK
double VarProcessMedi = 0.02;//0,08 used, when GPS signal is  weak, no roll no heading
double VarProcessSlow = 0.001;//  0,004used, when GPS signal is  weak, no roll no heading
double VarProcessVerySlow = 0.0001;//0,03  used, when GPS signal is  weak, no roll no heading
bool filterGPSpos = false;

double HeadingQuotaVTG = 0.5;

//UBX
uint8_t UBXRingCount1 = 0, UBXRingCount2 = 0, UBXDigit1 = 0, UBXDigit2 = 0, OGIfromUBX = 0;
short UBXLength1 = 100, UBXLength2 = 100;
bool isUBXPVT1 = false,  isUBXRelPosNED = false, existsUBXRelPosNED = false;

double virtLat = 0.0, virtLon = 0.0;//virtual Antenna Position

// ai, 07.10.2020: use the GGA Message to determine Fix-Quality
bool bNMEAstarted = false, bGGAexists = false;
String sNMEA;
int16_t i, iPos;
int8_t cFixQualGGA;
// END ai, 07.10.2020: use the GGA Message to determine Fix-Quality

time_t previousMillisHz;
time_t gpsHzMaxMillis;
time_t gpsHzMinMillis;
time_t gpsHzCurrentMillis;

//heading + roll
double HeadingRelPosNED = 0, cosHeadRelPosNED = 1, HeadingVTG = 0, HeadingVTGOld = 0, cosHeadVTG = 1, HeadingMix = 0, cosHeadMix = 1;
double HeadingDiff = 0, HeadingMax = 0, HeadingMin = 0, HeadingMixBak = 0, HeadingQualFactor = 0.5;
uint8_t noRollCount = 0,  drivDirect = 0;
bool dualGPSHeadingPresent = false, rollPresent = false, virtAntPosPresent = false, add360ToRelPosNED = false, add360ToVTG = false;
double roll = 0.0, rollToAOG = 0.0;
uint8_t dualAntNoValueCount = 0, dualAntNoValueMax = 20;// if dual Ant value not valid for xx times, send position without correction/heading/roll
int16_t antDistDeviation;

NAV_PVT UBXPVT1[sizeOfUBXArray];
NAV_RELPOSNED UBXRelPosNED[sizeOfUBXArray];

QueueHandle_t HDTQueue = xQueueCreate( 1, sizeof( NmeaQueueMessage ));
QueueHandle_t VTGQueue = xQueueCreate( 1, sizeof( NmeaQueueMessage ));
QueueHandle_t GGAQueue = xQueueCreate( 1, sizeof( NmeaQueueMessage ));
QueueHandle_t RMCQueue = xQueueCreate( 1, sizeof( NmeaQueueMessage ));

void queueSerialNmea( QueueHandle_t queue, const uint8_t* buffer, uint8_t length ){
	NmeaQueueMessage message;
	message.length = length;
	memcpy( message.data, buffer, length );
	xQueueOverwrite( queue, &message );
}

bool powerUnstable = false;
time_t powerUnstableMillis = 0;
uint8_t udpHelloCounter;

// Variables ------------------------------


void headingRollCalc( ){

	rollPresent = false;
	dualGPSHeadingPresent = false;
	filterGPSpos = false;
	add360ToRelPosNED = false;
	add360ToVTG = false;

	if( existsUBXRelPosNED  ){
		//check if all values are vaild
		if(( bitRead( UBXRelPosNED[UBXRingCount2].flags, 0 )) || ( gpsConfig.checkUBXFlags == 0 )){//gnssFixOK?
			if( bitRead( UBXRelPosNED[UBXRingCount2].flags, 2 ) || ( gpsConfig.checkUBXFlags == 0 )){//1 if relative position components and moving baseline are valid
			//RelPosNED OK: heading + roll calc												 

				antDistDeviation = UBXRelPosNED[UBXRingCount2].relPosLength - gpsConfig.AntDist;
				if(( UBXRelPosNED[UBXRingCount2].relPosLength > ( gpsConfig.AntDist / gpsConfig.AntDistDeviationFactor )) && ( UBXRelPosNED[UBXRingCount2].relPosLength < ( gpsConfig.AntDist * gpsConfig.AntDistDeviationFactor )))
				{
					//check if vector length between antennas is in range = indicator of heading/roll quality

					dualAntNoValueCount = 0;//reset watchdog

					if(( UBXRelPosNED[UBXRingCount2].relPosLength > ( gpsConfig.AntDist / (( gpsConfig.AntDistDeviationFactor / 4 ) + 0.75 ))) && ( UBXRelPosNED[UBXRingCount2].relPosLength < ( gpsConfig.AntDist * (( gpsConfig.AntDistDeviationFactor / 4 ) + 0.75 ))))
					{	//check if vector length between antennas is in range = indicator of heading/roll quality							
						//for roll calc only 1/4 deviation !!

				//signal perfect: deviation less than 1/4
						//set filters for heading
						if( UBXPVT1[UBXRingCount1].gSpeed > 500 ) //driving at least 1.8km/h
						{
							headVarProcess = VarProcessFast;  //set Kalman filter
							headVTGVarProcess = VarProcessMedi; //set Kalman filter
							if( drivDirect < 2  ){//forewards or unknown
								if( UBXRelPosNED[UBXRingCount2].relPosHeading != 0  ){//RelPosNED ok								
									HeadingQualFactor = 0.8;//0.7
									HeadingQuotaVTG = 0.0;
								}
								else {//RelPosNED fails
									HeadingQualFactor = 0.4;//0,5
									HeadingQuotaVTG = 1.0;//0.3
								}
							}
							else { //backwards
								HeadingQualFactor = 0.6;
								HeadingQuotaVTG = 0.0;
							}
						}
						else {//low speed
							HeadingQualFactor = 0.4;
							HeadingQuotaVTG = 0.0;
							headVarProcess = VarProcessMedi;  //set Kalman filter
							headVTGVarProcess = VarProcessSlow; //set Kalman filter
						}

						if((( gpsConfig.headingAngleCorrection > 70 ) && ( gpsConfig.headingAngleCorrection < 110 ))
							|| (( gpsConfig.headingAngleCorrection > 250 ) && ( gpsConfig.headingAngleCorrection < 290 )))
						{//ant left+right: headingCorrectionAngle 90 or 270
				//roll		
							if((( UBXPVT1[UBXRingCount1].gSpeed > 3000 ) && ( abs( UBXRelPosNED[UBXRingCount2].relPosD ) < ( gpsConfig.AntDist * 3 )))
								|| (( UBXPVT1[UBXRingCount1].gSpeed <= 3000 ) && ( abs( UBXRelPosNED[UBXRingCount2].relPosD ) < ( gpsConfig.AntDist * 2 )))){//50% = 26.7° max
								roll = ( atan2(( double( UBXRelPosNED[UBXRingCount2].relPosD ) + ( double( UBXRelPosNED[UBXRingCount2].relPosHPD ) / 100 )), gpsConfig.AntDist )) / PI180;
								roll -= gpsConfig.rollAngleCorrection;
								noRollCount = 0;
								rollVarProcess = VarProcessVeryFast; //set Kalman filter fast
								if( gpsConfig.debugmodeHeading ){ Serial.print( "roll calc: " ); Serial.print( roll ); }
							}
							else {
								rollVarProcess = VarProcessSlow;
								roll = roll * 0.92;
								noRollCount++;
							}
						}
						if( gpsConfig.debugmodeFilterPos ){ Serial.println( "perfect Signal" ); }
					}
					else
						//signal medium
					{
						if( UBXPVT1[UBXRingCount1].gSpeed > 500 ){ //driving at least 1.8km/h
							if( drivDirect < 2 )//forewards or unknown
							{
								HeadingQualFactor = 0.45;//0,5
								HeadingQuotaVTG = double( UBXPVT1[UBXRingCount1].gSpeed ) / double( 8000 );//10000    // 24.03.2020
								if( HeadingQuotaVTG > 1.0 ){ HeadingQuotaVTG = 1.0; }//at x km/h use only VTG
							}
							else {
								HeadingQualFactor = 0.5;
								HeadingQuotaVTG = 0.0;
							}
						}
						else {
							HeadingQualFactor = 0.35;//0,4
							HeadingQuotaVTG = 0.0;
						}
						headVarProcess = VarProcessFast; //set Kalman filter
						headVTGVarProcess = VarProcessSlow; //set Kalman filter
						rollVarProcess = VarProcessVerySlow; //roll slowly to 0  24.03.2020
						roll = roll * 0.92;
						noRollCount++;
						//start filter GPS pos, set Kalman values
						if( gpsConfig.filterGPSposOnWeakSignal == 1 ){
							filterGPSpos = true;
							latVarProcess = VarProcessVeryFast; //set Kalman filter
							lonVarProcess = VarProcessVeryFast;
						}

						if( gpsConfig.debugmodeFilterPos ){ Serial.println( "medium Signal" ); }
						if( gpsConfig.debugmode ){
							Serial.println( "poor quality of GPS signal: NO roll calc, heading calc OK" );
							Serial.print( "Antenna distance set: " ); Serial.print( gpsConfig.AntDist ); Serial.print( "  Ant. dist from GPS: " ); Serial.println( UBXRelPosNED[UBXRingCount2].relPosLength );
						}
					}//end of deviation good or perfect					
				//heading
					HeadingRelPosNED = ( double( UBXRelPosNED[UBXRingCount2].relPosHeading ) * 0.00001 ) + gpsConfig.headingAngleCorrection;
					if( HeadingRelPosNED >= 360 ){ HeadingRelPosNED -= 360; }
					//go to cos for filtering to avoid 360-0° jump
					cosHeadRelPosNED = cos(( HeadingRelPosNED * PI180 ));
					if( gpsConfig.debugmodeRAW ){
						Serial.print( "GPShead cos filtCos filtGPSHead" ); Serial.print( "," );
						Serial.print( HeadingRelPosNED ); Serial.print( "," );
						Serial.print( cosHeadRelPosNED, 4 ); Serial.print( "," );
					}

					if( gpsConfig.debugmodeHeading ){ Serial.print( "RelPosNED heading calc: " ); Serial.print( HeadingRelPosNED ); }
					//Kalman filter heading
					headK = cosHeadRelPosNED;//input
					headPc = headP + headVarProcess;
					headG = headPc / ( headPc + headVar );
					headP = ( 1 - headG ) * headPc;
					headXp = headXe;
					headZp = headXp;
					headXe = ( headG * ( headK - headZp )) + headXp;//result
					cosHeadRelPosNED = headXe;
					//go back to degree
					if( HeadingRelPosNED <= 180 )
					{
						HeadingRelPosNED = acos( headXe ) / PI180;
					}
					else { HeadingRelPosNED = double( 360.0 ) - ( acos( headXe ) / PI180 ); }

					if( gpsConfig.debugmodeRAW ){
						Serial.print( headXe, 4 ); Serial.print( "," );
						Serial.print( HeadingRelPosNED ); Serial.print( "," );
					}

					dualGPSHeadingPresent = true;

					if( gpsConfig.debugmodeHeading ){
						Serial.print( "heading filterd: " ); Serial.print( HeadingRelPosNED );
						Serial.print( " Heading Diff per sec: " ); Serial.print( HeadingDiff * 1000 );
					}

					//filter roll
					rollK = roll;//input
					rollPc = rollP + rollVarProcess;
					rollG = rollPc / ( rollPc + rollVar );
					rollP = ( 1 - rollG ) * rollPc;
					rollXp = rollXe;
					rollZp = rollXp;
					rollXe = ( rollG * ( rollK - rollZp )) + rollXp;//result					

					if( gpsConfig.debugmodeRAW ){
						Serial.print( "roll filtRoll," );
						Serial.print( roll ); Serial.print( "," );
						Serial.print( rollXe ); Serial.print( "," );
					}

					roll = rollXe;
					if( noRollCount < 40 ){ rollPresent = true; }//24.03.2020
					else { noRollCount = 42; rollPresent = false; }//prevent overflow
					if( gpsConfig.debugmodeHeading ){
						Serial.print( " roll filtered: " ); Serial.println( roll );
						Serial.print( "Antenna distance set: " ); Serial.print( gpsConfig.AntDist ); Serial.print( "  Ant. dist from GPS: " ); Serial.println( UBXRelPosNED[UBXRingCount2].relPosLength );
					}
					if( gpsConfig.debugmodeRAW ){
						Serial.print( "MaxHeadingDiff," ); Serial.print( HeadingDiff );
						Serial.print( ",AntDist AntDisGPS," );
						Serial.print( gpsConfig.AntDist ); Serial.print( "," ); Serial.print( UBXRelPosNED[UBXRingCount2].relPosLength ); Serial.print( "," );
					}
				}

				//very poor signal quality, or one antenna
				else {
					roll = roll * 0.9;//go slowly to 0
					dualAntNoValueCount++;
					if( dualAntNoValueCount < dualAntNoValueMax ){ dualGPSHeadingPresent = true; }
					else { if( dualAntNoValueCount > 200 ){ dualAntNoValueCount = 220; }					}
					HeadingQualFactor = 0.4;//45
					//set Kalman filter for VTG heading
					if( UBXPVT1[UBXRingCount1].gSpeed > 100 ) //driving at least 0.36km/h
						if( UBXPVT1[UBXRingCount1].gSpeed > 1000 ) //driving at least 3.6km/h
						{
							headVTGVarProcess = VarProcessVerySlow;
						}
						else { headVTGVarProcess = VarProcessVerySlow * 0.1; }
					else { headVTGVarProcess = VarProcessVerySlow * 0.001; }


					HeadingQuotaVTG = double( UBXPVT1[UBXRingCount1].gSpeed ) / double( 3000 );
					if( HeadingQuotaVTG > 1.0 ){ HeadingQuotaVTG = 1.0; }

					if( gpsConfig.filterGPSposOnWeakSignal == 1 ){
						filterGPSpos = true;
						latVarProcess = VarProcessFast; //set Kalman filter
						lonVarProcess = VarProcessFast;
					}
					if( gpsConfig.debugmodeFilterPos ){ Serial.println( "very weak Signal, or only 1 Antenna" ); }
					if( gpsConfig.debugmode || gpsConfig.debugmodeHeading ){
						Serial.println( "poor quality of GPS signal, or only 1 Antenna: NO heading/roll calc" );
						Serial.print( "Antenna distance set: " ); Serial.print( gpsConfig.AntDist ); Serial.print( "  Ant. dist from GPS: " ); Serial.println( UBXRelPosNED[UBXRingCount2].relPosLength );
					}
					if( gpsConfig.debugmodeRAW ){ Serial.print( ",,,,,,,,,,,,," ); }
				}
			}

			//do this, if GNSFix is OK:

			//HeadingVTG Kalman filter go to cos for filtering to avoid 360-0° jump
			//cosHeadVTG = cos(( HeadingVTG * 0.6 ) + ( UBXPVT1[UBXRingCount1].headMot * 0.000004 * PI180 ));
			cosHeadVTG = cos(( UBXPVT1[UBXRingCount1].headMot * 0.00001 * PI180 ));
			headVTGK = cosHeadVTG;//input
			if( abs( cosHeadVTG ) > 0.98 ){ headVTGPc = headVTGP + ( headVTGVarProcess * 10 ); }//"open" filter in 356-4 deg region
			else { headVTGPc = headVTGP + headVTGVarProcess; }
			headVTGG = headVTGPc / ( headVTGPc + headVTGVar );
			headVTGP = ( 1 - headVTGG ) * headVTGPc;
			headVTGXp = headVTGXe;
			headVTGZp = headVTGXp;
			headVTGXe = ( headVTGG * ( headVTGK - headVTGZp )) + headVTGXp;//result

			cosHeadVTG = headVTGXe;

			//go back to degree
			HeadingVTGOld = HeadingVTG;//bak old value for constrain
			if( UBXPVT1[UBXRingCount1].headMot <= 18000000 )
			{
				HeadingVTG = acos( headVTGXe ) / PI180;
			}
			else { HeadingVTG = double( 360.0 ) - ( acos( headVTGXe ) / PI180 ); }

			if( gpsConfig.debugmodeHeading ){ Serial.print( "VTG heading ( only from Ant R ) filtered: " ); Serial.println( HeadingVTG ); }

			if( gpsConfig.debugmodeRAW ){
				Serial.print( "headVTG filtCosHeadVTG filtHeadVTG," );
				Serial.print( float( UBXPVT1[UBXRingCount1].headMot ) * 0.00001 ); Serial.print( "," );
				Serial.print( cosHeadVTG, 4 ); Serial.print( "," );
				Serial.print( HeadingVTG ); Serial.print( "," );
			}

			//driving direction calcs done here, after HeadingVTG and dual heading was filtered
			if( dualAntNoValueCount < ( dualAntNoValueMax * 3 )){
				if( UBXPVT1[UBXRingCount1].gSpeed > 100 ){//driving at least 0.36 km/h
					drivDirect = 2;     //set to backwards
					if( abs( HeadingRelPosNED - HeadingVTG ) <= 60 ){  //40 330 20         // almost same direction = forewards
						drivDirect = 1;
					}
					else {
						if(( HeadingRelPosNED > 305 ) && ( HeadingVTG < 55 )){
							drivDirect = 1;
							add360ToVTG = true;
						}
						if(( HeadingRelPosNED < 55 ) && ( HeadingVTG > 305 )){
							drivDirect = 1;
							add360ToRelPosNED = true;
						}
					}
				}
				else { //too slow
					drivDirect = 0;
				}
			}
			else { drivDirect = 0; }
			if( gpsConfig.debugmodeHeading ){
				Serial.print( "driving direction: " ); Serial.println( drivDirect );
				Serial.print( "Heading quota VTG :" ); Serial.println( HeadingQuotaVTG );
			}

		}
		else { if( gpsConfig.debugmode ){ Serial.println( "UBX RelPosNED flag: relative position not valid ->  NO heading + roll calc" ); } }
	}

//single Antenna
	else { 
		HeadingQualFactor = 0.4;//0,5
		drivDirect = 0;// 0 = unknown
		HeadingQuotaVTG = 1.0;
		//set Kalman filter for VTG heading
		if( UBXPVT1[UBXRingCount1].gSpeed > 100 ) //driving at least 0.36km/h
			if( UBXPVT1[UBXRingCount1].gSpeed > 1000 ) //driving at least 3.6km/h
			{
				headVTGVarProcess = VarProcessSlow;
			}
			else { headVTGVarProcess = VarProcessVerySlow; }
		else { headVTGVarProcess = VarProcessVerySlow * 0.1; }

		//HeadingVTG Kalman filter go to cos for filtering to avoid 360-0° jump
		cosHeadVTG = cos(( UBXPVT1[UBXRingCount1].headMot * 0.00001 * PI180 ));
		headVTGK = cosHeadVTG;//input
		if( abs( cosHeadVTG ) > 0.98 ){ headVTGPc = headVTGP + ( headVTGVarProcess * 10 ); }//"open" filter in 356-4 deg region
		else { headVTGPc = headVTGP + headVTGVarProcess; }
		headVTGG = headVTGPc / ( headVTGPc + headVTGVar );
		headVTGP = ( 1 - headVTGG ) * headVTGPc;
		headVTGXp = headVTGXe;
		headVTGZp = headVTGXp;
		headVTGXe = ( headVTGG * ( headVTGK - headVTGZp )) + headVTGXp;//result
		cosHeadVTG = headVTGXe;
		//go back to degree
		HeadingVTGOld = HeadingVTG;//bak old value for constrain
		if( UBXPVT1[UBXRingCount1].headMot <= 18000000 )
		{
			HeadingVTG = acos( headVTGXe ) / PI180;
		}
		else { HeadingVTG = double( 360.0 ) - ( acos( headVTGXe ) / PI180 ); }

		if(( gpsConfig.debugmodeHeading ) || ( gpsConfig.debugmodeVirtAnt )){ Serial.println( "UBX RelPosNED not present ( single Antenna ), or checksums invalid" ); }
	}//end single antenna


	//roll: filter before sending to AOG, if roll corrected position is send
	if( gpsConfig.GPSPosCorrByRoll ){ rollToAOG = ( rollToAOG * 0.7 ) + ( roll * 0.3 ); }
	else { rollToAOG = roll; }

	//heading
	if( UBXPVT1[UBXRingCount1].gSpeed > 5 ){//prevent /0
		HeadingDiff = 2.5 + ( double( gpsConfig.MaxHeadChangPerSec ) * ( UBXPVT1[UBXRingCount1].iTOW - UBXPVT1[( UBXRingCount1 + sizeOfUBXArray - 1 ) % sizeOfUBXArray].iTOW )) / ( double( UBXPVT1[UBXRingCount1].gSpeed )*0.72 );
	}
	else { HeadingDiff = 200; }
	
	HeadingMin = HeadingVTGOld - HeadingDiff;
	HeadingMax = HeadingVTGOld + HeadingDiff;
	//if(( HeadingVTG > HeadingDiff ) && ( HeadingVTG < ( 360 - HeadingDiff )) && ( UBXPVT1[UBXRingCount1].gSpeed > 1000 )){
	if(( HeadingVTG > HeadingDiff ) && ( HeadingVTG < ( 360 - HeadingDiff )) && ( abs( HeadingVTGOld - HeadingVTG ) < 150 ) && ( UBXPVT1[UBXRingCount1].gSpeed > 1000 )){
		if( gpsConfig.debugmodeRAW ){
			Serial.print( "HeadingVTG VTGconstrain," );
			Serial.print( HeadingVTG ); Serial.print( "," );
		}
		HeadingVTG = constrain( HeadingVTG, HeadingMin, HeadingMax );
		if( gpsConfig.debugmodeHeading ){
			Serial.print( "VTGLimits: HeadMin: " ); Serial.print( HeadingMin ); Serial.print( " HeadMax: " );
			Serial.print( HeadingMax ); Serial.print( " Heading VTG: " ); Serial.println( HeadingVTG );
		}
		if( gpsConfig.debugmodeRAW ){ Serial.print( HeadingVTG ); Serial.print( "," ); }
	}
	else { if( gpsConfig.debugmodeRAW ){ Serial.print( "VTG NO constrain, , ," ); } }


	HeadingMixBak = HeadingMix;

	if( existsUBXRelPosNED ){
		if( drivDirect < 2 ){//calc HeadingMix with VTG and RelPosNED heading
			if(( abs( HeadingRelPosNED - HeadingVTG )) <= 20 ){
				HeadingMix = (( HeadingRelPosNED * ( 1.0 - HeadingQuotaVTG )) + ( HeadingVTG * HeadingQuotaVTG ));
			}
			else {
				if( add360ToVTG ){
					HeadingMix = (( HeadingRelPosNED * ( 1.0 - HeadingQuotaVTG )) + (( HeadingVTG + 360 ) * HeadingQuotaVTG ));
				}
				else if( add360ToRelPosNED ){
					HeadingMix = ((( HeadingRelPosNED + 360 ) * ( 1.0 - HeadingQuotaVTG )) + ( HeadingVTG * HeadingQuotaVTG ));
				}
				else{ HeadingMix = HeadingRelPosNED; } //when GPS is above steer axle
			}
			if( HeadingMix > 360 ){
				HeadingMix -= 360;
			}
		}
		else { HeadingMix = HeadingRelPosNED; }//backwards
	}
	else { HeadingMix = HeadingVTG; }//single Antenna

	if( abs( HeadingMixBak - HeadingMix ) <= 20 ){   // use old and new HeadingMix values 
		HeadingMix = ( HeadingMixBak * ( double( 1 ) - HeadingQualFactor )) + ( HeadingMix * HeadingQualFactor );
	}
	else {
		if(( HeadingMixBak > 340 ) && ( HeadingMix < 20 )){HeadingMix += 360;	}
		if(( HeadingMixBak < 20 ) && ( HeadingMix > 340 )){HeadingMixBak += 360;}

		HeadingMix = (( HeadingMixBak * ( double( 1 ) - HeadingQualFactor )) + ( HeadingMix * HeadingQualFactor ));

		if( HeadingMix > 360 ){	HeadingMix -= 360;}
	}


	if( gpsConfig.debugmodeRAW ){
		Serial.print( "VTGQuota HeadQualFac HeadingVTG HeadingRelPos HeadingMixUnLim," );
		Serial.print( HeadingQuotaVTG ); Serial.print( "," );
		Serial.print( HeadingQualFactor ); Serial.print( "," );
		Serial.print( HeadingVTG ); Serial.print( "," );
		Serial.print( HeadingRelPosNED ); Serial.print( "," );
		Serial.print( HeadingMix ); Serial.print( "," );
	}
/*
	HeadingMin = HeadingMixBak - HeadingDiff;
	HeadingMax = HeadingMixBak + HeadingDiff;
	if(( HeadingMixBak > HeadingDiff ) && ( HeadingMixBak < ( 360-HeadingDiff ))){//360 to 0
		HeadingMix = constrain( HeadingMix, HeadingMin, HeadingMax );
		if( gpsConfig.debugmodeHeading ){
			Serial.print( "MixLimits: HeadMin: " ); Serial.print( HeadingMin ); Serial.print( " HeadMax: " );
			Serial.print( HeadingMax ); Serial.print( " Heading Mix: " ); Serial.println( HeadingMix );
		}
		if( gpsConfig.debugmodeRAW ){Serial.print( "constrain used," );}
	}
	else { if( gpsConfig.debugmodeRAW ){ Serial.print( "NO constrain," ); } }
*/
	if( gpsConfig.debugmodeRAW ){
	//	Serial.print( "headDiffMin HeadDiffMax HeadingMix," );
	//	Serial.print( HeadingMin ); Serial.print( "," );
	//	Serial.print( HeadingMax ); Serial.print( "," );
	//	Serial.print( HeadingMix ); Serial.print( "," );
		Serial.print( "NoRollCount DualAntNoValueCount," );
		Serial.print( noRollCount ); Serial.print( "," );
		Serial.print( dualAntNoValueCount ); Serial.print( "," );
	}
}



//-------------------------------------------------------------------------------------------------

void virtualAntennaPoint( ){
	double virtLatRad = double( UBXPVT1[UBXRingCount1].lat ) / double( 10000000 ) * PI180;
	double virtLonRad = double( UBXPVT1[UBXRingCount1].lon ) / double( 10000000 ) * PI180;
	double virtLatRadTemp = 0.0, virtAntHeadingDiff = 0.0, headingRad = 0.0, WayByRadius = 0.0;

	double toRight = 0;
	virtAntPosPresent = false;

	if( rollPresent && ( gpsConfig.GPSPosCorrByRoll == 1 )){ //calculate virtual Antenna point using roll
		toRight = tan( roll * PI180 ) * gpsConfig.AntHeight;
		if( gpsConfig.debugmodeVirtAnt ){
			Serial.print( "roll degr: " ); Serial.print( roll, 2 ); Serial.print( " to right by roll: " );
			Serial.println( toRight );
		}
		if( gpsConfig.debugmodeRAW ){
			Serial.print( "roll toRightRoll toRight SetToRight SetForew," );
			Serial.print( roll ); Serial.print( "," );
			Serial.print( toRight ); Serial.print( "," );
		}

		toRight = toRight - gpsConfig.virtAntRight;
		if( gpsConfig.debugmodeRAW ){
			Serial.print( toRight ); Serial.print( "," );
			Serial.print( gpsConfig.virtAntRight ); Serial.print( "," );
			Serial.print( gpsConfig.virtAntForew ); Serial.print( "," );
		}
	}
	else {
		toRight = 0 - gpsConfig.virtAntRight;
		if( gpsConfig.debugmodeRAW ){ Serial.print( ",,,,,," ); }
	}

	if( gpsConfig.virtAntForew != 0 ){
		if( toRight != 0 ){
			//shift Antenna foreward and to the right: direction to move point: heading-atan( foreward/right )
			virtAntHeadingDiff = ( atan2( toRight, gpsConfig.virtAntForew )) / PI180;
			//distance to move  cm  / mean radius of earth cm ( WGS84 6.371.000,8m )
			double distAnt = sqrt(( toRight * toRight ) + ( gpsConfig.virtAntForew * gpsConfig.virtAntForew ));
			WayByRadius = distAnt / 637100080;
			if( gpsConfig.debugmodeRAW ){
				Serial.print( "distToMove SetForew virtAntHeadDiff," );
				Serial.print( distAnt,4 ); Serial.print( "," );
				Serial.print( gpsConfig.virtAntForew ); Serial.print( "," );
				Serial.print( virtAntHeadingDiff ); Serial.print( "," );
				Serial.print( "WayByRadius," );
				Serial.print( WayByRadius,10 ); Serial.print( "," );
			}
			if( gpsConfig.debugmodeVirtAnt ){
				Serial.print( "virt Ant: real dist to move" ); Serial.print( distAnt );
				Serial.print( " WayByRadius: " ); Serial.println( WayByRadius, 10 );
				Serial.print( " angle corr by: " ); Serial.println( virtAntHeadingDiff, 2 );
			}
		}
		else {
			//shift Antenna only foreward
			virtAntHeadingDiff = 0.0;
			//distance to move  cm  / mean radius of earth cm ( WGS84 6.371.000,8m )
			WayByRadius = gpsConfig.virtAntForew / 637100080;

		}
	}
	else {
		if( toRight != 0 ){
			//shift Antenna only right ( needed for roll compensation )
			virtAntHeadingDiff = 90.0;
			WayByRadius = toRight / 637100080;
			if( gpsConfig.debugmodeVirtAnt ){
				Serial.print( "effective dist to move virt Ant: " ); Serial.print( toRight );
				Serial.print( " WayToRaduis: " ); Serial.println( WayByRadius, 12 );
			}
		}
	}//direction+way



	if(( virtLatRad != 0.0 ) && ( virtLonRad != 0.0 )){
		//all calculations in radians:  decimal * PI / 180
		headingRad = HeadingMix + virtAntHeadingDiff;
		if( headingRad > 360 ){ headingRad -= 360; }
		else {
			if( headingRad < 0 ){ headingRad += 360; }
		}
		headingRad *= PI180;
		
		virtLatRadTemp = asin(( sin( virtLatRad ) * cos( WayByRadius )) + ( cos( virtLatRad ) * sin( WayByRadius ) * cos( headingRad )));
		
		//may solve E/W Greenwich problem
		//if( virtLon > 0 ){
			virtLonRad = virtLonRad + atan2(( sin( headingRad ) * sin( WayByRadius ) * cos( virtLatRad )), ( cos( WayByRadius ) - ( sin( virtLatRad ) * sin( virtLatRadTemp ))));
		//}
		//else{
		//	virtLonRad = virtLonRad - atan2(( sin( HeadingRad ) * sin( WayByRadius ) * cos( virtLatRad )), ( cos( WayByRadius ) - ( sin( virtLatRad )* sin( virtLatRadTemp ))));
		//}
		virtLatRad = virtLatRadTemp;
	
		//radians to dec
		virtLat = virtLatRad / PI180;
		virtLon = virtLonRad / PI180;

		virtAntPosPresent = true;
		if( gpsConfig.debugmodeRAW ){
			Serial.print( "virtLat virtLon," );
			Serial.print( virtLat,7 ); Serial.print( "," );
			Serial.print( virtLon,7 ); Serial.print( "," );
		}
		if( gpsConfig.debugmodeVirtAnt ){
			Serial.println( "Virtual Antenna point calc:" );
			Serial.print( "UBX1 Lat: " ); Serial.print(( double( UBXPVT1[UBXRingCount1].lat )/ 10000000 ), 7 ); Serial.print( " virtLat: " ); Serial.print( virtLat, 7 );
			Serial.print( "UBX1 Lon: " ); Serial.print(( double( UBXPVT1[UBXRingCount1].lon )/ 10000000 ), 7 ); Serial.print( " virtLon: " ); Serial.println( virtLon, 7 );
			Serial.print( "virt Point head: " ); Serial.print(( headingRad ), 3 ); Serial.print( " GPS mix head: " ); Serial.print( HeadingMix, 3 );
			Serial.print( " roll: " ); Serial.println( roll, 3 );
		}
	}
	else { if( gpsConfig.debugmode||gpsConfig.debugmodeVirtAnt ){ Serial.println( "No virtual Antenna point: lat/lon = 0" ); } }
}

//-------------------------------------------------------------------------------------------------


void filterPosition(){
	//input to the kalman filter
	if( virtAntPosPresent ){ latK = virtLat; lonK = virtLon; }
	else { latK = double( UBXPVT1[UBXRingCount1].lat  ) * 0.0000001; lonK = double( UBXPVT1[UBXRingCount1].lon  ) * 0.0000001; }

	if( gpsConfig.debugmodeHeading || gpsConfig.debugmodeFilterPos ){ Serial.print( " lat: " ); Serial.print( latK, 7 ); }

	//Kalman filter
	latPc = latP + latVarProcess;
	latG = latPc / ( latPc + latVar );
	latP = ( 1 - latG ) * latPc;
	latXp = latXe;
	latZp = latXp;
	latXe = ( latG * ( latK - latZp )) + latXp;

	//Kalman filter
	lonPc = lonP + lonVarProcess;
	lonG = lonPc / ( lonPc + lonVar );
	lonP = ( 1 - lonG ) * lonPc;
	lonXp = lonXe;
	lonZp = lonXp;
	lonXe = ( lonG * ( lonK - lonZp )) + lonXp;

	if( gpsConfig.debugmodeRAW ){
		Serial.print( "virtLat virtLon filtVirtLat FiltVirtLog," );
		Serial.print( virtLat,8 ); Serial.print( "," );
		Serial.print( virtLon,8 ); Serial.print( "," );
		Serial.print( latXe,8 ); Serial.print( "," );
		Serial.print( lonXe,8 ); Serial.print( "," );
	}

	virtLon = lonXe;//result
	virtLat = latXe;//result
	if( gpsConfig.debugmodeHeading || gpsConfig.debugmodeFilterPos ){
		Serial.print( " lat filtered: " ); Serial.print( virtLat, 7 );
		Serial.print( " lon: " ); Serial.print( lonK, 7 ); Serial.print( " lon filtered: " ); Serial.println( virtLon, 7 );
	}
}

void headingAndPosition ( void* z ){
  constexpr TickType_t xFrequency = 1;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for( ;;  ){
		
		//getUBX();//read serials

		if( UBXRingCount1 != OGIfromUBX )//new UBX exists
		{
			headingRollCalc();
			if( existsUBXRelPosNED ){
				//virtual Antenna point?
				if(( gpsConfig.virtAntForew != 0 ) || ( gpsConfig.virtAntRight != 0 ) ||
					(( gpsConfig.GPSPosCorrByRoll == 1 ) && ( gpsConfig.AntHeight > 0 )))
				{//all data there
					virtualAntennaPoint();
				}
			}
			else //only 1 Antenna
			{
				virtAntPosPresent = false;
				if(( gpsConfig.debugmodeHeading ) || ( gpsConfig.debugmodeVirtAnt )){ Serial.println( "no dual Antenna values so not virtual Antenna point calc" ); }
			}

			//filter position: set kalman variables
			//0: no fix 1: GPS only -> filter slow, else filter fast, but filter due to no roll compensation
			if( UBXPVT1[UBXRingCount1].fixType <= 1 ){ latVarProcess = VarProcessSlow; lonVarProcess = VarProcessSlow; filterGPSpos = true; }
			else { if( !dualGPSHeadingPresent ){ latVarProcess = VarProcessFast; lonVarProcess = VarProcessFast; filterGPSpos = true; } }
					//filterGPSPosition might set false an Kalman variables set, if signal is perfect ( in void HeadingRollCalc )
					
					if( gpsConfig.filterGPSposOnWeakSignal == 0 ){ filterGPSpos = false; }
			
					filterPosition();//runs always to fill kalman variables

			buildGGA();
			buildVTG();
			buildHDT();
			buildOGI();
			buildRMC();

			if( gpsConfig.sendOGI == 1 ){
				udpRoof.writeTo( OGIBuffer, OGIdigit, ipDestination, gpsConfig.aogPortSendTo );
				if( gpsConfig.debugmodeRAW ){
					Serial.print( "millis," ); Serial.print( millis()); Serial.print( "," );
					Serial.print( "UBXRingCount1 OGIfromUBX PAOGI," );
					Serial.print( UBXRingCount1 ); Serial.print( "," );
					Serial.print( OGIfromUBX ); Serial.print( "," );
					Serial.print( "DualGPSPres RollPres VirtAntPres DrivDir FilterPos," );
					Serial.print( dualGPSHeadingPresent ); Serial.print( "," );
					Serial.print( rollPresent ); Serial.print( "," );
					Serial.print( virtAntPosPresent ); Serial.print( "," );
					Serial.print( drivDirect ); Serial.print( "," );
					Serial.print( filterGPSpos ); Serial.print( "," );
					Serial.print( "PVThead RelPosNEDhead," );
					Serial.print( UBXPVT1[UBXRingCount1].headMot ); Serial.print( "," );
					Serial.print( UBXRelPosNED[UBXRingCount2].relPosHeading ); Serial.print( "," );
					Serial.print( "PVTlat PVTlon," );
					Serial.print( UBXPVT1[UBXRingCount1].lat ); Serial.print( "," );
					Serial.print( UBXPVT1[UBXRingCount1].lon ); Serial.print( "," );
					for ( byte N = 0; N < OGIdigit; N++ ){Serial.write( OGIBuffer[N]);}
				}
				newOGI = false;
			}
			if( gpsConfig.sendGGA ){
				udpRoof.writeTo( GGABuffer, GGAdigit, ipDestination, gpsConfig.aogPortSendTo );
				newGGA = false;
			}
			if( gpsConfig.sendVTG ){
				udpRoof.writeTo( VTGBuffer, VTGdigit, ipDestination, gpsConfig.aogPortSendTo );
				newVTG = false;
			}
			if( gpsConfig.sendHDT ){
				udpRoof.writeTo( HDTBuffer, HDTdigit, ipDestination, gpsConfig.aogPortSendTo );
				newHDT = false;
			}
			if( gpsConfig.sendSerialNmeaHDT ){
				queueSerialNmea( HDTQueue, HDTBuffer, HDTdigit );
			}
			if( gpsConfig.sendSerialNmeaVTG ){
				queueSerialNmea( VTGQueue, VTGBuffer, VTGdigit );
			}
			if( gpsConfig.sendSerialNmeaGGA ){
				queueSerialNmea( GGAQueue, GGABuffer, GGAdigit );
			}
			if( gpsConfig.sendSerialNmeaRMC ){
				queueSerialNmea( RMCQueue, RMCBuffer, RMCdigit );
			}
			gpsHzCurrentMillis = millis() - previousMillisHz;
			previousMillisHz = millis();
			gpsHzMaxMillis = max( gpsHzCurrentMillis, gpsHzMaxMillis );
			gpsHzMinMillis = min( gpsHzCurrentMillis, gpsHzMinMillis );
		}
		if( digitalRead( gpsConfig.gpioDcPowerGood ) == LOW ){
				powerUnstableMillis = millis();
				powerUnstable = true;
			}
			else if( ( millis() - powerUnstableMillis ) / 60000 > 5){
				powerUnstable = false;
		}
		if( ++udpHelloCounter >= 25 ){ //2.5s
			udpHelloCounter = 0;
			uint8_t helloFromGPS[] = { 128, 129, 120, 120, 5, 0, 0, 0, 0, 0, 71 };
			udpRoof.writeTo( helloFromGPS, sizeof( helloFromGPS ), ipDestination, gpsConfig.aogPortSendTo );
		}
		if( millis() - lastHelloReceivedMillis > 5000 ){
			ipDestination = IPAddress( 192, 168, 5, 255 );
		}

		vTaskDelayUntil( &xLastWakeTime, xFrequency );
	}
}

void initHeadingAndPosition( ){
  xTaskCreate( headingAndPosition, "headingAndPosition", 3096, NULL, 5, NULL );
}
