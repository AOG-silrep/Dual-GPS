
#include <stdio.h>
#include "main.hpp"

uint64_t NavPvtCount;
time_t RelPosNedMillis;
time_t NavPvtMillis;
uint64_t RelPosNedCount;
time_t previousRelPosNedMillis;
time_t previousNavPvtMillis;
time_t GGATimestampMillis;
time_t previousGGATimestampMillis;
time_t GGAReceivedMillis;

void getSerialUBX( void* z ) {
	bool checksumOk1 = false;
	byte incomByte1 = 0, idx1 = 0;
	word CRC1 = 0;
	static unsigned char UBXchecksum1[2] = { 0,0 };
	byte nextUBXcount1 = 0;

  constexpr TickType_t xFrequency = 1;
  TickType_t xLastWakeTime = xTaskGetTickCount();

	for( ;; ){
		//serial1 = main if only one
		while (Serial1.available())
		{
			nextUBXcount1 = (UBXRingCount1 + 1) % sizeOfUBXArray;

			incomByte1 = Serial1.read();
			//if (gpsConfig.debugmodeUBX) { Serial.print("incom Byte: "); Serial.print(incomByte1); Serial.print("UBXRingCount1: "); Serial.print(UBXRingCount1); Serial.print(" nextUBXCount1: "); Serial.println(nextUBXcount1); }

			// ai, 07.10.2020: use the GGA Message to determine Fix-Quality
			if (incomByte1 == '$') {
				bNMEAstarted = true;
				sNMEA = "";
			}
			if (bNMEAstarted == true) {
				sNMEA.concat((char)incomByte1); // add the char to the NMEA message
				if (incomByte1 == 10) { // ASCII(10) <LF> (Linefeed) ends the message
					bNMEAstarted = false;
					if (sNMEA.substring(3, 6) == "GGA") { // GGA Message found
						// Parse the HHMMSS.ss timestamp from field 1 to measure F9P message interval
						int firstComma = sNMEA.indexOf(',');
						if (firstComma > 0 && firstComma + 9 < (int)sNMEA.length()) {
							int t = firstComma + 1;
							if (sNMEA[t] >= '0' && sNMEA[t] <= '2') { // sanity check on hour
								uint8_t hh = (sNMEA[t]   - '0') * 10 + (sNMEA[t+1] - '0');
								uint8_t mm = (sNMEA[t+2] - '0') * 10 + (sNMEA[t+3] - '0');
								uint8_t ss = (sNMEA[t+4] - '0') * 10 + (sNMEA[t+5] - '0');
								// fractional: .cc = centiseconds, multiply by 10 to get ms
								uint16_t frac = 0;
								if (sNMEA[t+6] == '.') {
									frac = (sNMEA[t+7] - '0') * 100 + (sNMEA[t+8] - '0') * 10;
								}
								uint32_t ts = ((uint32_t)hh * 3600 + mm * 60 + ss) * 1000 + frac;
								previousGGATimestampMillis = GGATimestampMillis;
								GGATimestampMillis = ts;
							}
						}
						GGAReceivedMillis = millis();

						// the fix quality is the char after the sixth ',' - so look for sixth ','
						iPos = -1;
						i = 0;
						do {
							iPos = sNMEA.indexOf(',', iPos + 1); // find position of next ','
							if (iPos > -1) { i++; }
						} while ((i < 6) && (iPos > -1));
						if (iPos > -1) { cFixQualGGA = sNMEA.charAt(iPos + 1); bGGAexists = true; }
					}
				}
			}
			// END ai, 07.10.2020: use the GGA Message to determine Fix-Quality


			//UBX comming?
			if (UBXDigit1 < 2) {
				if (incomByte1 == UBX_HEADER[UBXDigit1]) {
					UBXDigit1++;
					//if (gpsConfig.debugmodeUBX) { Serial.print("UBX1 started: digit  "); Serial.println(UBXDigit1 - 1); }
				}
				else
					UBXDigit1 = 0;
			}
			else {
				//add incoming Byte to UBX
				((unsigned char*)(&UBXPVT1[nextUBXcount1]))[UBXDigit1 - 2] = incomByte1;
				UBXDigit1++;
				//if (gpsConfig.debugmodeUBX) { Serial.print(incomByte1); Serial.print(": incoming byte number: "); Serial.println(UBXDigit1); }

				if (UBXDigit1 == 5) {
					/*if (debugmode) {
						Serial.print("UBX1 wrong sentence: digit  ");
						Serial.print(UBXDigit1 - 1);
						Serial.print("cls  ");
						Serial.print(UBXPVT1[nextUBXcount1].cls);
						Serial.print("id  ");
						Serial.println(UBXPVT1[nextUBXcount1].id);
					}*/
					if (!((UBXPVT1[nextUBXcount1].cls == 0x01) && (UBXPVT1[nextUBXcount1].id == 0x07))) {
						//wrong sentence
						UBXDigit1 = 0;
					}
					//else { if (debugmode) { Serial.println("UBX PVT1 found"); } }
				}//5
				else
				{
					if (UBXDigit1 == 7) {//length
						UBXLength1 = UBXPVT1[nextUBXcount1].len + 8;//+2xheader,2xclass,2xlength,2xchecksum
						//if (debugmode) { Serial.print("UBXLength1: "); Serial.println(UBXLength1); }
					}
					else
					{
						if (UBXDigit1 == UBXLength1) { //UBX complete
							UBXchecksum1[0] = 0;
							UBXchecksum1[1] = 0;
							for (int i = 0; i < (UBXLength1 - 4); i++) {
								UBXchecksum1[0] += ((unsigned char*)(&UBXPVT1[nextUBXcount1]))[i];
								UBXchecksum1[1] += UBXchecksum1[0];
							}
							/*if (debugmode) {
								Serial.print("UBX Checksum0 expected "); Serial.print(UBXchecksum1[0]);
								Serial.print("  incomming Checksum0: "); Serial.println(UBXPVT1[nextUBXcount1].CK0);
								Serial.print("UBX Checksum1 expected "); Serial.print(UBXchecksum1[1]);
								Serial.print("  incomming Checksum1: "); Serial.println(UBXPVT1[nextUBXcount1].CK1);
							}
							*/
							if ((UBXPVT1[nextUBXcount1].CK0 == UBXchecksum1[0]) && (UBXPVT1[nextUBXcount1].CK1 == UBXchecksum1[1])) {
								UBXDigit1 = 0;

								if (gpsConfig.debugmodeRAW) {
									Serial.print("SerIn: #lstUBX #newUBX newUBX lat lon"); Serial.print(",");
									Serial.print(UBXRingCount1); Serial.print(",");
									Serial.print(nextUBXcount1); Serial.print(",");
									Serial.print(UBXPVT1[nextUBXcount1].lat); Serial.print(",");
									Serial.print(UBXPVT1[nextUBXcount1].lon); Serial.print(",");
									//Serial.print("SerIn: RelPosNED heading down dist flags"); Serial.print(",");
									//Serial.print(UBXRelPosNED[UBXRingCount2].relPosHeading); Serial.print(",");
									//Serial.print(UBXRelPosNED[UBXRingCount2].relPosD); Serial.print("."); Serial.print(UBXRelPosNED[nextUBXcount2].relPosHPD); Serial.print(",");
									//Serial.print(UBXRelPosNED[UBXRingCount2].relPosLength); Serial.print(",");
									//Serial.print(UBXRelPosNED[UBXRingCount2].flags); Serial.print(",");
								}

								UBXRingCount1 = nextUBXcount1;
								UBXLength1 = 100;
								if (gpsConfig.debugmodeUBX) {
									Serial.print("got UBX1 PVT lat: "); Serial.print(UBXPVT1[nextUBXcount1].lat);
									Serial.print(" lon: "); Serial.println(UBXPVT1[nextUBXcount1].lon);
								}
								previousNavPvtMillis = NavPvtMillis;
								NavPvtMillis = millis();
								NavPvtCount ++;
							}
							else {
								if (gpsConfig.debugmodeUBX) { Serial.println("UBX1 PVT checksum invalid"); }
								diagnostics.badChecksumNavPVTCount += 1;
								saveDiagnostics();
							}
						}//UBX complete
						else
						{//too long
							//Serial.print(UBXRelPosNED[UBXRingCount2].len); Serial.print(" ");
							//Serial.print(UBXRelPosNED[UBXRingCount2].res1); Serial.print(" ");
							//Serial.print(UBXRelPosNED[UBXRingCount2].refStID); Serial.print(" ");
							//Serial.print(UBXRelPosNED[UBXRingCount2].iTOW); Serial.print(" ");
							//Serial.print(UBXRelPosNED[UBXRingCount2].res2); Serial.print(" ");
							//Serial.print(UBXRelPosNED[UBXRingCount2].res3); Serial.print(" ");
							//Serial.print(UBXRelPosNED[UBXRingCount2].field); Serial.print(" ");
							//Serial.print(UBXRelPosNED[UBXRingCount2].field); Serial.print(" ");
							//Serial.print(UBXRelPosNED[UBXRingCount2].field); Serial.print(" ");
							//Serial.print(UBXRelPosNED[UBXRingCount2].field); Serial.print(" ");
							//Serial.print(UBXRelPosNED[UBXRingCount2].field); Serial.print(" ");
							//Serial.print(UBXRelPosNED[UBXRingCount2].field); Serial.print(" ");
							//Serial.println(" RelPosNED");
							if (UBXDigit1 > (108)) {
								UBXDigit1 = 0;
								UBXLength1 = 100;
								diagnostics.wrongLengthNavPVTCount += 1;
								saveDiagnostics();
							}
						}//UBX complete
					}//7
				}//5
			}//>2
		}//while serial 1 available

		vTaskDelayUntil( &xLastWakeTime, xFrequency );
	}
}//void getUBX

void getSerial2UBX ( void* z ){
	bool checksumOk2 = false;
	byte incomByte2 = 0, idx2 = 0;
	word CRC2 = 0;
	static unsigned char UBXchecksum2[2] = { 0,0 };
	byte nextUBXcount2 = 0;

  constexpr TickType_t xFrequency = 1;
  TickType_t xLastWakeTime = xTaskGetTickCount();

	for( ;; ){
		//dual GPS: serial 2
		while (Serial2.available())
		{
			nextUBXcount2 = (UBXRingCount2 + 1) % sizeOfUBXArray;
			incomByte2 = Serial2.read();
			//if (gpsConfig.debugmodeUBX) { Serial.print("incom Byte2: "); Serial.print(incomByte2); }

			//UBX comming?
			if (UBXDigit2 < 2) {
				if (incomByte2 == UBX_HEADER[UBXDigit2]) {
					UBXDigit2++;
					//if (gpsConfig.debugmodeUBX) { Serial.print("UBX2 RelPosNED started: digit  "); Serial.println(UBXDigit2 - 1); }
				}
				else
					UBXDigit2 = 0;
			}
			else {
				//add incoming Byte to UBX
				((unsigned char*)(&UBXRelPosNED[nextUBXcount2]))[UBXDigit2 - 2] = incomByte2;
				UBXDigit2++;
				//if (gpsConfig.debugmodeUBX) { Serial.print(incomByte2); Serial.print(": incoming byte number: "); Serial.println(UBXDigit2); }

				if (UBXDigit2 == 5) {

					if (!((UBXRelPosNED[nextUBXcount2].cls == 0x01) && (UBXRelPosNED[nextUBXcount2].id == 0x3C))) {
						//wrong sentence
						/*if (gpsConfig.debugmodeUBX) {
						Serial.print("UBX2 wrong sentence: digit  ");
						Serial.print(UBXDigit2 - 1);
						Serial.print("cls  ");
						Serial.print(UBXRelPosNED[nextUBXcount2].cls, HEX);
						Serial.print("id  ");
						Serial.println(UBXRelPosNED[nextUBXcount2].id, HEX);
					}*/
						UBXDigit2 = 0;
					}
					else { if (gpsConfig.debugmodeUBX) { Serial.println("UBX RelPosNED found"); } }
				}//5
				else
				{
					if (UBXDigit2 == 7) {//length
						UBXLength2 = UBXRelPosNED[nextUBXcount2].len + 8;//+2xheader,2xclass,2xlength,2xchecksum
						//if (debugmode) { Serial.print("UBXLength2: "); Serial.println(UBXLength2); }
					}
					else
					{
						if (UBXDigit2 == UBXLength2) { //UBX complete
							UBXchecksum2[0] = 0;
							UBXchecksum2[1] = 0;
							for (int i = 0; i < (UBXLength2 - 4); i++) {
								UBXchecksum2[0] += ((unsigned char*)(&UBXRelPosNED[nextUBXcount2]))[i];
								UBXchecksum2[1] += UBXchecksum2[0];
							}
							/*	if (gpsConfig.debugmodeUBX) {
									Serial.print("UBX2 Checksum0 expected "); Serial.print(UBXchecksum2[0]);
									Serial.print("  incomming Checksum0: "); Serial.println(UBXRelPosNED[nextUBXcount2].CK0);
									Serial.print("UBX2 Checksum1 expected "); Serial.print(UBXchecksum2[1]);
									Serial.print("  incomming Checksum1: "); Serial.println(UBXRelPosNED[nextUBXcount2].CK1);
								}
								*/
							if ((UBXRelPosNED[nextUBXcount2].CK0 == UBXchecksum2[0]) && (UBXRelPosNED[nextUBXcount2].CK1 == UBXchecksum2[1])) {
								//checksum OK
								if (gpsConfig.checkUBXFlags) {
									if (bitRead(UBXRelPosNED[nextUBXcount2].flags, 8)) {
										//flag: heading OK
										existsUBXRelPosNED = true;
										UBXRingCount2 = nextUBXcount2;
										if (gpsConfig.debugmodeUBX) {
											Serial.print("got RelPosNED. Heading: "); Serial.print((UBXRelPosNED[nextUBXcount2].relPosHeading * 0.00001), 2);
											Serial.print(" down vector (cm): "); Serial.println((float(UBXRelPosNED[nextUBXcount2].relPosD) + (float(UBXRelPosNED[nextUBXcount2].relPosHPD) * 0.01)), 2);
										}
										if (gpsConfig.debugmodeRAW) {
											Serial.print("SerIn: RelPosNED heading down dist flags"); Serial.print(",");
											Serial.print(UBXRelPosNED[UBXRingCount2].relPosHeading); Serial.print(",");
											Serial.print((float(UBXRelPosNED[nextUBXcount2].relPosD) + (float(UBXRelPosNED[nextUBXcount2].relPosHPD) * 0.01)), 2); Serial.print(",");
											Serial.print(UBXRelPosNED[UBXRingCount2].relPosLength); Serial.print(",");
											Serial.print(UBXRelPosNED[UBXRingCount2].flags); Serial.print(",");
										}
									}
									else { if (gpsConfig.debugmodeUBX) { Serial.println("UBXRelPosNED flag heading: NOT valid, checksum OK; sentence NOT used"); } }
								}
								else {//don't check UBX flags, checksum ok
									existsUBXRelPosNED = true;
									UBXRingCount2 = nextUBXcount2;
									if (gpsConfig.debugmodeUBX) {
										Serial.print("got RelPosNED. Heading: "); Serial.print((UBXRelPosNED[nextUBXcount2].relPosHeading * 0.00001), 2);
										Serial.print(" down vector (cm): "); Serial.println((float(UBXRelPosNED[nextUBXcount2].relPosD) + (float(UBXRelPosNED[nextUBXcount2].relPosHPD) * 0.01)), 2);
									}
									if (gpsConfig.debugmodeRAW) {
										Serial.print("SerIn: RelPosNED heading down dist flags"); Serial.print(",");
										Serial.print(UBXRelPosNED[UBXRingCount2].relPosHeading); Serial.print(",");
										Serial.print(UBXRelPosNED[UBXRingCount2].relPosD); Serial.print("."); Serial.print(UBXRelPosNED[nextUBXcount2].relPosHPD); Serial.print(",");
										Serial.print(UBXRelPosNED[UBXRingCount2].relPosLength); Serial.print(",");
										Serial.print(UBXRelPosNED[UBXRingCount2].flags); Serial.print(",");
									}
								}
								previousRelPosNedMillis = RelPosNedMillis;
								RelPosNedMillis = millis();
								RelPosNedCount ++;
								UBXDigit2 = 0;
								UBXLength2 = 100;
							}
							else { //checksum wrong
								UBXDigit2 = 0;
								UBXLength2 = 100;
								if (gpsConfig.debugmodeUBX) { Serial.println("UBX2 RelPosNED checksum invalid"); }
								diagnostics.badChecksumRelPosNEDCount += 1;
								saveDiagnostics();
							}
						}//UBX complete
						else
						{//too long
							if (UBXDigit2 > (108)) {
								UBXDigit2 = 0;
								UBXLength2 = 100;
								diagnostics.wrongLengthRelPosNEDCount += 1;
								saveDiagnostics();
							}
						}//UBX complete
					}//7
				}//5
			}//>2
		}//while serial 2 available

		vTaskDelayUntil( &xLastWakeTime, xFrequency );
	}
}

void initSerialUbxReceivers( ){
  xTaskCreate( getSerialUBX, "getSerialUBX", 3096, NULL, 5, NULL );
  xTaskCreate( getSerial2UBX, "getSerial2UBX", 3096, NULL, 5, NULL );
}