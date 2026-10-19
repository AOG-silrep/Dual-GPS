
#include <stdio.h>
#include "main.hpp"

uint64_t NavPvtCount;
uint32_t InvalidNavPvtCount;
uint32_t JumpNavPvtCount;
uint32_t HeadingInvalidRelPosNedCount;
uint32_t NotFixedRelPosNedCount;
uint32_t ExtrapolatedRelPosNedCount;
uint32_t HeadingRangeRelPosNedCount;
uint32_t HeadingInaccurateRelPosNedCount;
time_t RelPosNedMillis;
time_t NavPvtMillis;
uint64_t RelPosNedCount;
time_t previousRelPosNedMillis;
time_t previousNavPvtMillis;
time_t GGATimestampMillis;
time_t previousGGATimestampMillis;
time_t GGAReceivedMillis;

//NAV-PVT plausibility check, so a bad position never becomes UBXRingCount1
constexpr int32_t nullIslandBox = 10000000;     //1° around 0,0 is open ocean, deg * 10^-7
constexpr double maxPlausibleSpeed = 30.0;      //m/s ( ~108 km/h )
constexpr double jumpMargin = 10.0;             //m, room for fix type changes
constexpr uint8_t maxJumpRejects = 10;          //after this many jumps in a row accept the new position

//NAV-RELPOSNED check, so a heading that can't be used never becomes UBXRingCount2
constexpr int32_t maxRelPosHeading = 36000000;  //360°, deg * 10^-5
constexpr uint32_t maxAccHeading = 200000;      //2°, deg * 10^-5; fixed on a 1-2 m baseline is ~0.1-0.5°

static bool hasLastGoodPVT = false;
static int32_t lastGoodLat = 0, lastGoodLon = 0;
static uint32_t lastGoodITOW = 0;
static uint8_t jumpRejects = 0;
static double jumpDistance = 0;

static bool isPositionValid( const NAV_PVT& pvt ) {
	if (pvt.fixType == 0 || pvt.fixType == 5) { return false; }//no fix or time only
	if (gpsConfig.checkUBXFlags && !bitRead(pvt.flags, 0)) { return false; }//gnssFixOK
	if (bitRead(pvt.flags3, 0)) { return false; }//invalidLlh
	if (pvt.lat > 900000000 || pvt.lat < -900000000 || pvt.lon > 1800000000 || pvt.lon < -1800000000) { return false; }
	if (abs(pvt.lat) < nullIslandBox && abs(pvt.lon) < nullIslandBox) { return false; }
	return true;
}

static bool isPositionJump( const NAV_PVT& pvt ) {
	if (!hasLastGoodPVT || jumpRejects >= maxJumpRejects) { return false; }
	int64_t dtMs = (int64_t)pvt.iTOW - lastGoodITOW;
	if (dtMs < 0) { dtMs += msPerGpsWeek; }
	double dLat = (double(pvt.lat) - lastGoodLat) * 0.0000001;
	double dLon = (double(pvt.lon) - lastGoodLon) * 0.0000001;
	if (dLon > 180) { dLon -= 360; }
	else if (dLon < -180) { dLon += 360; }
	dLon *= cos(lastGoodLat * 0.0000001 * PI180);
	jumpDistance = hypot(dLat, dLon) * 111195.0;//m per degree
	return jumpDistance > maxPlausibleSpeed * dtMs * 0.001 + jumpMargin;
}

//counts the first check that fails
static bool isHeadingValid( const NAV_RELPOSNED& relPos ) {
	if (gpsConfig.checkUBXFlags) {
		if (!bitRead(relPos.flags, 8)) { HeadingInvalidRelPosNedCount++; return false; }//relPosHeadingValid
		if (((relPos.flags >> 3) & 3) != 2) { NotFixedRelPosNedCount++; return false; }//carrSoln: 2 = fixed ambiguities
		if (bitRead(relPos.flags, 6) || bitRead(relPos.flags, 7)) { ExtrapolatedRelPosNedCount++; return false; }//refPosMiss, refObsMiss
	}
	if (relPos.relPosHeading < 0 || relPos.relPosHeading > maxRelPosHeading) { HeadingRangeRelPosNedCount++; return false; }
	if (relPos.accHeading > maxAccHeading) { HeadingInaccurateRelPosNedCount++; return false; }
	return true;
}

//NMEA number d[d].dd at p in hundredths, -1 if it is empty or not followed by ','
static int16_t parseHundredths( const String& s, int p ) {
	const int len = s.length();
	long v = 0;
	uint8_t intDigits = 0, decDigits = 0;
	while (p < len && isDigit(s[p])) {
		if (++intDigits > 3) { return -1; }
		v = v * 10 + (s[p++] - '0');
	}
	v *= 100;
	if (p < len && s[p] == '.') {
		p++;
		while (p < len && isDigit(s[p])) {
			if (decDigits == 0) { v += (s[p] - '0') * 10; }
			else if (decDigits == 1) { v += s[p] - '0'; }
			decDigits++;
			p++;
		}
	}
	if (intDigits == 0 || p >= len || s[p] != ',') { return -1; }
	return (v > 9999) ? 9999 : v;
}

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

						//HDOP is two ',' further ( 7 = numSV, 8 = HDOP ), an empty field keeps the last one
						if (iPos > -1) { iPos = sNMEA.indexOf(',', iPos + 1); }
						if (iPos > -1) { iPos = sNMEA.indexOf(',', iPos + 1); }
						if (iPos > -1) {
							int16_t hdop = parseHundredths(sNMEA, iPos + 1);
							if (hdop >= 0) { hdopGGA = hdop; }
						}
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
						if (UBXPVT1[nextUBXcount1].len != navPvtPayloadLength) {
							UBXDigit1 = 0;
							UBXLength1 = 100;
							if (gpsConfig.debugmodeUBX) { Serial.print("UBX1 PVT wrong length: "); Serial.println(UBXPVT1[nextUBXcount1].len); }
							diagnostics.wrongLengthNavPVTCount += 1;
							saveDiagnostics();
						}
						else {
							UBXLength1 = UBXPVT1[nextUBXcount1].len + 8;//+2xheader,2xclass,2xlength,2xchecksum
						}
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

								if (!isPositionValid(UBXPVT1[nextUBXcount1])) {
									InvalidNavPvtCount++;
									if (gpsConfig.debugmodeUBX) {
										Serial.print("UBX1 PVT position invalid, checksum OK; sentence NOT used. fixType: "); Serial.print(UBXPVT1[nextUBXcount1].fixType);
										Serial.print(" lat: "); Serial.print(UBXPVT1[nextUBXcount1].lat);
										Serial.print(" lon: "); Serial.println(UBXPVT1[nextUBXcount1].lon);
									}
								}
								else if (isPositionJump(UBXPVT1[nextUBXcount1])) {
									JumpNavPvtCount++;
									jumpRejects++;
									if (gpsConfig.debugmodeUBX) {
										Serial.print("UBX1 PVT position jump, checksum OK; sentence NOT used. distance (m): "); Serial.println(jumpDistance, 1);
									}
								}
								else {
									jumpRejects = 0;
									lastGoodLat = UBXPVT1[nextUBXcount1].lat;
									lastGoodLon = UBXPVT1[nextUBXcount1].lon;
									lastGoodITOW = UBXPVT1[nextUBXcount1].iTOW;
									hasLastGoodPVT = true;
									UBXRingCount1 = nextUBXcount1;
									if (gpsConfig.debugmodeUBX) {
										Serial.print("got UBX1 PVT lat: "); Serial.print(UBXPVT1[nextUBXcount1].lat);
										Serial.print(" lon: "); Serial.println(UBXPVT1[nextUBXcount1].lon);
									}
								}
								UBXLength1 = 100;
								previousNavPvtMillis = NavPvtMillis;
								NavPvtMillis = millis();
								NavPvtCount ++;
							}
							else {
								UBXDigit1 = 0;
								UBXLength1 = 100;
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
							//backstop, the length is checked at 7: the next byte goes to UBXDigit1 - 2, past the slot
							if (UBXDigit1 >= sizeof(NAV_PVT) + 2) {
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
					if (UBXDigit2 == 7) {//length, version is the byte just written
						if (UBXRelPosNED[nextUBXcount2].len != relPosNedPayloadLength || UBXRelPosNED[nextUBXcount2].ver != relPosNedVersion) {
							UBXDigit2 = 0;
							UBXLength2 = 100;
							if (gpsConfig.debugmodeUBX) {
								Serial.print("UBX2 RelPosNED wrong length: "); Serial.print(UBXRelPosNED[nextUBXcount2].len);
								Serial.print(" version: "); Serial.println(UBXRelPosNED[nextUBXcount2].ver);
							}
							diagnostics.wrongLengthRelPosNEDCount += 1;
							saveDiagnostics();
						}
						else {
							UBXLength2 = UBXRelPosNED[nextUBXcount2].len + 8;//+2xheader,2xclass,2xlength,2xchecksum
						}
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
								if (isHeadingValid(UBXRelPosNED[nextUBXcount2])) {
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
								else if (gpsConfig.debugmodeUBX) {
									Serial.print("UBX2 RelPosNED heading not used, checksum OK; flags: "); Serial.print(UBXRelPosNED[nextUBXcount2].flags, HEX);
									Serial.print(" carrSoln: "); Serial.print((UBXRelPosNED[nextUBXcount2].flags >> 3) & 3);
									Serial.print(" heading: "); Serial.print((UBXRelPosNED[nextUBXcount2].relPosHeading * 0.00001), 2);
									Serial.print(" accHeading: "); Serial.println((UBXRelPosNED[nextUBXcount2].accHeading * 0.00001), 2);
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
							//backstop, the length is checked at 7: the next byte goes to UBXDigit2 - 2, past the slot
							if (UBXDigit2 >= sizeof(NAV_RELPOSNED) + 2) {
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
  //above async_tcp (10) and the web UI, so a UI burst can't hold the UBX streams back and make RelPosNED stale
  xTaskCreate( getSerialUBX, "getSerialUBX", 3096, NULL, 12, NULL );
  xTaskCreate( getSerial2UBX, "getSerial2UBX", 3096, NULL, 12, NULL );
}