//$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M ,  ,*47
//   0     1      2      3    4      5 6  7  8   9    10 11  12 13  14
//        Time      Lat       Lon

/*
GGA - essential fix data which provide 3D location and accuracy data.

$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47

Where:
GGA          Global Positioning System Fix Data
123519       Fix taken at 12:35:19 UTC
4807.038,N   Latitude 48 deg 07.038' N
01131.000,E  Longitude 11 deg 31.000' E
1            Fix quality: 0 = invalid
1 = GPS fix (SPS)
2 = DGPS fix
3 = PPS fix
4 = Real Time Kinematic
5 = Float RTK
6 = estimated (dead reckoning) (2.3 feature)
7 = Manual input mode
8 = Simulation mode
(7)    08           Number of satellites being tracked
(8)    0.9          Horizontal dilution of position
545.4,M      Altitude, Meters, above mean sea level
46.9,M       Height of geoid (mean sea level) above WGS84
ellipsoid
(empty field) time in seconds since last DGPS update
(empty field) DGPS station ID number
*47          the checksum data, always begins with *
*
*

$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48
*
VTG          Track made good and ground speed
054.7,T      True track made good (degrees)
034.4,M      Magnetic track made good
005.5,N      Ground speed, knots
010.2,K      Ground speed, Kilometers per hour
*48          Checksum
*
$PAOGI
* * From GGA:
				
				$PAOGI
				(1) 123519 Fix taken at 1219 UTC
				Roll corrected position
				(2,3) 4807.038,N Latitude 48 deg 07.038' N
				(4,5) 01131.000,E Longitude 11 deg 31.000' E
				(6) 1 Fix quality:
					0 = invalid
					1 = GPS fix(SPS)
					2 = DGPS fix
					3 = PPS fix
					4 = Real Time Kinematic
					5 = Float RTK
					6 = estimated(dead reckoning)(2.3 feature)
					7 = Manual input mode
					8 = Simulation mode
				(7) Number of satellites being tracked
				(8) 0.9 Horizontal dilution of position
				(9) 545.4 Altitude (ALWAYS in Meters, above mean sea level)
				(10) 1.2 time in seconds since last DGPS update
				(11) 022.4 Speed over the ground in knots - can be positive or negative
				FROM AHRS:
				(12) Heading in degrees
				(13) Roll angle in degrees(positive roll = right leaning - right down, left up)
				(14) Pitch angle in degrees(Positive pitch = nose up)
				(15) Yaw Rate in Degrees / second
				* CHKSUM
*/


//----------------------------------------------------------------------------------------------------------------------------------------------------

#include "main.hpp"

//NMEA
uint8_t OGIBuffer[sizeOfOGIBuffer], HDTBuffer[sizeOfHDTBuffer], VTGBuffer[sizeOfVTGBuffer], GGABuffer[sizeOfGGABuffer], RMCBuffer[sizeOfRMCBuffer];
bool newOGI = false, newHDT = false, newGGA = false, newVTG = false, newRMC = false;
uint8_t OGIdigit = 0, GGAdigit = 0, VTGdigit = 0, HDTdigit = 0, RMCdigit = 0;

//append ddmm.mmmmmmm (degDigits 2) or dddmm.mmmmmmm (degDigits 3) from a count of 1e-7 minutes, sign written by the caller
static void appendDegMin( uint8_t* buffer, uint8_t& digit, int64_t minE7, uint8_t degDigits ) {
	if (minE7 < 0) { minE7 = -minE7; }
	long deg = minE7 / 600000000;//60 min * 10^7
	long min = minE7 % 600000000;
	if (degDigits == 3) { buffer[digit++] = (deg / 100) + 48; deg = deg % 100; }
	buffer[digit++] = (deg / 10) + 48;
	buffer[digit++] = (deg % 10) + 48;
	buffer[digit++] = (min / 100000000) + 48;
	min = min % 100000000;
	buffer[digit++] = (min / 10000000) + 48;
	min = min % 10000000;
	buffer[digit++] = 0x2E;//.
	for (long div = 1000000; div > 0; div /= 10) {
		buffer[digit++] = (min / div) + 48;
		min = min % div;
	}
}

//append [-]ddd.dd (intDigits 3) or [-]dd.dd (intDigits 2) from a count of hundredths, rounded by the caller
static void appendHundredths( uint8_t* buffer, uint8_t& digit, long v, uint8_t intDigits ) {
	if (v < 0) { buffer[digit++] = 0x2D; v = -v; }//-
	if (intDigits == 3) { buffer[digit++] = (v / 10000) + 48; v = v % 10000; }
	buffer[digit++] = (v / 1000) + 48;
	v = v % 1000;
	buffer[digit++] = (v / 100) + 48;
	v = v % 100;
	buffer[digit++] = 0x2E;//.
	buffer[digit++] = (v / 10) + 48;
	buffer[digit++] = (v % 10) + 48;
}

//DOP from a count of hundredths as d.dd or dd.dd, no leading zero
static void appendDOP( uint8_t* buffer, uint8_t& digit, uint16_t v ) {
	if (v > 9999) { v = 9999; }
	if (v >= 1000) { buffer[digit++] = (v / 1000) + 48; }
	buffer[digit++] = ((v / 100) % 10) + 48;
	buffer[digit++] = 0x2E;//.
	buffer[digit++] = ((v / 10) % 10) + 48;
	buffer[digit++] = (v % 10) + 48;
}

//HDOP in hundredths from the F9P's GGA; without a GGA on UART1 NAV-PVT pDOP, which is >= HDOP
static uint16_t hdopE2() {
	return (hdopGGA >= 0) ? hdopGGA : epochPVT.pDOP;
}

//HeadingMix in hundredths of a degree, 359.996 rounds to 36000 and is sent as 000.00
static long headingE2() {
	long v = lround(HeadingMix * 100);
	if (v >= 36000) { v -= 36000; }
	return v;
}

void buildOGI() {
	byte CRC = 0;
	long templng;

	//create $PAOGI
	OGIBuffer[0] = 0x24;//$
	OGIBuffer[1] = 0x50;//P
	OGIBuffer[2] = 0x41;//A
	OGIBuffer[3] = 0x4F;//O
	OGIBuffer[4] = 0x47;//G
	OGIBuffer[5] = 0x49;//I
	OGIBuffer[6] = 0x2C;//,

	OGIdigit = 7;

	//always +48 to get ASCII: "0" = 48

	// time stamp hhmmss
	byte time = epochPVT.hour;
	OGIBuffer[OGIdigit++] = (time / 10) + 48;
	time = time % 10;
	OGIBuffer[OGIdigit++] = time + 48;
	time = epochPVT.min;
	OGIBuffer[OGIdigit++] = (time / 10) + 48;
	time = time % 10;
	OGIBuffer[OGIdigit++] = time + 48;
	time = epochPVT.sec;
	OGIBuffer[OGIdigit++] = (time / 10) + 48;
	time = time % 10;
	OGIBuffer[OGIdigit++] = time + 48;
	OGIBuffer[OGIdigit++] = 0x2E;//.
	templng = epochPVT.nano / 1000000;//ms
	OGIBuffer[OGIdigit++] = (templng / 100) + 48;
	templng = templng % 100;
	OGIBuffer[OGIdigit++] = (templng / 10) + 48;

	OGIBuffer[OGIdigit++] = 0x2C;//,

	//lat lon in 1e-7 minutes: NAV-PVT 1e-7 deg * 60 is exact, the filtered / virtual antenna double is rounded once
	int64_t latMin, lonMin;
	if (filterGPSpos) {//weak signal: Kalman filtered
		latMin = llround(latXe * 600000000.0);
		lonMin = llround(lonXe * 600000000.0);
	}
	else if (virtAntPosPresent) {
		latMin = llround(virtLat * 600000000.0);
		lonMin = llround(virtLon * 600000000.0);
	}
	else {
		latMin = int64_t(epochPVT.lat) * 60;
		lonMin = int64_t(epochPVT.lon) * 60;
	}
	if (gpsConfig.debugmodeRAW) { Serial.print("buildOGI latMin lonMin,"); Serial.print(latMin); Serial.print(","); Serial.print(lonMin); Serial.print(","); }

	//lat: ddmm.mmmmmmm
	appendDegMin(OGIBuffer, OGIdigit, latMin, 2);
	OGIBuffer[OGIdigit++] = 0x2C;//,
	//N/S
	OGIBuffer[OGIdigit++] = (latMin > 0) ? 0x4E : 0x53;//N : S
	OGIBuffer[OGIdigit++] = 0x2C;//,

	//lon: dddmm.mmmmmmm
	appendDegMin(OGIBuffer, OGIdigit, lonMin, 3);
	OGIBuffer[OGIdigit++] = 0x2C;//,
	//E/W
	OGIBuffer[OGIdigit++] = (lonMin < 0) ? 0x57 : 0x45;//W : E
	OGIBuffer[OGIdigit++] = 0x2C;//,

	//fix type
	if (bGGAexists) { OGIBuffer[OGIdigit++] = cFixQualGGA; }// code by ai, 07.10.2020: use the GGA Message to determine Fix-Quality	
	else {
		if ((bitRead(epochPVT.flags, 1) == true) && (epochPVT.fixType == 3)) { OGIBuffer[OGIdigit++] = 52; }//4 = RTK
		else { OGIBuffer[OGIdigit++] = epochPVT.fixType + 48; }
	}
	OGIBuffer[OGIdigit++] = 0x2C;//,

	byte temp = epochPVT.numSV;//number of satellites
	OGIBuffer[OGIdigit++] = (temp / 10) + 48;
	temp = temp % 10;
	OGIBuffer[OGIdigit++] = temp + 48;
	OGIBuffer[OGIdigit++] = 0x2C;//,

	//HDOP
	appendDOP(OGIBuffer, OGIdigit, hdopE2());
	OGIBuffer[OGIdigit++] = 0x2C;//,

	//height main sea level mm
	if (epochPVT.hMSL < 0) {//below sea level, so write "-"
		OGIBuffer[OGIdigit++] = 0x2D;
		templng = abs(epochPVT.hMSL);
	}
	else templng = epochPVT.hMSL;
	OGIBuffer[OGIdigit++] = (templng / 1000000) + 48;
	templng = templng % 1000000;
	OGIBuffer[OGIdigit++] = (templng / 100000) + 48;
	templng = templng % 100000;
	OGIBuffer[OGIdigit++] = (templng / 10000) + 48;
	templng = templng % 10000;
	OGIBuffer[OGIdigit++] = (templng / 1000) + 48;
	templng = templng % 1000;
	OGIBuffer[OGIdigit++] = 0x2E;//.
	OGIBuffer[OGIdigit++] = (templng / 100) + 48;
	templng = templng % 100;
	OGIBuffer[OGIdigit++] = (templng / 10) + 48;
	OGIBuffer[OGIdigit++] = 0x2C;//,

    //shoud be age of NTRIP, not included in UBX PVT 
	OGIBuffer[OGIdigit++] = 0x2C;//,

	//ground speed knots
	if (drivDirect == 2) {//backwards, so write "-"
		OGIBuffer[OGIdigit++] = 0x2D;}
	if (epochPVT.gSpeed > 30) { templng = (1.9438445 * epochPVT.gSpeed); }
	else templng = 0;//send 0 if slower than 0,1km/h
	OGIBuffer[OGIdigit++] = (templng / 10000) + 48;
	templng = templng % 10000;
	OGIBuffer[OGIdigit++] = (templng / 1000)+48;
	templng = templng % 1000;
	OGIBuffer[OGIdigit++] = 0x2E;//.
	OGIBuffer[OGIdigit++] = (templng / 100)+48;
	templng = templng %100;
	OGIBuffer[OGIdigit++] = (templng / 10)+48;
	templng = templng %10;
	OGIBuffer[OGIdigit++] = templng +48;

	OGIBuffer[OGIdigit++] = 0x2C;//,


	//GPS dual heading ddd.dd
	appendHundredths(OGIBuffer, OGIdigit, headingE2(), 3);
	OGIBuffer[OGIdigit++] = 0x2C;//,

	//roll [-]dd.dd, sign from the rounded value
	appendHundredths(OGIBuffer, OGIdigit, lround((0 - rollToAOG) * 100), 2);
	OGIBuffer[OGIdigit++] = 0x2C;//,

	//pitch
	OGIBuffer[OGIdigit++] = 0x2C;//,
	//yaw
	OGIBuffer[OGIdigit++] = 0x2C;//,

	OGIBuffer[OGIdigit++] = 0x2A;//*

	//checksum
	for (byte index = 1; index <= (OGIdigit); index++) {
		if (OGIBuffer[index] == 0x2A) {
			break;
		}
		CRC = CRC ^ OGIBuffer[index];
	}
	OGIBuffer[OGIdigit++] = byte(CRC / 16) + 48;
	if (OGIBuffer[OGIdigit - 1] > 57) { OGIBuffer[OGIdigit-1] += 7; }//ASCII 48 = 0 ASCII A = 65
	OGIBuffer[OGIdigit++] = byte(CRC % 16) + 48;
	if (OGIBuffer[OGIdigit - 1] > 57) { OGIBuffer[OGIdigit - 1] += 7; }//ASCII 48 = 0 ASCII A = 65
	OGIBuffer[OGIdigit++] = 0x0D;
	OGIBuffer[OGIdigit++] = 0x0A;	
	
	
	newOGI = true;
}

//------------------------------------------------------------------------------------------------------------------------------------------------------


void buildHDT() {
	byte CRC = 0;

	HDTBuffer[0] = 0x24;//$
	HDTBuffer[1] = 0x47;//G
	HDTBuffer[2] = 0x4E;//N
	HDTBuffer[3] = 0x48;//H
	HDTBuffer[4] = 0x44;//D
	HDTBuffer[5] = 0x54;//T
	HDTBuffer[6] = 0x2C;//,

	HDTdigit = 7;

	//heading ddd.dd
	appendHundredths(HDTBuffer, HDTdigit, headingE2(), 3);

	HDTBuffer[HDTdigit++] = 0x2C;//,
	HDTBuffer[HDTdigit++] = 0x54;//T
	HDTBuffer[HDTdigit++] = 0x2A;//*

	//checksum
	for (byte index = 1; index <= (HDTdigit); index++) {
		if (HDTBuffer[index] == 0x2A) {
			break;
		}
		CRC = CRC ^ HDTBuffer[index];
	}
	HDTBuffer[HDTdigit++] = byte(CRC / 16) + 48;
	if (HDTBuffer[HDTdigit - 1] > 57) { HDTBuffer[HDTdigit - 1] += 7; }//ASCII 48 = 0 ASCII A = 65
	HDTBuffer[HDTdigit++] = byte(CRC % 16) + 48;
	if (HDTBuffer[HDTdigit - 1] > 57) { HDTBuffer[HDTdigit - 1] += 7; }//ASCII 48 = 0 ASCII A = 65
	HDTBuffer[HDTdigit++] = 0x0D;
	HDTBuffer[HDTdigit++] = 0x0A;

	newHDT = true;
}


//----------------------------------------------------------------------------------------------------------------------------------------------------



void buildGGA() {
	byte CRC = 0;

	//create $GPGGA
	GGABuffer[0] = 0x24;//$
	GGABuffer[1] = 0x47;//G
	GGABuffer[2] = 0x50;//P
	GGABuffer[3] = 0x47;//G
	GGABuffer[4] = 0x47;//G
	GGABuffer[5] = 0x41;//A
	GGABuffer[6] = 0x2C;//,

	GGAdigit = 7;

	//always +48 to get ASCII: "0" = 48

	// time stamp hhmmss
	byte time = epochPVT.hour;
	GGABuffer[GGAdigit++] = (time / 10) + 48;
	time = time % 10;
	GGABuffer[GGAdigit++] = time + 48;
	time = epochPVT.min;
	GGABuffer[GGAdigit++] = (time / 10) + 48;
	time = time % 10;
	GGABuffer[GGAdigit++] = time + 48;
	time = epochPVT.sec;
	GGABuffer[GGAdigit++] = (time / 10) + 48;
	time = time % 10;
	GGABuffer[GGAdigit++] = time + 48;
	GGABuffer[GGAdigit++] = 0x2E;//.
	long timel = epochPVT.nano / 1000000;//ms
	GGABuffer[GGAdigit++] = (timel / 100) + 48;
	timel = timel % 100;
	GGABuffer[GGAdigit++] = (timel / 10) + 48;

	GGABuffer[GGAdigit++] = 0x2C;//,

	//lat: ddmm.mmmmmmm, NAV-PVT 1e-7 deg * 60 = 1e-7 min exact
	appendDegMin(GGABuffer, GGAdigit, int64_t(epochPVT.lat) * 60, 2);
	GGABuffer[GGAdigit++] = 0x2C;//,
	//N/S
	GGABuffer[GGAdigit++] = (epochPVT.lat > 0) ? 0x4E : 0x53;//N : S
	GGABuffer[GGAdigit++] = 0x2C;//,

	//lon: dddmm.mmmmmmm
	appendDegMin(GGABuffer, GGAdigit, int64_t(epochPVT.lon) * 60, 3);
	GGABuffer[GGAdigit++] = 0x2C;//,
	//E/W
	GGABuffer[GGAdigit++] = (epochPVT.lon < 0) ? 0x57 : 0x45;//W : E
	GGABuffer[GGAdigit++] = 0x2C;//,

	//fix type
	if (bGGAexists) { GGABuffer[GGAdigit++] = cFixQualGGA; }// code by ai, 07.10.2020: use the GGA Message to determine Fix-Quality	
	else {
		if ((bitRead(epochPVT.flags, 1) == true) && (epochPVT.fixType == 3)) { GGABuffer[GGAdigit++] = 52; }//4 = RTK
		else { GGABuffer[GGAdigit++] = epochPVT.fixType + 48; }
	}
	GGABuffer[GGAdigit++] = 0x2C;//,

	byte temp = epochPVT.numSV;//number of satellites
	GGABuffer[GGAdigit++] = (temp / 10) + 48;
	temp = temp % 10;
	GGABuffer[GGAdigit++] = temp + 48;
	GGABuffer[GGAdigit++] = 0x2C;//,

	//HDOP
	appendDOP(GGABuffer, GGAdigit, hdopE2());
	GGABuffer[GGAdigit++] = 0x2C;//,

	long templng = epochPVT.hMSL;//height main sea level mm
	GGABuffer[GGAdigit++] = (templng / 1000000) + 48;
	templng = templng % 1000000;
	GGABuffer[GGAdigit++] = (templng / 100000) + 48;
	templng = templng % 100000;
	GGABuffer[GGAdigit++] = (templng / 10000) + 48;
	templng = templng % 10000;
	GGABuffer[GGAdigit++] = (templng / 1000) + 48;
	templng = templng % 1000;
	GGABuffer[GGAdigit++] = 0x2E;//.
	GGABuffer[GGAdigit++] = (templng / 100) + 48;
	templng = templng % 100;
	GGABuffer[GGAdigit++] = (templng / 10) + 48;
	GGABuffer[GGAdigit++] = 0x2C;//,
	GGABuffer[GGAdigit++] = 0x4D;//M
	GGABuffer[GGAdigit++] = 0x2C;//,
	//no height above geoid
	GGABuffer[GGAdigit++] = 0x2C;//,	
	//so no M
	GGABuffer[GGAdigit++] = 0x2C;//,
	//shoud be age of NTRIP, not included in UBX PVT 
	GGABuffer[GGAdigit++] = 0x2C;//,
	//no station numer


	GGABuffer[GGAdigit++] = 0x2A;//*

	//checksum
	for (byte index = 1; index <= (GGAdigit); index++) {
		if (GGABuffer[index] == 0x2A) {
			break;
		}
		CRC = CRC ^ GGABuffer[index];
	}
	GGABuffer[GGAdigit++] = byte(CRC / 16) + 48;
	if (GGABuffer[GGAdigit - 1] > 57) { GGABuffer[GGAdigit - 1] += 7; }//ASCII 48 = 0 ASCII A = 65
	GGABuffer[GGAdigit++] = byte(CRC % 16) + 48;
	if (GGABuffer[GGAdigit - 1] > 57) { GGABuffer[GGAdigit - 1] += 7; }//ASCII 48 = 0 ASCII A = 65
	GGABuffer[GGAdigit++] = 0x0D;
	GGABuffer[GGAdigit++] = 0x0A;

	newGGA = true;

	/*
		if (debugmode) {
			Serial.println("build OGI from PVT:");
			for (byte n = 0; n < GGAdigit; n++) {
				Serial.print(char(GGABuffer[n]));
			}
			Serial.println();
		}
		*/
}


//----------------------------------------------------------------------------------------------------------------------------------------------------


void buildVTG() {
	byte CRC = 0;

	//create $GPVTG
	VTGBuffer[0] = 0x24;//$
	VTGBuffer[1] = 0x47;//G
	VTGBuffer[2] = 0x50;//P
	VTGBuffer[3] = 0x56;//V
	VTGBuffer[4] = 0x54;//T
	VTGBuffer[5] = 0x47;//G
	VTGBuffer[6] = 0x2C;//,

	VTGdigit = 7;

	//always +48 to get ASCII: "0" = 48
	//heading ddd.dd, HeadingMix decided in Heading calc
	appendHundredths(VTGBuffer, VTGdigit, headingE2(), 3);
	VTGBuffer[VTGdigit++] = 0x2C;//,
	VTGBuffer[VTGdigit++] = 0x54;//T
	VTGBuffer[VTGdigit++] = 0x2C;//,
	//no magnetic heading
	VTGBuffer[VTGdigit++] = 0x2C;//,
	//no M
	VTGBuffer[VTGdigit++] = 0x2C;//,

	//ground speed knots
	if (drivDirect == 2) {//backwards, so write "-"
		VTGBuffer[VTGdigit++] = 0x2D;
	}
	long speed1000Knotes;
	if (epochPVT.gSpeed > 30) { speed1000Knotes = (1.9438445 * epochPVT.gSpeed); }
	else speed1000Knotes = 0;//send 0 if slower than 0,1km/h
	VTGBuffer[VTGdigit++] = (speed1000Knotes / 10000) + 48;
	speed1000Knotes = speed1000Knotes % 10000;
	VTGBuffer[VTGdigit++] = (speed1000Knotes / 1000) + 48;
	speed1000Knotes = speed1000Knotes % 1000;
	VTGBuffer[VTGdigit++] = 0x2E;//.
	VTGBuffer[VTGdigit++] = (speed1000Knotes / 100) + 48;
	speed1000Knotes = speed1000Knotes % 100;
	VTGBuffer[VTGdigit++] = (speed1000Knotes / 10) + 48;
	speed1000Knotes = speed1000Knotes % 10;
	VTGBuffer[VTGdigit++] = speed1000Knotes + 48;
	VTGBuffer[VTGdigit++] = 0x2C;//,
	VTGBuffer[VTGdigit++] = 0x4E;//N
	VTGBuffer[VTGdigit++] = 0x2C;//,

	//ground speed km/h
	long speed1000kmh = (3.6 * epochPVT.gSpeed);
	VTGBuffer[VTGdigit++] = (speed1000kmh / 10000) + 48;
	speed1000kmh = speed1000kmh % 10000;
	VTGBuffer[VTGdigit++] = (speed1000kmh / 1000) + 48;
	speed1000kmh = speed1000kmh % 1000;
	VTGBuffer[VTGdigit++] = 0x2E;//.
	VTGBuffer[VTGdigit++] = (speed1000kmh / 100) + 48;
	speed1000kmh = speed1000kmh % 100;
	VTGBuffer[VTGdigit++] = (speed1000kmh / 10) + 48;
	speed1000kmh = speed1000kmh % 10;
	VTGBuffer[VTGdigit++] = speed1000kmh + 48;

	VTGBuffer[VTGdigit++] = 0x2C;//,

	//Kilometers
	VTGBuffer[VTGdigit++] = 0x4B;//K

	VTGBuffer[VTGdigit++] = 0x2C;//,

	//Mode
	VTGBuffer[VTGdigit++] = 0x44;//D

	VTGBuffer[VTGdigit++] = 0x2A;//*

	//checksum
	for (byte index = 1; index <= (VTGdigit); index++) {
		if (VTGBuffer[index] == 0x2A) {
			break;
		}
		CRC = CRC ^ VTGBuffer[index];
	}
	VTGBuffer[VTGdigit++] = byte(CRC / 16) + 48;
	if (VTGBuffer[VTGdigit - 1] > 57) { VTGBuffer[VTGdigit - 1] += 7; }//ASCII 48 = 0 ASCII A = 65
	VTGBuffer[VTGdigit++] = byte(CRC % 16) + 48;
	if (VTGBuffer[VTGdigit - 1] > 57) { VTGBuffer[VTGdigit - 1] += 7; }//ASCII 48 = 0 ASCII A = 65
	VTGBuffer[VTGdigit++] = 0x0D;
	VTGBuffer[VTGdigit++] = 0x0A;

	newVTG = true;
}

void buildRMC() {
	byte CRC = 0;

	//create $GPRMC
	RMCBuffer[0] = 0x24;//$
	RMCBuffer[1] = 0x47;//G
	RMCBuffer[2] = 0x50;//P
	RMCBuffer[3] = 0x52;//R
	RMCBuffer[4] = 0x4D;//M
	RMCBuffer[5] = 0x43;//C
	RMCBuffer[6] = 0x2C;//,

	RMCdigit = 7;

	// always +48 to get ASCII: "0" = 48
	// time stamp hhmmss
	byte time = epochPVT.hour;
	RMCBuffer[RMCdigit++] = (time / 10) + 48;
	time = time % 10;
	RMCBuffer[RMCdigit++] = time + 48;
	time = epochPVT.min;
	RMCBuffer[RMCdigit++] = (time / 10) + 48;
	time = time % 10;
	RMCBuffer[RMCdigit++] = time + 48;
	time = epochPVT.sec;
	RMCBuffer[RMCdigit++] = (time / 10) + 48;
	time = time % 10;
	RMCBuffer[RMCdigit++] = time + 48;
	RMCBuffer[RMCdigit++] = 0x2E;//.
	long timel = epochPVT.nano / 1000000;//ms
	RMCBuffer[RMCdigit++] = (timel / 100) + 48;
	timel = timel % 100;
	RMCBuffer[RMCdigit++] = (timel / 10) + 48;

	RMCBuffer[RMCdigit++] = 0x2C;//,

	//Status: A/V
	RMCBuffer[RMCdigit++] = 0x41; //FIXME

	RMCBuffer[RMCdigit++] = 0x2C;//,

	//lat: ddmm.mmmmmmm, NAV-PVT 1e-7 deg * 60 = 1e-7 min exact
	appendDegMin(RMCBuffer, RMCdigit, int64_t(epochPVT.lat) * 60, 2);
	RMCBuffer[RMCdigit++] = 0x2C;//,
	//N/S
	RMCBuffer[RMCdigit++] = (epochPVT.lat > 0) ? 0x4E : 0x53;//N : S
	RMCBuffer[RMCdigit++] = 0x2C;//,

	//lon: dddmm.mmmmmmm
	appendDegMin(RMCBuffer, RMCdigit, int64_t(epochPVT.lon) * 60, 3);
	RMCBuffer[RMCdigit++] = 0x2C;//,
	//E/W
	RMCBuffer[RMCdigit++] = (epochPVT.lon < 0) ? 0x57 : 0x45;//W : E
	RMCBuffer[RMCdigit++] = 0x2C;//,

	//ground speed knots
	if (drivDirect == 2) {//backwards, so write "-"
		RMCBuffer[RMCdigit++] = 0x2D;
	}
	long speed1000Knotes;
	if (epochPVT.gSpeed > 30) { speed1000Knotes = (1.9438445 * epochPVT.gSpeed); }
	else speed1000Knotes = 0;//send 0 if slower than 0,1km/h
	RMCBuffer[RMCdigit++] = (speed1000Knotes / 10000) + 48;
	speed1000Knotes = speed1000Knotes % 10000;
	RMCBuffer[RMCdigit++] = (speed1000Knotes / 1000) + 48;
	speed1000Knotes = speed1000Knotes % 1000;
	RMCBuffer[RMCdigit++] = 0x2E;//.
	RMCBuffer[RMCdigit++] = (speed1000Knotes / 100) + 48;
	speed1000Knotes = speed1000Knotes % 100;
	RMCBuffer[RMCdigit++] = (speed1000Knotes / 10) + 48;
	speed1000Knotes = speed1000Knotes % 10;
	RMCBuffer[RMCdigit++] = speed1000Knotes + 48;
	RMCBuffer[RMCdigit++] = 0x2C;//,

	//course over ground ddd.dd, HeadingMix decided in Heading calc
	appendHundredths(RMCBuffer, RMCdigit, headingE2(), 3);
	RMCBuffer[RMCdigit++] = 0x2C;//,

	//Date
	uint8_t date = epochPVT.day;
	RMCBuffer[RMCdigit++] = ( date / 10 ) + 48;
	date = date % 10;
	RMCBuffer[RMCdigit++] = date + 48;
	date = epochPVT.month;
	RMCBuffer[RMCdigit++] = ( date / 10 ) + 48;
	date = date % 10;
	RMCBuffer[RMCdigit++] = date + 48;
	date = epochPVT.year % 100;
	RMCBuffer[RMCdigit++] = ( date / 10 ) + 48;
	date = date % 10;
	RMCBuffer[RMCdigit++] = date + 48;
	RMCBuffer[RMCdigit++] = 0x2C;//,

	//no magnetic variation
	RMCBuffer[RMCdigit++] = 0x2C;//,

	//Mode
	RMCBuffer[RMCdigit++] = 0x44;//D

	RMCBuffer[RMCdigit++] = 0x2A;//*

	//checksum
	for (byte index = 1; index <= (RMCdigit); index++) {
		if (RMCBuffer[index] == 0x2A) {
			break;
		}
		CRC = CRC ^ RMCBuffer[index];
	}
	RMCBuffer[RMCdigit++] = byte(CRC / 16) + 48;
	if (RMCBuffer[RMCdigit - 1] > 57) { RMCBuffer[RMCdigit - 1] += 7; }//ASCII 48 = 0 ASCII A = 65
	RMCBuffer[RMCdigit++] = byte(CRC % 16) + 48;
	if (RMCBuffer[RMCdigit - 1] > 57) { RMCBuffer[RMCdigit - 1] += 7; }//ASCII 48 = 0 ASCII A = 65
	RMCBuffer[RMCdigit++] = 0x0D;
	RMCBuffer[RMCdigit++] = 0x0A;

	newRMC = true;
}
