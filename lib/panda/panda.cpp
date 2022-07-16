#include "panda.h"

void makePANDAfromPVT(UBX_NAV_PVT_data_t *pvt, char *panda, float yaw, float roll, float pitch)
{
	uint8_t latDegrees = pvt->lat / 10000000;
	double latMinutes = (((double)pvt->lat / 10000000.0) - (double)latDegrees) * 60.0;

	uint8_t lonDegrees = pvt->lon / 10000000;
	double lonMinutes = (((double)pvt->lon / 10000000.0) - (double)lonDegrees) * 60.0;

	char lonLetter = (pvt->lon > 0) ? 'E' : 'W';
	char latLetter = (pvt->lat > 0) ? 'N' : 'S';

	uint8_t fixType = 0;
	if (pvt->flags.bits.gnssFixOK)
	{
		fixType = 1;
	}
	if (pvt->flags.bits.diffSoln)
	{
		fixType = 2;
	}
	if (pvt->flags.bits.carrSoln == 1)
	{
		fixType = 5;
	}
	if (pvt->flags.bits.carrSoln == 2)
	{
		fixType = 4;
	}

	snprintf(panda, 196, "$PANDA,%02u%02u%02u.%02u,%02u%2.8f,%c,%03u%3.8f,%c,%u,%u,%.1f,%.2f,%.1f,%.1f,%04u,%02i,%02i,%02u*",
			pvt->hour,
			pvt->min,
			pvt->sec,
			(uint8_t)((pvt->iTOW % 1000) / 10),
			latDegrees,
			latMinutes,
			latLetter,
			lonDegrees,
			lonMinutes,
			lonLetter,
			fixType,
			pvt->numSV,
			(float)pvt->pDOP * 0.01,
			(float)pvt->hMSL / 1000.0,
			0.0,
			(float)pvt->gSpeed * 0.00194384,
			(uint16_t)(yaw * 10),
			(int16_t)(roll * 10),
			(int16_t)(pitch * 10),
			0);

	int16_t sum = 0, inx;
	char tmp;

	// The checksum calc starts after '$' and ends before '*'
	for (inx = 1; inx < 200; inx++)
	{
		tmp = panda[inx];
		// * Indicates end of data and start of checksum
		if (tmp == '*')
			break;
		sum ^= tmp; // Build checksum
	}

	sprintf(panda + strlen(panda), "%02X\r\n", sum);
}