#include "panda.h"

void makePANDAfromPVT(UBX_NAV_PVT_data_t *pvt, char *panda, float yaw, float pitch, float roll)
{
	uint8_t latDegrees = pvt->lat / 10000000;
	float latMinutes = (((float)pvt->lat / 10000000.0) - (float)latDegrees) * 60.0;

	uint8_t lonDegrees = pvt->lon / 10000000;
	float lonMinutes = (((float)pvt->lon / 10000000.0) - (float)lonDegrees) * 60.0;

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

	sprintf(panda, "$PANDA,%02u%02u%02u.%02u,%02u%2.7f,%c,%03u%3.7f,%c,%u,%u,%.1f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f*",
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
			yaw,
			roll,
			pitch,
			0.0);

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

	sprintf(panda + strlen(panda), "%X\r\n", sum);
}

/*void makePANDA(SFE_UBLOX_GNSS *GNSS, char *panda)
{
  uint8_t latDegrees = GNSS->getLatitude() / 10000000;
  float latMinutes = (((float)GNSS->getLatitude() / 10000000.0) - (float)latDegrees) * 60.0;

  uint8_t lonDegrees = GNSS->getLongitude() / 10000000;
  float lonMinutes = (((float)GNSS->getLongitude() / 10000000.0) - (float)lonDegrees) * 60.0;

  char lonLetter = (GNSS->getLongitude() > 0) ? 'E' : 'W';
  char latLetter = (GNSS->getLatitude() > 0) ? 'N' : 'S';

  uint8_t fixType = 0;
  if (GNSS->getGnssFixOk())
  {
    fixType = 1;
  }
  if (GNSS->getDiffSoln())
  {
    fixType = 2;
  }
  if (GNSS->getCarrierSolutionType() == 1)
  {
    fixType = 5;
  }
  if (GNSS->getCarrierSolutionType() == 2)
  {
    fixType = 4;
  }

  snprintf(panda, 100, "$PANDA,%02u%02u%02u.%02u,%02u%2.7f,%c,%03u%3.7f,%c,%u,%u,%.1f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%u*",
          GNSS->getHour(),
          GNSS->getMinute(),
          GNSS->getSecond(),
          (uint8_t)((GNSS->getTimeOfWeek() % 1000) / 10),
          latDegrees,
          latMinutes,
          latLetter,
          lonDegrees,
          lonMinutes,
          lonLetter,
          fixType,
          GNSS->getSIV(),
          (float)GNSS->getPDOP() * 0.01,
          (float)GNSS->getAltitudeMSL() / 1000.0,
          0.0,
          (float)GNSS->getGroundSpeed() * 0.00194384,
          145.2,
          2.4,
          0.1,
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

  snprintf(panda + strlen(panda), 100, "%X\r\n", sum);
}*/