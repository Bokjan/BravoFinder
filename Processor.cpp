#include "global.h"
#include "RawDataReader.h"
void ProcessRawData(void)
{
	RawDataReader *Reader = new RawDataReader;
	Reader->ReadWaypoints();
	Reader->Output();
	delete Reader;
}