#include <fstream>
#include "global.h"
#include "DataStructure.h"
class RawDataReader{
	int vertexIdent;
	//std::map<string, std::vector<Waypoint> > Waypoints;
	string GetDataPath(void)
	{
		return string("F:\\NavigationData\\NavData\\");
	}
	string GetOutPath(void)
	{
		return string("F:\\NavigationData\\NavData\\Output\\");
	}
public:
	RawDataReader(void)
	{
		vertexIdent = 0;
		printf("Reading data from %s\n", GetDataPath().c_str());
	}
	void ReadWaypoints(void);
	void ReadSids(void);
	void Output(void);
};