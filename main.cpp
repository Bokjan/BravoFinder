#include "global.h"
#include "DataStructure.h"
#include "Processor.h"
std::map<string, Airport> Airports;
std::map<string, std::vector<Waypoint> > Waypoints;
std::vector<Vertex> g;
int main(void)
{
	ProcessRawData();
	return 0;
}