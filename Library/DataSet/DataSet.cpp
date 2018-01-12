#include <set>
#include <cstdio>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include "DataSet.hpp"
#include "../DataStructure/Graph.hpp"

using std::string;

namespace bf
{
	class InternalStruct
	{
	public:
		std::set<string> initializedAirports;
	};
}

void bf::DataSet::SetDataPath(const std::string &s)
{
	path = s;
}

bf::DataSet::DataSet(void)
{
	graph = new Graph;
	is = new InternalStruct;
}

bf::DataSet::~DataSet(void)
{
	delete is;
	delete graph;
}

void bf::DataSet::InitializeAirports(void)
{
	puts("iap");
	static bool done = false;
	if (done)
		return;
	done = true;
	std::ifstream ifs(path + "/NAVDATA/airports.dat");
	if (!ifs.is_open())
		throw std::runtime_error("Failed to open file: NAVDATA/airports.dat");
	constexpr int bufferSize = 128;
	char buffer[bufferSize];
	while (ifs.getline(buffer, bufferSize))
	{
		if (buffer[0] == ';')
			continue;
		string icao(buffer, 4);
		string latitude(buffer + 4, 10);
		string longitude(buffer + 15, 10);
		graph->graphHelper->AddVertex(icao, (float) atof(latitude.c_str()), (float) atof(longitude.c_str()));
	}
	ifs.close();
}

void bf::DataSet::InitializeFixes(void)
{
	puts("if");
	static bool done = false;
	if (done)
		return;
	done = true;
	std::ifstream ifs(path + "/NAVDATA/wpNavRTE.txt");
	if (!ifs.is_open())
		throw std::runtime_error("Failed to open file: NAVDATA/wpNavRTE.txt");
	constexpr int bufferSize = 128;
	char buffer[bufferSize];
	string ident;
	float latitude, longitude;
	while (ifs.getline(buffer, bufferSize))
	{
		if (buffer[0] == ';')
			continue;
		std::istringstream iss(buffer);
		iss >> ident >> ident >> ident >> latitude >> longitude;
		graph->graphHelper->AddVertex(ident, latitude, longitude);
		break;
	}
	while (ifs >> ident >> ident >> ident >> latitude >> longitude)
		graph->graphHelper->AddVertex(ident, latitude, longitude);
	ifs.close();
}

void bf::DataSet::InitializeRoutes(void)
{
	puts("ir");
	static bool done = false;
	if (done)
		return;
	done = true;
	std::ifstream ifs(path + "/NAVDATA/wpNavRTE.txt");
	if (!ifs.is_open())
		throw std::runtime_error("Failed to open file: NAVDATA/wpNavRTE.txt");
	graph->graphHelper->AddRouteString("DCT");
	graph->graphHelper->AddRouteString("SID");
	graph->graphHelper->AddRouteString("STAR");
	string lastRoute, lastFix;
	string route, fix;
	float lastLat(0), lat, lon;
	int lastRouteId(-1);
	constexpr int bufferSize = 128;
	char buffer[bufferSize];
	while (ifs.getline(buffer, bufferSize))
	{
		if (buffer[0] == ';')
			continue;
		std::istringstream iss(buffer);
		iss >> route >> fix >> fix >> lat;
		lastFix = fix;
		lastLat = lat;
		graph->graphHelper->AddRouteString(route);
		lastRoute = route;
		lastRouteId = graph->graphHelper->GetRouteIndex(route);
		break;
	}
	while (ifs >> route >> fix >> fix >> lat >> lon)
	{
		if (route == lastRoute)
		{
			int first = graph->graphHelper->FindVertexId(lastFix, lastLat);
			int second = graph->graphHelper->FindVertexId(fix, lat);
			graph->AddUndirectedEdge(first, second, lastRouteId);
		}
		else
		{
			graph->graphHelper->AddRouteString(route);
			lastRoute = route;
			lastRouteId = graph->graphHelper->GetRouteIndex(route);
		}
		lastFix = fix;
		lastLat = lat;
	}
	ifs.close();
}

void bf::DataSet::Initialize(void)
{
	InitializeAirports();
	InitializeFixes();
	InitializeRoutes();
	InitializeRoutes();
	puts("init complete");
}

void bf::DataSet::InitializeAirport(string s)
{
	for (auto &i : s)
		i = (char) toupper(i);
	if(is->initializedAirports.find(s) != is->initializedAirports.end())
		return;
	is->initializedAirports.insert(s);
	printf("init ap %s\n", s.c_str());
	std::ostringstream oss;
	oss << path << "/SIDSTARS/" << s;
	std::ifstream ifs(oss.str());
	if (!ifs.is_open())
		throw std::runtime_error("Failed to open file: SIDSTARS/" + s + ".txt");
	const static int bufferSize = 1024;
	char buffer[bufferSize];

}
