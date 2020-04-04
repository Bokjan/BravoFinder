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

		std::set<int> ParseSID(int vid, std::ifstream &is, GraphHelper *gh);

		std::set<int> ParseSTAR(int vid, std::ifstream &is, GraphHelper *gh);
	};
}

static inline void TruncateNB(string &s)
{
	if (s.size() < 4)
		return;
	if (s[s.size() - 2] == 'N'
	    && s[s.size() - 1] == 'B')
		s.pop_back(), s.pop_back();
}

std::set<int> bf::InternalStruct::ParseSID(int vid, std::ifstream &is, GraphHelper *gh)
{
	std::set<int> v;
	auto vertices = gh->GetVertices();
	constexpr int bufferSize = 1024;
	char buffer[bufferSize];
	string s, fix;
	while (is >> s)
		if (s == "SIDS")
			break;
	while (is.getline(buffer, bufferSize))
	{
		if (buffer[0] == 'E' && buffer[1] == 'N' && buffer[2] == 'D')
			break;
		std::istringstream iss(buffer);
		while (iss >> s)
			if (s == "FIX")
				iss >> fix;
		TruncateNB(fix);
		int fixid = -1;
		auto range = gh->GetVertexRange(fix);
		for (auto i = range.first; i != range.second; ++i)
		{
			// magic number, solving duplicated fixes
			if (vertices[vid].coord.DistanceFrom(vertices[i->second].coord) < 400)
			{
				fixid = i->second;
				break;
			}
		}
		if (fixid != -1)
			v.insert(fixid);
	}
	return v;
}

std::set<int> bf::InternalStruct::ParseSTAR(int vid, std::ifstream &is, GraphHelper *gh)
{
	std::set<int> v;
	auto vertices = gh->GetVertices();
	constexpr int bufferSize = 1024;
	char buffer[bufferSize];
	string s, fix;
	while (is.getline(buffer, bufferSize))
	{
		if (buffer[0] == 'E' && buffer[1] == 'N' && buffer[2] == 'D')
			break;
		std::istringstream iss(buffer);
		iss >> s;
		while (iss >> s)
			if (s == "FIX")
			{
				iss >> fix;
				break;
			}
		TruncateNB(fix);
		int fixId = -1;
		auto range = gh->GetVertexRange(fix);
		for (auto i = range.first; i != range.second; ++i)
			// magic number, solving duplicated fixes
			if (vertices[vid].coord.DistanceFrom(vertices[i->second].coord) < 400)
			{
				fixId = i->second;
				break;
			}
		if (fixId != -1)
			v.insert(fixId);
	}
	return v;
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
	if (bIsAirportsInitialized)
		return;
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
	bIsAirportsInitialized = true;
}

void bf::DataSet::InitializeFixes(void)
{
	if (bIsFixesInitialized)
		return;
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
	graph->SetMaxVertices(graph->graphHelper->GetVertices().size());
	graph->AllocateEdges();
	bIsFixesInitialized = true;
}

void bf::DataSet::InitializeRoutes(void)
{
	if (bIsRoutesInitialized)
		return;
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
	bIsRoutesInitialized = true;
}

void bf::DataSet::Initialize(void)
{
	InitializeAirports();
	InitializeFixes();
	InitializeRoutes();
	InitializeRoutes();
}

void bf::DataSet::InitializeAirport(string s)
{
	for (auto &i : s)
		i = (char) toupper(i);
	if (is->initializedAirports.find(s) != is->initializedAirports.end())
		return;
//	auto DCT_ID = graph->graphHelper->GetRouteIndex("DCT");
	auto SID_ID = graph->graphHelper->GetRouteIndex("SID");
	auto STAR_ID = graph->graphHelper->GetRouteIndex("STAR");
	auto vertices = graph->graphHelper->GetVertices();
	std::ostringstream oss;
	oss << path << "/SIDSTARS/" << s << ".txt";
	std::ifstream ifs(oss.str());
	if (!ifs.is_open())
		throw std::runtime_error("Failed to open file: SIDSTARS/" + s + ".txt");
	int airportVid = graph->graphHelper->FindVertexId(s);
	if (airportVid == -1)
		throw std::invalid_argument(s + " vertex doesn't exist");
	auto sidFixes = is->ParseSID(airportVid, ifs, graph->graphHelper);
	auto starFixes = is->ParseSTAR(airportVid, ifs, graph->graphHelper);
	for (auto i : sidFixes)
	{
		graph->AddEdge(airportVid, i, SID_ID);
	}
	for (auto i : starFixes)
	{
		graph->AddEdge(i, airportVid, STAR_ID);
	}
	ifs.close();
	is->initializedAirports.insert(s);
}

inline static string StringToUpper(const string &s)
{
	auto r = s;
	for (auto &i : r)
		i = (char) toupper(i);
	return r;
}

std::shared_ptr<bf::Result> bf::DataSet::FindRoute(const string &depature, const string &arrival)
{
	this->InitializeAirport(depature);
	this->InitializeAirport(arrival);
	int u = graph->graphHelper->FindVertexId(StringToUpper(depature));
	int v = graph->graphHelper->FindVertexId(StringToUpper(arrival));
	auto r = graph->Dijkstra(u, v);
	// Generate route string
	r->route = this->GenerateRouteString(r->legs);
	return r;
}

string bf::DataSet::GenerateRouteString(const std::vector<Leg> &legs)
{
	if (legs.size() == 1)
		return legs.front().from + " " + legs.front().route + " " + legs.front().to;
	string r = legs.front().from + " ";
	r += legs.front().route + " ";
	string lastRoute;
	for (decltype(legs.size()) i = 1; i < legs.size(); ++i)
	{
		if (legs[i].route == lastRoute)
			continue;
		lastRoute = legs[i].route;
		r += legs[i - 1].to + " ";
		r += legs[i].route + " ";
	}
	r += legs.back().to;
	return r;
}
