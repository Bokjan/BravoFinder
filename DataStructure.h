#include "global.h"
namespace Enum{
	enum WpType{
		AP,
		VOR,
		NDB,
		FIX
	};
}
struct Vertex{
	Enum::WpType type;
	//int ident;
	string name;
	double lat, lon;
	Vertex(Enum::WpType _t, const char *_n, double _lat, double _lon)
	{
		type = _t;
		name = _n;
		lat = _lat;
		lon = _lon;
	}
};
struct Waypoint{
	int ident;
	string name;
	double lat, lon;
	Waypoint(int _i, string _n, double _lat, double _lon)
	{
		ident = _i;
		name = _n;
		lat = _lat;
		lon = _lon;
	}
};
struct Airport{
	string ICAO;
	double lat, lon;
	std::vector<Waypoint> wps;
	Airport(string _i, double _lat, double _lon)
	{
		ICAO = _i;
		lat = _lat;
		lon = _lon;
	}
};
extern std::map<string, Airport> Airports;
extern std::map<string, std::vector<Waypoint> > Waypoints;
extern std::vector<Vertex> g;