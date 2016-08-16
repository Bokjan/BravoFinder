#include <cstdio>
#include <string>
#include "Finder.hpp"
using std::string;
namespace Internal
{
	void InitializeAirports(string file)
	{
		FILE *fp = fopen(file.c_str(), "r");
		char icao[8];
		char row[128];
		double lat, lon;
		icao[4] = '\0';
		while(fgets(row, 127, fp))
		{
			if(row[0] == ';')
				continue;
			for(int i = 0; i < 4; ++i)
				icao[i] = row[i];
			nodemap[string(icao)] = idCounter;
			sscanf(row + 4, "%lf%lf", &lat, &lon);
			nodes.push_back(Node(idCounter, lat, lon));
			for(int i = 0; i < 5; ++i)
				nodes[idCounter].icao[i] = icao[i];
			++idCounter;
		}
	}
}
