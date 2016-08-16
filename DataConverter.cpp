#include <cstdio>
#include <string>
#include "Finder.hpp"
#include "Utilities.hpp"
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
			int currentId = nodes.size();
			for(int i = 0; i < 4; ++i)
				icao[i] = row[i];
			sscanf(row + 4, "%lf%lf", &lat, &lon);
			nodemap[string(icao)] = currentId;
			nodes.push_back(Node(currentId, lat, lon));
			for(int i = 0; i < 5; ++i)
				nodes[currentId].name[i] = icao[i];
		}
	}
	void InitializeNavigationRoutes(string file)
	{
		FILE *fp = fopen(file.c_str(), "r");
		char row[128];
		while(fgets(row, 127, fp))
		{
			if(row[0] == ';')
				continue;
			//Node
			int seq;
			char route[8];
			char point[8];
			double lat, lon;
			int nodeId = nodes.size();
			sscanf(row, "%s%d%s%lf%lf", route, &seq, point, &lat, &lon);
			nodes.push_back(Node(nodeId, lat, lon));
			nodemap[string(point)] = nodeId;
			Bravo::StringCopy(point, nodes[idCounter].name);
			//Route
			if(routemap.find(route) != routemap.end())
				continue;
			int routeId = routes.size();
			routes.push_back(route);
			routemap[route] = routeId;
		}
		fseek(fp, 0, SEEK_SET);
		int lastSeq = -1;
		int lastNode = -1;
		int thisNode = -1;
		while(fgets(row, 127, fp))
		{
			if(row[0] == ';')
				continue;
			int seq;
			char route[8];
			char point[8];
			double lat, lon;
			sscanf(row, "%s%d%s%lf%lf", route, &seq, point, &lat, &lon);
			thisNode = nodemap[point];
			if(lastSeq == seq - 1)
			{
				double dist = Bravo::GetDistance(lat, lon, nodes[lastNode].lat, nodes[lastNode].lon);
				g[lastNode].push_back(Edge(thisNode, routemap[route], dist));
				g[thisNode].push_back(Edge(thisNode, routemap[route], dist));
			}
			lastSeq = seq;
			lastNode = nodemap[point];
		}
	}
}
