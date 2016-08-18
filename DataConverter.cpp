#include <map>
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
		fclose(fp);
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
			char route[16];
			char point[16];
			double lat, lon;
			int nodeId = nodes.size();
			sscanf(row, "%s%d%s%lf%lf", route, &seq, point, &lat, &lon);
			if(nodemap.find(point) == nodemap.end())
			{
				nodes.push_back(Node(nodeId, lat, lon));
				nodemap[string(point)] = nodeId;
				Bravo::StringCopy(point, nodes[nodeId].name);
			}
			else
				nodeId = nodemap[point];
			//Route
			if(routemap.find(route) != routemap.end())
				continue;
			int routeId = routes.size();
			routes.push_back(route);
			routemap[route] = routeId;
		}
		fseek(fp, 0, SEEK_SET);
		int lastSeq = -1;
		int prevNode = -1;
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
				double dist = Bravo::GetDistance_NM(lat, lon, nodes[prevNode].lat, nodes[prevNode].lon);
				g[prevNode].push_back(Edge(thisNode, routemap[route], dist));
				//g[thisNode].push_back(Edge(prevNode, routemap[route], dist));
			}
			lastSeq = seq;
			prevNode = thisNode;
		}
		routemap["SID"] = SidMapId = routes.size();
		routes.push_back("SID");
		routemap["STAR"] = StarMapId = routes.size();
		routes.push_back("STAR");
		fclose(fp);
	}
	void InitializeDAFixes(char *file, char *ICAO)
	{
		FILE *fp = fopen(file, "r");
		int offset;
		char fix[128];
		char word[128];
		char row[1024];
		std::map<string, int> depFix;
		std::map<string, int> arrFix;
		while(fscanf(fp, "%s", word) != EOF)
		{
			if(Bravo::StringEquals("SID", word))
			{
				SID_LABEL:
				while(fscanf(fp, "%s", word))
				{
					if(Bravo::StringEquals("FIX", word))
						fscanf(fp, "%s", fix);
					else if(Bravo::StringEquals("SID", word))
					{
						if(depFix.find(fix) == depFix.end())
							depFix[fix] = nodemap[fix];
						goto SID_LABEL;
					}
					else if(Bravo::StringEquals("STAR", word))
					{
						if(depFix.find(fix) == depFix.end())
							depFix[fix] = nodemap[fix];
						goto STAR_LABEL;
					}
				}
				if(depFix.find(fix) == depFix.end())
					depFix[fix] = nodemap[fix];
			}
			else if(Bravo::StringEquals("STAR", word))
			{
				STAR_LABEL:
				while(fscanf(fp, "%s", word))
				{
					if(Bravo::StringEquals("FIX", word))
					{
						fscanf(fp, "%s", fix);
						break;
					}
				}
				if(arrFix.find(fix) == arrFix.end())
					arrFix[fix] = nodemap[fix];
			}
		}
		std::map<string, int>::iterator it;
		Node &AP = nodes[nodemap[ICAO]];
		for(it = depFix.begin(); it != depFix.end(); ++it)
		{
			Node &FIX = nodes[it->second];
			double dist = Bravo::GetDistance_NM(AP.lat, AP.lon, FIX.lat, FIX.lon);
			g[AP.id].push_back(Edge(FIX.id, SidMapId, dist));
		}
		for(it = arrFix.begin(); it != arrFix.end(); ++it)
		{
			Node &FIX = nodes[it->second];
			double dist = Bravo::GetDistance_NM(AP.lat, AP.lon, FIX.lat, FIX.lon);
			g[FIX.id].push_back(Edge(AP.id, StarMapId, dist));
		}
		fclose(fp);
	}
}
