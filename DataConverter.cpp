#include <map>
#include <cstdio>
#include <string>
#include <vector>
#include "Finder.hpp"
#include "Utilities.hpp"
using std::string;
namespace Internal
{
	int GetNodeIndex(char *str, double lat)
	{
		static int id = 0;
		std::vector<HashNode>::iterator it;
		int hash = Bravo::BkdrHash(str) % MODULUS;
		std::vector<HashNode> &vector = nodemap[hash];
		for(it = vector.begin(); it != vector.end(); ++it)
			if(it->lat == lat)
				return it->id;
		nodemap[hash].push_back(HashNode(id, lat));
		return id++;
	}
	int GetAPIndex(const char *str)
	{
		std::vector<HashNode>::iterator it;
		int hash = Bravo::BkdrHash(str) % MODULUS;
		std::vector<HashNode> &vector = nodemap[hash];
		for(it = vector.begin(); it != vector.end(); ++it)
			return it->id;
		return 0;
	}
	int GetDAIndex(int ApId, char *str)
	{
		double alat = nodes[ApId].lat;
		double alon = nodes[ApId].lon;
		std::vector<HashNode>::iterator it;
		int hash = Bravo::BkdrHash(str) % MODULUS;
		std::vector<HashNode> &vector = nodemap[hash];
		for(it = vector.begin(); it != vector.end(); ++it)
		{
			double flat = it->lat;
			double flon = nodes[it->id].lon;
			if(Bravo::GetDistance_NM(alat, alon, flat, flon) < 1000)
				return it->id;
		}
		return 0;
	}
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
			sscanf(row + 4, "%lf%lf", &lat, &lon);
			int currentId = GetNodeIndex(icao, lat);
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
			if(row[0] == ';' || row[0] == '/')
				continue;
			//Node
			int seq;
			char route[16];
			char point[16];
			double lat, lon;
			sscanf(row, "%s%d%s%lf%lf", route, &seq, point, &lat, &lon);
			int nodeId = GetNodeIndex(point, lat);
			if(nodeId == nodes.size())
			{
				nodes.push_back(Node(nodeId, lat, lon));
				Bravo::StringCopy(point, nodes[nodeId].name);
			}
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
			thisNode = GetNodeIndex(point, lat);
			if(lastSeq == seq - 1)
			{
				double dist = Bravo::GetDistance_NM(lat, lon, nodes[prevNode].lat, nodes[prevNode].lon);
				g[prevNode].push_back(Edge(thisNode, routemap[route], dist));
				g[thisNode].push_back(Edge(prevNode, routemap[route], dist));
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
	inline void GetDepArrFixString(FILE *fp, char *str)
	{
		fscanf(fp, "%s", str);
		/*
		  NAVDATA appends "NB" to the end of a NDB waypoint
		  Example:
		  There is an NDB near ZBAA called CDY(Che Dao Yu)
		  in SIDSTAR files CDY becomes CDYNB
		  So the following codes detect and change these NDBs to correct format
		*/
		int len = Bravo::StringLength(str);
		if(len > 3 && str[len - 1] == 'B' && str[len - 2] == 'N')
			str[len - 2] = '\0';
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
		int ApId = GetAPIndex(ICAO);
		while(fscanf(fp, "%s", word) != EOF)
		{
  			if(Bravo::StringEquals("SIDS", word))
			{
				SID_LABEL:
				while(fscanf(fp, "%s", word))
				{
					if(Bravo::StringEquals("FIX", word))
						GetDepArrFixString(fp, fix);
					else if(Bravo::StringEquals("SID", word))
					{
						if(depFix.find(fix) == depFix.end())
							depFix[fix] = GetDAIndex(ApId, fix);
						goto SID_LABEL;
					}
					else if(Bravo::StringEquals("STAR", word))
					{
						if(depFix.find(fix) == depFix.end())
							depFix[fix] = GetDAIndex(ApId, fix);
						goto STAR_LABEL;
					}
					else if (Bravo::StringEquals("ENDSIDS", word)) 
					{
						break;
					}
				}
				if(depFix.find(fix) == depFix.end())
					depFix[fix] = GetDAIndex(ApId, fix);
			}
			else if(Bravo::StringEquals("STAR", word))
			{
				STAR_LABEL:
				while(fscanf(fp, "%s", word))
				{
					if(Bravo::StringEquals("FIX", word))
					{
						GetDepArrFixString(fp, fix);
						break;
					}
				}
				if(arrFix.find(fix) == arrFix.end())
					arrFix[fix] = GetDAIndex(ApId, fix);
			}
		}
		std::map<string, int>::iterator it;
		Node &AP = nodes[GetAPIndex(ICAO)];
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
