#include "RawDataReader.h"
#define iter(c) c::iterator
using std::endl;
void RawDataReader::ReadWaypoints(void)
{
	FILE *fin;
	string file;
	//VORs
	file = GetDataPath() + "pssvor.dat";
	fin = fopen(file.c_str(), "r");
	printf("Reading VOR info from %s\n", file.c_str());
	{
		char name[128], note[128];
		double lat, lon, freq;
		while (fscanf(fin, "%s%lf%lf%lf%s", name, &lat, &lon, &freq, note) != EOF){
			std::vector<Waypoint> &vec = Waypoints[name];
			g.push_back(Vertex(Enum::VOR, name, lat, lon));
			vec.push_back(Waypoint(vertexIdent, name, lat, lon));
			++vertexIdent;
		}
	}
	fclose(fin);
	//NDBs
	file = GetDataPath() + "pssndb.dat";
	fin = fopen(file.c_str(), "r");
	printf("Reading NDB info from %s\n", file.c_str());
	{
		char name[128], note[128];
		double lat, lon, freq;
		while (fscanf(fin, "%s%lf%lf%lf%s", name, &lat, &lon, &freq, note) != EOF){
			std::vector<Waypoint> &vec = Waypoints[name];
			g.push_back(Vertex(Enum::NDB, name, lat, lon));
			vec.push_back(Waypoint(vertexIdent, name, lat, lon));
			++vertexIdent;
		}
	}
	fclose(fin);
	//FIXes
	file = GetDataPath() + "psswpt.dat";
	fin = fopen(file.c_str(), "r");
	printf("Reading FIX info from %s\n", file.c_str());
	{
		char name[32];
		double lat, lon;
		while (fscanf(fin, "%c%c%c%c%c%lf%lf", \
			&name[0], &name[1], &name[2], &name[3], &name[4], \
			&lat, &lon) != EOF){
			name[5] = 0;
			std::vector<Waypoint> &vec = Waypoints[name];
			g.push_back(Vertex(Enum::FIX, name, lat, lon));
			vec.push_back(Waypoint(vertexIdent, name, lat, lon));
			++vertexIdent;
			fgets(name, 31, fin);
		}
	}
	fclose(fin);
	//APs
	file = GetDataPath() + "pssapt.dat";
	fin = fopen(file.c_str(), "r");
	printf("Reading APT info from %s\n", file.c_str());
	{
		int _i;
		char name[32], note[128];
		double lat, lon;
		while (fscanf(fin, "%s%lf%lf%d%s", name, &lat, &lon, &_i, note) != EOF){
			std::vector<Waypoint> &vec = Waypoints[name];
			g.push_back(Vertex(Enum::AP, name, lat, lon));
			vec.push_back(Waypoint(vertexIdent, name, lat, lon));
			Airports.insert(std::make_pair(name, Airport(name, lat, lon)));
			++vertexIdent;
		}
	}
	fclose(fin);
}
void RawDataReader::Output(void)
{
	string outfile = GetOutPath() + "out.dat";
	std::ofstream fout(outfile);
	printf("Outputing data to %s\n", outfile.c_str());
	for (std::map<string, std::vector<Waypoint> >::iterator it = Waypoints.begin(); \
		it != Waypoints.end(); ++it){
		//printf("%d\n", it->second.size());
		for (std::vector<Waypoint>::iterator ite = it->second.begin(); \
			ite != it->second.end(); ++ite){
			fout << ite->ident << ' ' << ite->name << ' ' << ite->lat << ' ' << ite->lon << endl;
		}
	}
	fout.close();
}