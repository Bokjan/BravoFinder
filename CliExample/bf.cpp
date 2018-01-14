#include <string>
#include <fstream>
#include <iostream>
#include "bfinder/bfinder.h"
using std::string;
using bf::DataSet;
string GetDataPath(void)
{
	size_t bufferSize = 2048;
	char buffer[bufferSize];
	std::ifstream ifs("navdata.txt");
	if(!ifs.is_open())
		throw std::runtime_error("navdata.txt doesn't exist");
	ifs.getline(buffer, bufferSize);
	ifs.close();
	return string(buffer);
}
int main(int argc, char *argv[])
{
	DataSet ds;
	ds.SetDataPath(GetDataPath());
	ds.Initialize();
	string dep, arr;
	for( ; ;)
	{
		std::cout << "Depature (ICAO): ";
		std::cin >> dep;
		std::cout << "Arrival (ICAO): ";
		std::cin >> arr;
		auto result = ds.FindRoute(dep, arr);
		std::cout << result->route << std::endl;
		std::cout << "From\tTo\tVia\tDistance" << std::endl;
		for(auto &i : result->legs)
			std::cout << i.from << '\t' << i.to << '\t' << i.route << '\t' << i.distance << std::endl;
		std::cout << "Wpt\tLat\tLon" << std::endl;
		for(auto &i : result->waypoints)
			std::cout << i.name << '\t' << i.coord.lat << '\t' << i.coord.lon << std::endl;
	}
}
