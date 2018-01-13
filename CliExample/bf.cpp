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
		auto path = ds.FindDetailedRoute(dep, arr);
		for(auto &i : path)
			std::cout << i.from << '\t' << i.route << '\t' << i.to << '\t' << i.distance << std::endl;
		std::cout << ds.GenerateRouteString(path) << std::endl;
	}
}
