#include <string>
#include <iostream>
#include "Interfaces.hpp"
#include "Utilities.hpp"
#define debug(...) fprintf(stderr, __VA_ARGS__)
using std::string;
int main(int argc, char *argv[])
{
	string dep, arr;
	char NavDataPath[1024];
	FILE *fp = fopen("navdata.txt", "r");
	fgets(NavDataPath, 1023, fp);
	fclose(fp);
	NavDataPath[Bravo::StringLength(NavDataPath) - 1] = '\0';
	if(!Bravo::SetNavDataPath(NavDataPath))
	{
		puts("Configuration Error!");
		return 0;
	}
	printf("Depature(ICAO): ");
	std::cin >> dep;
	printf("Arrival (ICAO): ");
	std::cin >> arr;
	if(!Bravo::InitializeAirports())
	{
		puts("Airport Info Initialization Failed!");
		return 0;
	}
	if(!Bravo::InitializeNavigationRoutes())
	{
		puts("Route Info Initialization Failed!");
		return 0;
	}
	if(!Bravo::InitializeDAFixes(dep.c_str()))
	{
		puts("Dep/Arr Info Initialization Failed!");
		return 0;
	}
	if(!Bravo::InitializeDAFixes(arr.c_str()))
	{
		puts("Dep/Arr Info Initialization Failed!");
		return 0;
	}
	Bravo::FindRoute(dep.c_str(), arr.c_str());
	return 0;
}
