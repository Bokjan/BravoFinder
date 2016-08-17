#include <cstdio>
#include "Interfaces.hpp"
#include "Utilities.hpp"
#include "DataConverter.hpp"
using std::string;
namespace Bravo
{
	string NAV_PATH;
	void HelloWorld(void)
	{
		puts("hello");
	}
	bool SetNavDataPath(const char *path)
	{
		NAV_PATH = path;
		string checker = NAV_PATH;
		checker += "NAVDATA/cycle_info.txt";
		if(IsFileExists(checker.c_str()))
			return true;
		else
			return false;
	}
	bool InitializeAirports(void)
	{
		string file = NAV_PATH + "NAVDATA/airports.dat";
		if(!IsFileExists(file.c_str()))
			return false;
		Internal::InitializeAirports(file);
		return true;
	}
	bool InitializeNavigationRoutes(void)
	{
		string file = NAV_PATH + "NAVDATA/wpNavRTE.txt";
		if(!IsFileExists(file.c_str()))
			return false;
		Internal::InitializeNavigationRoutes(file);
		return true;
	}
	bool InitializeDAFixes(const char *ICAO)
	{
		char file[128];
		sprintf(file, "%sSIDSTARS/%s.txt", NAV_PATH.c_str(), ICAO);
		if(!IsFileExists(file))
			return false;
		Internal::InitializeDAFixes(file, (char*)ICAO);
		return true;
	}
}
