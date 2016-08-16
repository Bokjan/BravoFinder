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
	}
}
