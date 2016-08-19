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
	for(int i = 1023; i > 0; --i)
		if(NavDataPath[i] == '\\' || NavDataPath[i] == '/')
		{
			NavDataPath[i + 1] = '\0';
			break;
		}
	if(!Bravo::SetNavDataPath(NavDataPath))
	{
		puts("配置文件错误");
		return 0;
	}
	printf("出发(ICAO): ");
	std::cin >> dep;
	printf("到达(ICAO): ");
	std::cin >> arr;
	if(!Bravo::InitializeAirports())
	{
		puts("机场信息初始化错误！");
		return 0;
	}
	if(!Bravo::InitializeNavigationRoutes())
	{
		puts("航路信息初始化错误！");
		return 0;
	}
	if(!Bravo::InitializeDAFixes(dep.c_str()))
	{
		puts("出发机场进离场信息初始化错误！");
		return 0;
	}
	if(!Bravo::InitializeDAFixes(arr.c_str()))
	{
		puts("到达机场进离场信息初始化错误！");
		return 0;
	}
	Bravo::FindRoute(dep.c_str(), arr.c_str());
	return 0;
}
