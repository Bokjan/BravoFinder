#include <string>
namespace Bravo
{
	extern std::string NAV_PATH;
	extern void HelloWorld(void);
	extern bool SetNavDataPath(const char *Path);
	extern bool InitializeAirports(void);
	extern bool InitializeNavigationRoutes(void);
	extern bool InitializeDAFixes(const char *ICAO);
}
