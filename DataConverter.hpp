#include <string>
namespace Internal
{
	extern int GetAPIndex(const char *str);
	extern void InitializeAirports(std::string);
	extern void InitializeNavigationRoutes(std::string);
	extern void InitializeDAFixes(char*, char*);
}
