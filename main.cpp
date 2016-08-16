#include "Interfaces.hpp"
#include "DataConverter.hpp"
#include "Finder.hpp"
#define debug(...) fprintf(stderr, __VA_ARGS__)
int main(void)
{
	bool ret = Bravo::SetNavDataPath("/Volumes/HGST_D/Prepar3D v3/PMDG/");
	Bravo::InitializeAirports();
	puts("Airport data initialized!");
	Bravo::InitializeNavigationRoutes();
	puts("Navigation route data initialized!");
	return 0;
}
