#include "Interfaces.hpp"
#include "Utilities.hpp"
#define debug(...) fprintf(stderr, __VA_ARGS__)
int main(void)
{
	bool ret = Bravo::SetNavDataPath("/Volumes/HGST_D/Prepar3D v3/PMDG/");
	Bravo::InitializeAirports();
	puts("Airport data initialized!");
	Bravo::InitializeNavigationRoutes();
	puts("Navigation route data initialized!");
	Bravo::InitializeDAFixes("ZGHA");
	printf("Dep/Arr fixes of %s initialized!\n", "ZGHA");
	Bravo::InitializeDAFixes("ZGGG");
	printf("Dep/Arr fixes of %s initialized!\n", "ZGGG");
	return 0;
}
