#include "Interfaces.hpp"
#include "Utilities.hpp"
#define debug(...) fprintf(stderr, __VA_ARGS__)
int main(int argc, char *argv[])
{
	char *dep = argv[1];
	char *arr = argv[2];
	bool ret = Bravo::SetNavDataPath("/Volumes/HGST_D/Prepar3D v3/PMDG/");
	Bravo::InitializeAirports();
	puts("Airport data initialized!");
	Bravo::InitializeNavigationRoutes();
	puts("Navigation route data initialized!");
	Bravo::InitializeDAFixes(dep);
	printf("Dep/Arr fixes of %s initialized!\n", dep);
	Bravo::InitializeDAFixes(arr);
	printf("Dep/Arr fixes of %s initialized!\n", arr);
	Bravo::FindRoute(dep, arr);
	return 0;
}
