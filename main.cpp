#include "Interfaces.hpp"
#include "Utilities.hpp"
#define debug(...) fprintf(stderr, __VA_ARGS__)
int main(int argc, char *argv[])
{
	char *dep = argv[1];
	char *arr = argv[2];
	int data = 0;
	if(Bravo::SetNavDataPath("/Volumes/HGST_D/Prepar3D v3/PMDG/"))
		++data;
	if(Bravo::InitializeAirports())
		puts("Airport data initialized!"), ++data;
	if(Bravo::InitializeNavigationRoutes())
		puts("Navigation route data initialized!"), ++data;
	if(Bravo::InitializeDAFixes(dep))
		printf("Dep/Arr fixes of %s initialized!\n", dep), ++data;
	if(Bravo::InitializeDAFixes(arr))
		printf("Dep/Arr fixes of %s initialized!\n", arr), ++data;
	if(data == 5)
		Bravo::FindRoute(dep, arr);
	else
		puts("Initialization failed!");
	return 0;
}
