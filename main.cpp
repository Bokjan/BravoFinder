#include "Interfaces.hpp"
#define debug(...) fprintf(stderr, __VA_ARGS__)
int main(void)
{
	bool ret = Bravo::SetNavDataPath("/Volumes/HGST_D/Prepar3D v3/PMDG/");
	Bravo::InitializeAirports();
	return 0;
}
