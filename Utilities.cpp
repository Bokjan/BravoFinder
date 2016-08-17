#include <cmath>
#include <cstdio>
namespace Bravo
{
	const double PI = 3.1415926535898;
	const double EARTH_RADIUS = 6378.137;
	bool IsFileExists(const char *file)
	{
		FILE *fp = fopen(file, "rb");
		if(NULL == fp)
			return false;
		else
		{
			fclose(fp);
			return true;
		}
	}
	int StringLength(char *_s)
	{
		char *__s = _s;
		while(*__s != '\0')
			++__s;
		return (int)(__s - _s);
	}
	void StringCopy(const char *src, char *dest)
	{
		while(*src != '\0')
		{
			*dest = *src;
			++dest, ++src;
		}
	}
	bool StringEquals(const char *_a, const char *_b)
	{
		char *a = (char*)_a;
		char *b = (char*)_b;
		do
		{
			if(*a != *b)
				return false;
			++a, ++b;
		}
		while(*a != '\0' || *b != '\0');
		return true;
	}
	inline double rad(double x)
	{
		return x * PI / 180.0;
	}
	double GetDistance_KM(double lat1, double lon1, double lat2, double lon2)
	{
		double radLat1 = rad(lat1);
	    double radLat2 = rad(lat2);
	    double a = radLat1 - radLat2;
	    double b = rad(lon1) - rad(lon2);
		double s = 2 * asin(sqrt(pow(sin(a / 2), 2) +
			cos(radLat1) * cos(radLat2) * pow(sin(b / 2), 2)));
		//Get the km value
		s *= EARTH_RADIUS;
		return s;
	}
	double GetDistance_NM(double lat1, double lon1, double lat2, double lon2)
	{
		//1 nautical mile = 1.852 kilometers
		return GetDistance_KM(lat1, lon1, lat2, lon2) / 1.852;
	}
}
