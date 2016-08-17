namespace Bravo
{
	extern bool IsFileExists(const char *);
	extern int StringLength(char *);
	extern void StringCopy(const char*, char *);
	extern bool StringEquals(const char *, const char *);
	extern double GetDistance_KM(double lat1, double lon1, double lat2, double lon2);
	extern double GetDistance_NM(double lat1, double lon1, double lat2, double lon2);
}
