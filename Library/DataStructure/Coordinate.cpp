#include <cmath>
#include "Coordinate.hpp"

bf::Coordinate::Coordinate(void) :
		latitude(0), longitude(0)
{
}

bf::Coordinate::Coordinate(float latitude, float longitude) :
		latitude(latitude), longitude(longitude)
{
}

static inline float rad(float x)
{
	const static auto PI = static_cast<float>(acos(-1.));
	return static_cast<float>(x * PI / 180.);
}

float bf::Coordinate::DistanceFrom(bf::Coordinate x)
{
	constexpr static float EARTH_RADIUS = 6378.137 / 1.852; // NM
	float radLat1 = rad(latitude);
	float radLat2 = rad(x.latitude);
	float a = radLat1 - radLat2;
	float b = rad(longitude) - rad(x.longitude);
	float s = static_cast<float>(
			2 * asin(sqrt(pow(sin(a / 2), 2) +
			              cos(radLat1) *
			              cos(radLat2) *
			              pow(sin(b / 2), 2)))
	);
	return s * EARTH_RADIUS;
}
