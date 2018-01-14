#include "Leg.hpp"

bf::Leg::Leg(void) = default;

bf::Leg::Leg(float distance, string from, string to, string route) :
		distance(distance), from(from), to(to), route(route)
{

}
