#include "Result.hpp"

bf::Result::Result(void) = default;

bf::WayPoint::WayPoint(const std::string &name, bf::Coordinate coord) :
		name(name), coord(coord)
{

}
