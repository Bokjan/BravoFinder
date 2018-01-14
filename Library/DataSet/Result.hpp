#ifndef BRAVOFINDERLIBRARY_RESULT_HPP
#define BRAVOFINDERLIBRARY_RESULT_HPP

#include <vector>
#include <string>
#include "Leg.hpp"
#include "../DataStructure/Coordinate.hpp"

namespace bf
{
	using std::string;

	class WayPoint
	{
	public:
		string name;
		Coordinate coord;

		WayPoint(const string &name, Coordinate coord);
	};

	class Result
	{
	private:
		float *d;
		int *prev;
	public:
		string route;
		std::vector<Leg> legs;
		std::vector<WayPoint> waypoints;

		Result(void);
	};
}


#endif //BRAVOFINDERLIBRARY_RESULT_HPP
