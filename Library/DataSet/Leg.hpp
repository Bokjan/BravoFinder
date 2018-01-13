#ifndef LIBRARY_LEG_HPP
#define LIBRARY_LEG_HPP

#include <string>

namespace bf
{
	using std::string;
	class Leg
	{
	public:
		float distance;
		string from, to, route;
		Leg(void);
		Leg(float distance, string from, string to, string route);
	};
}


#endif //LIBRARY_LEG_HPP
