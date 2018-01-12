#ifndef LIBRARY_VERTEX_HPP
#define LIBRARY_VERTEX_HPP

#include <string>
#include "Coordinate.hpp"

namespace bf
{
	class Vertex
	{
	public:
		int id;
		Coordinate coord;
		std::string name;
	};
}


#endif //LIBRARY_VERTEX_HPP
