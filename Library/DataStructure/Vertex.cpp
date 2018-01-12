#include "Vertex.hpp"

bf::Vertex::Vertex(void)
{

}

bf::Vertex::Vertex(int id, bf::Coordinate coord, std::string name) :
		id(id), coord(coord), name(name)
{

}
