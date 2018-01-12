#include "GraphHelper.hpp"

inline static float abs(float x)
{
	return x > 0 ? x : -x;
}

bf::GraphHelper::GraphHelper(void)
{
	// A estimated value
	routes.reserve(this->MAX_ROUTES);
	vertices.reserve(this->MAX_VERTICES);
}

void bf::GraphHelper::AddVertex(const string &ident, float latitude, float longitude)
{
	if (FindVertexId(ident, latitude) != -1)
		return;
	Vertex v;
	v.id = vid++;
	v.coord.latitude = latitude;
	v.coord.longitude = longitude;
	v.name = ident;
	vertices.push_back(v);
	vmap.insert(std::make_pair(ident, v.id));
}

int bf::GraphHelper::FindVertexId(const string &ident, float latitude)
{
	constexpr float epsilon = 1e0;
	const auto range = vmap.equal_range(ident);
	for (auto it = range.first; it != range.second; ++it)
		if (abs(vertices[it->second].coord.latitude - latitude) < epsilon)
			return it->second;
	return -1;
}

int bf::GraphHelper::FindVertexId(const std::string &ident)
{
	auto f = vmap.find(ident);
	return f == vmap.end() ? -1 : f->second;
}

const std::string &bf::GraphHelper::GetRouteString(int index)
{
	return routes[index];
}

void bf::GraphHelper::AddRouteString(const std::string &route)
{
	routes.push_back(route);
	rmap.insert(std::make_pair(route, (int) (routes.size() - 1)));
}

const std::vector<bf::Vertex> &bf::GraphHelper::GetVertices(void)
{
	return vertices;
}

int bf::GraphHelper::GetRouteIndex(const std::string &route)
{
	auto f = rmap.find(route);
	return f == rmap.end() ? -1 : f->second;
}
