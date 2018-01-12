#include <cmath>
#include "Graph.hpp"

bf::Graph::Graph(void)
{
	for (auto &i : edges)
		i.reserve(this->EST_DEGREE);
	this->SetGraphHelper(new GraphHelper);
}

bf::Graph::~Graph(void)
{

}

bf::Graph::Edge::Edge(int id, float dist, int routeId) :
		id(id), dist(dist), routeId(routeId)
{

}

void bf::Graph::AddEdge(int v1, int v2, int route)
{
	auto vertices = graphHelper->GetVertices();
	float dist = vertices[v1].coord.DistanceFrom(vertices[v2].coord);
	edges[v1].emplace_back(Edge(v2, dist, route));
}

static inline double rad(double x)
{
	const static auto PI = static_cast<float>(acos(-1.));
	return static_cast<float>(x * PI / 180.);
}

static inline double square(double x)
{
	return x * x;
}

static inline float DistanceBetween(float lat1, float lon1, float lat2, float lon2)
{
	constexpr static double EARTH_RADIUS = 6378.137 / 1.852; // NM
	auto radLat1 = rad(lat1);
	auto radLat2 = rad(lat2);
	auto a = radLat1 - radLat2;
	auto b = rad(lon1) - rad(lon2);
	auto s = 2 * asin(sqrt(square(sin(a / 2)) +
	                       cos(radLat1) *
	                       cos(radLat2) *
	                       square(sin(b / 2))));
	return static_cast<float>(s * EARTH_RADIUS);
}

void bf::Graph::AddUndirectedEdge(int v1, int v2, int route)
{
	static auto vertices = graphHelper->GetVertices();
	auto c1 = vertices[v1].coord;
	auto c2 = vertices[v2].coord;
	float dist = DistanceBetween(c1.latitude, c1.longitude, c2.latitude, c2.longitude);
	edges[v1].push_back(Edge(v2, dist, route));
	edges[v2].push_back(Edge(v1, dist, route));
}

void bf::Graph::SetGraphHelper(bf::GraphHelper *graphHelper)
{
	Graph::graphHelper = graphHelper;
}
