#include <queue>
#include <cmath>
#include <cstdio>
#include <algorithm>
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
	edges[v1].emplace_back(Edge(v2, dist, route));
	edges[v2].emplace_back(Edge(v1, dist, route));
}

void bf::Graph::AddEdge(int v1, int v2, int route)
{
	static auto vertices = graphHelper->GetVertices();
	auto c1 = vertices[v1].coord;
	auto c2 = vertices[v2].coord;
	float dist = DistanceBetween(c1.latitude, c1.longitude, c2.latitude, c2.longitude);
	edges[v1].emplace_back(Edge(v2, dist, route));
}

void bf::Graph::SetGraphHelper(bf::GraphHelper *gh)
{
	graphHelper = gh;
}

std::vector<bf::Leg> bf::Graph::Dijkstra(int start, int terminal)
{
	std::vector<Leg> ret;
	using P = std::pair<float, int>;
	const static float INF = 30000.0;
	auto d = new float[MAX_VERTICES];
	auto prev = new int[MAX_VERTICES];
	static auto vertices = graphHelper->GetVertices();
	std::priority_queue<P, std::vector<P>, std::greater<P>> pq;
	std::fill(d, d + MAX_VERTICES, INF);
	d[start] = 0;
	pq.push(std::make_pair(0.0, start));
	while (!pq.empty())
	{
		auto c = pq.top();
		pq.pop();
		if (d[c.second] < c.first)
			continue;
		for (auto i : edges[c.second])
		{
			if (d[c.second] + i.dist < d[i.id])
			{
				prev[i.id] = c.second;
				d[i.id] = d[c.second] + i.dist;
				pq.push(std::make_pair(d[i.id], i.id));
			}
		}
	}
	if (d[terminal] == INF)
		ret.emplace_back(Leg(vertices[start].coord.DistanceFrom(vertices[terminal].coord),
		                     graphHelper->GetVertexString(start),
		                     graphHelper->GetVertexString(terminal), "DCT"));
	else
		this->ConstructLegs(ret, start, terminal, d, prev);
	delete[] d;
	delete[] prev;
	return ret;
}

void bf::Graph::ConstructLegs(std::vector<Leg> &l, int s, int t, float *d, int *prev)
{
	std::vector<int> path;
	for (int pos = t;;)
	{
		path.emplace_back(pos);
		if (pos == s)
			break;
		pos = prev[pos];
	}
	std::reverse(path.begin(), path.end());
	for (auto i = 0; i < path.size() - 1; ++i)
	{
		string route;
		float distance = 0;
		auto from = graphHelper->GetVertexString(path[i]);
		auto to = graphHelper->GetVertexString(path[i + 1]);
		for (auto &e : edges[path[i]])
		{
			if (e.id == path[i + 1])
			{
				if (!route.empty() && route != "DCT")
					break;
				distance = e.dist;
				route = graphHelper->GetRouteString(e.routeId);
			}
		}
		l.emplace_back(Leg(distance, from, to, route));
	}
}
