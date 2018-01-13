#ifndef LIBRARY_GRAPH_HPP
#define LIBRARY_GRAPH_HPP

#include <vector>
#include <string>
#include "GraphHelper.hpp"
#include "../DataSet/Leg.hpp"

namespace bf
{
	using std::string;

	class Graph
	{
	private:
		struct Edge
		{
			int id;
			float dist;
			int routeId;

			Edge(int id, float dist, int routeId);
		};

		const static size_t MAX_VERTICES = 50000;
		const static size_t EST_DEGREE = 16;
		std::vector<Edge> edges[MAX_VERTICES];

		void ConstructLegs(std::vector<Leg> &l, int s, int t, float *d, int *prev);

	public:
		GraphHelper *graphHelper;

		Graph(void);

		~Graph(void);

		void AddEdge(int v1, int v2, int route);

		void AddUndirectedEdge(int v1, int v2, int route);

		void SetGraphHelper(GraphHelper *graphHelper);

		std::vector<Leg> Dijkstra(int start, int terminal);
	};
}
#endif //LIBRARY_GRAPH_HPP
