#ifndef LIBRARY_GRAPH_HPP
#define LIBRARY_GRAPH_HPP

#include <vector>
#include <string>
#include "GraphHelper.hpp"

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
	public:
		GraphHelper *graphHelper;

		Graph(void);

		~Graph(void);

		void AddEdge(int v1, int v2, int route);

		void AddUndirectedEdge(int v1, int v2, int route);

		void SetGraphHelper(GraphHelper *graphHelper);

		void Dijkstra(int start, int terminal);
	};
}
#endif //LIBRARY_GRAPH_HPP
