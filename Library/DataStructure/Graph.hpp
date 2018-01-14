#ifndef LIBRARY_GRAPH_HPP
#define LIBRARY_GRAPH_HPP

#include <memory>
#include <vector>
#include <string>
#include "GraphHelper.hpp"
#include "../DataSet/Leg.hpp"
#include "../DataSet/Result.hpp"

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

		size_t MAX_VERTICES = 100000;
		const static size_t EST_DEGREE = 16;
		std::vector<Edge> *edges;

		void ConstructResult(std::shared_ptr<Result> result, int s, int t, float *d, int *prev);

	public:
		GraphHelper *graphHelper;

		Graph(void);

		~Graph(void);

		void AddEdge(int v1, int v2, int route);

		void AddUndirectedEdge(int v1, int v2, int route);

		std::shared_ptr<Result> Dijkstra(int start, int terminal);

		void SetMaxVertices(size_t MAX_VERTICES);

		void AllocateEdges(void);
	};
}
#endif //LIBRARY_GRAPH_HPP
