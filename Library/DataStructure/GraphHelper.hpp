#ifndef LIBRARY_GRAPHHELPER_HPP
#define LIBRARY_GRAPHHELPER_HPP

#include <map>
#include <vector>
#include <string>
#include "Vertex.hpp"

namespace bf
{
	using std::string;

	class GraphHelper
	{
	private:
		const static size_t MAX_ROUTES = 10000;
		const static size_t MAX_VERTICES = 50000;
		int vid = 0;
		std::vector<string> routes;
		std::vector<Vertex> vertices;
		std::map<string, int> rmap;
		std::multimap<string, int> vmap;

	public:
		GraphHelper(void);

		void AddVertex(const string &ident, float latitude, float longitude);

		int FindVertexId(const string &ident);

		int FindVertexId(const string &ident, float latitude);

		void AddRouteString(const string &route);

		const string &GetRouteString(int index);

		int GetRouteIndex(const string &route);

		const std::vector<Vertex> &GetVertices(void);
	};
}


#endif //LIBRARY_GRAPHHELPER_HPP
