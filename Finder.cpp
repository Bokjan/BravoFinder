#include "Finder.hpp"
namespace Internal
{
	int idCounter = 0;
	const int MAX_V = 150000;
	std::vector<Node> nodes;
	std::vector<string> routes;
	std::map<string, int> nodemap;
	std::map<string, int> routemap;
	std::vector<Edge> g[MAX_V];
}
