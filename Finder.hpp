#include <map>
#include <vector>
#include <string>
using std::string;
namespace Internal
{
	enum NodeType
	{
		AP,
		VOR,
		VORD,
		NDB,
		ILS,
		ILSD,
		FIX
	};
	struct Node
	{
		int id;
		//NodeType type;
		char name[8];
		double lat, lon;
		Node(int id, double lat, double lon):
			id(id), lat(lat), lon(lon) { }
	};
	struct Edge
	{
		int to, way;
		double dist;
		Edge(int to, int way, double dist):
			to(to), way(way), dist(dist) { }
	};
	extern const int MAX_V;
	extern int idCounter;
	extern int SidMapId;
	extern int StarMapId;
	extern std::map<string, int> nodemap;
	extern std::map<string, int> routemap;
	extern std::vector<Node> nodes;
	extern std::vector<string> routes;
	extern std::vector<Edge> g[];
	extern double FindRoute(int dep, int arr);
}
