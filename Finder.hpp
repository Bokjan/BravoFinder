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
		NDB,
		FIX
	};
	struct Node
	{
		int id;
		//NodeType type;
		char icao[5];
		double lat, lon;
		Node(int id, double lat, double lon):
			id(id), lat(lat), lon(lon) { }
	};
	extern int idCounter;
	extern std::map<string, int> nodemap;
	extern std::vector<Node> nodes;
}
