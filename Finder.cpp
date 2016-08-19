#include "Finder.hpp"
#include "Utilities.hpp"
#include <queue>
#include <vector>
#include <utility>
#include <cstdio>
namespace Internal
{
	int idCounter = 0;
	int SidMapId;
	int StarMapId;
	const int MAX_V = 150000;
	const int MODULUS = 1000003;
	const double LF_INF = 100000.0;
	std::vector<Node> nodes;
	std::vector<string> routes;
	std::vector<HashNode> nodemap[MODULUS];
	std::map<string, int> routemap;
	std::vector<Edge> g[MAX_V];

	typedef std::pair<double, int> P;//first 最短距离, second 顶点编号

	int pre[MAX_V];
	double d[MAX_V];

	string Path(int dep, int arr)
	{
		std::vector<int> path;
		int pos = arr;
		for( ; ;)
		{
			path.push_back(pos);
			if(pos == dep)
				break;
			pos = pre[pos];
		}
		std::vector<int>::reverse_iterator it;
		string ReadableRoute;
		string lastRoute = "";
		double totalDist = 0;
		std::vector<string> wpt;
		std::vector<string> rte;
		for(it = path.rbegin(); it != path.rend(); ++it)
		{
			std::vector<Edge>::iterator ite;
			for(ite = g[*it].begin(); it != path.rend() - 1 && ite != g[*it].end(); ++ite)
			{
				if(ite->to == *(it + 1))
				{
					totalDist += ite->dist;
					printf("%s->%s\t%s\t%lf\n", nodes[*it].name, nodes[ite->to].name, routes[ite->way].c_str(), totalDist);
					if(!Bravo::StringEquals(routes[ite->way].c_str(), lastRoute.c_str()))
					{
						wpt.push_back(nodes[*it].name);
						rte.push_back(routes[ite->way]);
					}
					lastRoute = routes[ite->way];
					break;
				}
			}
		}
		std::vector<string>::iterator it1, it2;
		it1 = wpt.begin();
		it2 = rte.begin();
		for(; it1 != wpt.end(); ++it1, ++it2)
		{
			ReadableRoute += *it1 + " ";
			ReadableRoute += *it2 + " ";
		}
		ReadableRoute += nodes[arr].name;
		return ReadableRoute;
	}
	string FindRoute(int dep, int arr)
	{
		std::priority_queue<P, std::vector<P>, std::greater<P> > queue;
		for(int i = 0; i != MAX_V; ++i)
	        d[i] = LF_INF;
		d[dep] = 0;
		queue.push(P(0.0, dep));
		while(!queue.empty())
		{
			P p = queue.top();
			queue.pop();
			int v = p.second;
			if(d[v] < p.first)
				continue;
			std::vector<Edge>::iterator it;
			for(it = g[v].begin(); it != g[v].end(); ++it)
				if(d[it->to] > d[v] + it->dist)
				{
					pre[it->to] = v;
					d[it->to] = d[v] + it->dist;
					queue.push(P(d[it->to], it->to));
				}
		}
		return Path(dep, arr);
	}
}
