#ifndef LIBRARY_DATASET_HPP
#define LIBRARY_DATASET_HPP

#include <string>

namespace bf
{
	using std::string;

	class Graph;

	class InternalStruct;

	class DataSet
	{
	private:
		string path;
		Graph *graph;
		InternalStruct *is;

		void InitializeFixes(void);

		void InitializeRoutes(void);

		void InitializeAirports(void);

	public:
		DataSet(void);

		~DataSet(void);

		void SetDataPath(const string &s);

		void Initialize(void);

		void InitializeAirport(string s);

		void Dijkstra(const string &i, const string &j);
	};
}
#endif //LIBRARY_DATASET_HPP
