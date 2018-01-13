#ifndef LIBRARY_DATASET_HPP
#define LIBRARY_DATASET_HPP

#include <string>
#include <vector>
#include "Leg.hpp"

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

		void InitializeAirport(string s);

		void InitializeAirports(void);

	public:
		DataSet(void);

		~DataSet(void);

		void SetDataPath(const string &s);

		void Initialize(void);

		string GenerateRouteString(const std::vector<Leg> &legs);

		std::vector<Leg> FindDetailedRoute(const string &depature, const string &arrival);
	};
}
#endif //LIBRARY_DATASET_HPP
