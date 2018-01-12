#ifndef LIBRARY_DATASET_HPP
#define LIBRARY_DATASET_HPP

#include <string>

namespace bf
{
	class Graph;
	class InternalStruct;

	class DataSet
	{
	private:
		std::string path;
		Graph *graph;
		InternalStruct *is;

		void InitializeFixes(void);

		void InitializeRoutes(void);

		void InitializeAirports(void);

	public:
		DataSet(void);

		~DataSet(void);

		void SetDataPath(const std::string &s);

		void Initialize(void);

		void InitializeAirport(std::string s);
	};
}
#endif //LIBRARY_DATASET_HPP
