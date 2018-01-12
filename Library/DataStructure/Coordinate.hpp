#ifndef LIBRARY_COORDINATE_HPP
#define LIBRARY_COORDINATE_HPP


namespace bf
{
	class Coordinate
	{
	public:
		float latitude;
		float longitude;

		Coordinate(void);

		Coordinate(float latitude, float longitude);

		float DistanceFrom(Coordinate x);
	};
}


#endif //LIBRARY_COORDINATE_HPP
