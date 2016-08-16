#include <cstdio>
namespace Bravo
{
	bool IsFileExists(const char *file)
	{
		FILE *fp = fopen(file, "rb");
		if(NULL == fp)
			return false;
		else
		{
			fclose(fp);
			return true;
		}
	}
}
