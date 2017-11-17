#include <stdio.h>
#include <cstdlib>
#include "Matrix.h"
#include "EdgeCollapse.h"
using namespace std;

int main(int argv, char **argc)
{
	if (argv < 4)
	{
		printf("Error input format!\n");
		return 0;
	}
	EdgeCollapse application;
	application.LoadFromObj(argc[1]);
	if (argv > 4)
		application.t = atof(argc[4]);
	else
		application.t = 0.01f;
	application.simplify(atof(argc[3]));
	application.SaveToObj(argc[2]);
	return 0;
}