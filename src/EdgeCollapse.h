#pragma once
#include <set>
#include <vector>
#include <map>
#include "SimpleObject.h"
#include "Matrix.h"

class EdgeCollapse : public SimpleOBJ::CSimpleObject
{
public:
	EdgeCollapse(): t(0.01f), v_Q(0), pairs(0), faces(0), projection(0) {}
	~EdgeCollapse();
	void simplify(float);
	void Destroy();

	float t;//æ‡¿Î„–÷µ

private:
	float countDelta(int, int);
	MyMath::Matrix countFaceMatrix(int);
	void initialize();
	inline bool belong(int point, int face);
	void _simplify();
	class Comparer
	{
	public:
		bool operator() (std::pair<SimpleOBJ::Vec3f, int>& a, std::pair<SimpleOBJ::Vec3f, int>& b)
		{
			if (a.first.L2Norm_Sqr() < b.first.L2Norm_Sqr())
				return true;
			else if (a.first.L2Norm_Sqr() == b.first.L2Norm_Sqr())
				return a.second < b.second;
			else return false;
		}
	} cmp;

	std::set<int> *pairs, *faces;
	std::vector<std::pair<float, std::pair<int, int> > > heap;
	std::map<std::pair<int, int>, int> order;

	MyMath::Matrix *v_Q;

	std::set<int> deletedPoints, deletedFaces;
	int *projection;
};

