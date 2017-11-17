#pragma once
#include <vector>
#include <map>

namespace MyMath
{
	void crossProduct(float*, float*, float*);
	void normalize(float*);
	void change_elem(std::vector<std::pair<float, std::pair<int, int> > >& heap, int index, std::pair<float, std::pair<int, int> >& elem, std::map<std::pair<int, int>, int>& order);
	void del_elem(std::vector<std::pair<float, std::pair<int, int> > >& heap, std::pair<int, int> tgt, std::map<std::pair<int, int>, int>& order);
}