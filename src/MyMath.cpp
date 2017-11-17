#include <cmath>
#include "MyMath.h"
using namespace std;

namespace MyMath
{
	static void swap(pair<float, pair<int, int> >& a, pair<float, pair<int, int> >& b)
	{
		pair<float, pair<int, int> > temp = a;
		a = b;
		b = temp;
	}

	void crossProduct(float *a, float *b, float *n)
	{
		n[0] = a[1]*b[2]-b[1]*a[2];
		n[1] = a[2]*b[0]-b[2]*a[0];
		n[2] = a[0]*b[1]-b[0]*a[1];
	}

	void normalize(float *v)
	{
		float normSqrt = sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
		v[0] /= normSqrt;
		v[1] /= normSqrt;
		v[2] /= normSqrt;
	}

	void change_elem(vector<pair<float, pair<int, int> > >& heap, int index, pair<float, pair<int, int> >& elem, map<pair<int, int>, int>& order)
	{
		order.erase(heap[index].second);
		heap[index] = elem;
		order[heap[index].second] = index;
		int temp = index;
		while (index)
		{
			pair<float, pair<int, int> >& temp_index = heap[index];
			pair<float, pair<int, int> >& temp_index2 = heap[(index-1)/2];
			if (temp_index < temp_index2)
			{
				order[temp_index.second] = (index-1)/2;
				order[temp_index2.second] = index;
				swap(temp_index, temp_index2);
				index = (index-1)/2;
			}
			else
				break;
		}
		index = temp;
		while (index*2+1 < heap.size())
		{
			pair<float, pair<int, int> >& temp_index = heap[index];
			pair<float, pair<int, int> >& temp_index2 = heap[index*2+1];
			if (temp_index2 < temp_index)
				if (index*2+2 < heap.size() && heap[index*2+2] < temp_index)
				{
					pair<float, pair<int, int> >& temp_index3 = heap[index*2+2];
					if (temp_index2 < temp_index3)
					{
						order[temp_index.second] = index*2+1;
						order[temp_index2.second] = index;
						swap(temp_index, temp_index2);
						index = index*2+1;
					}
					else
					{
						order[temp_index.second] = index*2+2;
						order[temp_index3.second] = index;
						swap(temp_index, temp_index3);
						index = index*2+2;
					}
				}
				else
				{
					order[temp_index.second] = index*2+1;
					order[temp_index2.second] = index;
					swap(temp_index, temp_index2);
					index = index*2+1;
				}
			else
				break;
		}
	}

	void del_elem(vector<pair<float, pair<int, int> > >& heap, pair<int, int> tgt, map<pair<int, int>, int>& order)
	{
		if (order.find(tgt) == order.end())
			return;
		int index = order[tgt];
		pair<float, pair<int, int> > last = heap[heap.size()-1];
		heap.pop_back();
		if (index == heap.size())
			order.erase(last.second);
		else
			change_elem(heap, index, last, order);
	}
}