#include <algorithm>
#include <iostream>
#include <assert.h>
#include <time.h>
#include "MyMath.h"
#include "EdgeCollapse.h"
#include "Vec3f.h"
using namespace std;

EdgeCollapse::~EdgeCollapse()
{
	Destroy();
}

void EdgeCollapse::Destroy()
{
	if (pairs)
	{
		delete[] pairs;
		pairs = 0;
	}
	if (faces)
	{
		delete[] faces;
		faces = 0;
	}
	if (v_Q)
	{
		delete[] v_Q;
		v_Q = 0;
	}
	if (projection)
	{
		delete[] projection;
		projection = 0;
	}

	SimpleOBJ::CSimpleObject::Destroy();
}

float EdgeCollapse::countDelta(int a, int b)//index:a b
{
	SimpleOBJ::Vec3f v = (m_pVertexList[a] + m_pVertexList[b])/2;
	float temp[4];
	for (int i=0; i<3; i++)
		temp[i] = v[i];
	temp[3] = 1;
	MyMath::Matrix m_v(1,4, temp);
	return (m_v * (v_Q[a] + v_Q[b]) * m_v.transposition()).Element[0];
}

MyMath::Matrix EdgeCollapse::countFaceMatrix(int index)
{
	SimpleOBJ::Vec3f v[3], n;
	for (int j=0; j<3; ++j)
		v[j] = m_pVertexList[m_pTriangleList[index][j]];
	MyMath::crossProduct((v[1]-v[0])._p, (v[2]-v[1])._p, n._p);
	n.Normalize();
	SimpleOBJ::Vec3f _d = -n*v[0];
	float face[4];
	face[3] = 0;
	for (int i=0; i<3; ++i)
	{
		face[i] = n[i];
		face[3] += _d[i];
	}
	MyMath::Matrix p(1,4,face);
	return p.transposition()*p;
}

void EdgeCollapse::initialize()
{
	assert(IsLoaded());
	t = t*t;
	set<pair<int, int> > v_edge_pairs, v_noedge_pairs;	
	projection = new int[m_nVertices];
	v_Q = new MyMath::Matrix[m_nVertices];
	pairs = new set<int>[m_nVertices];
	faces = new set<int>[m_nVertices];
	//遍历每个面
	for (int i=0; i<m_nTriangles; ++i)
	{
		//计算K方阵
		MyMath::Matrix temp = countFaceMatrix(i);
		for (int j=0; j<3; ++j)
			v_Q[m_pTriangleList[i][j]] += temp;

		//获取边连接的Pair
		if (m_pTriangleList[i][0] < m_pTriangleList[i][1]) v_edge_pairs.insert(pair<int, int>(m_pTriangleList[i][0], m_pTriangleList[i][1]));
		else v_edge_pairs.insert(pair<int, int>(m_pTriangleList[i][1], m_pTriangleList[i][0]));
		if (m_pTriangleList[i][1] < m_pTriangleList[i][2]) v_edge_pairs.insert(pair<int, int>(m_pTriangleList[i][1], m_pTriangleList[i][2]));
		else v_edge_pairs.insert(pair<int, int>(m_pTriangleList[i][2], m_pTriangleList[i][1]));
		if (m_pTriangleList[i][0] < m_pTriangleList[i][2]) v_edge_pairs.insert(pair<int, int>(m_pTriangleList[i][0], m_pTriangleList[i][2]));
		else v_edge_pairs.insert(pair<int, int>(m_pTriangleList[i][2], m_pTriangleList[i][0]));
		
		//更新与点关联的面的信息
		faces[m_pTriangleList[i][0]].insert(i);
		faces[m_pTriangleList[i][1]].insert(i);
		faces[m_pTriangleList[i][2]].insert(i);
	}

	//获取距离小于阈值的无边pair
// 	for (int i=0; i<m_nVertices; ++i)
// 		for (int j=i+1; j<m_nVertices; ++j)
// 			if ((m_pVertexList[i] - m_pVertexList[j]).L2Norm_Sqr() < t)
// 				if (v_edge_pairs.find(pair<int, int>(i, j)) == v_edge_pairs.end())
// 					v_noedge_pairs.insert(pair<int, int>(i,j));
	pair<SimpleOBJ::Vec3f, int> *_m_pVertexList = new pair<SimpleOBJ::Vec3f, int>[m_nVertices];
	for (int i=0; i<m_nVertices; ++i)
		_m_pVertexList[i] = pair<SimpleOBJ::Vec3f, int>(m_pVertexList[i], i);
	sort(_m_pVertexList, _m_pVertexList+m_nVertices, cmp);
	for (int i=0; i<m_nVertices; ++i)
		for (int j=i+1; j<m_nVertices; ++j)
			if ((_m_pVertexList[i].first - _m_pVertexList[j].first).L2Norm_Sqr() < t)
			{
				int _i, _j;
				if (_m_pVertexList[i].second < _m_pVertexList[j].second)
				{
					_i = _m_pVertexList[i].second;
					_j = _m_pVertexList[j].second;
				}
				else
				{
					_i = _m_pVertexList[j].second;
					_j = _m_pVertexList[i].second;
				}
				if (v_edge_pairs.find(pair<int, int>(_i, _j)) == v_edge_pairs.end())
					v_noedge_pairs.insert(pair<int, int>(_i,_j));
			}
			else
				if ((_m_pVertexList[j].first.L2Norm_Sqr() - _m_pVertexList[i].first.L2Norm_Sqr()) >= t)
					break;
	delete[] _m_pVertexList;

	//初始化堆
	for (set<pair<int, int> >::iterator iter = v_edge_pairs.begin(); iter != v_edge_pairs.end(); ++iter)
	{
		heap.push_back(pair<float, pair<int, int> >(countDelta(iter->first, iter->second), *iter));
		pairs[iter->first].insert(iter->second);
		pairs[iter->second].insert(iter->first);
	}
	for (set<pair<int, int> >::iterator iter = v_noedge_pairs.begin(); iter != v_noedge_pairs.end(); ++iter)
	{
		heap.push_back(pair<float, pair<int, int> >(countDelta(iter->first, iter->second), *iter));
		pairs[iter->first].insert(iter->second);
		pairs[iter->second].insert(iter->first);
	}

	make_heap(heap.begin(), heap.end(), greater<pair<float, pair<int, int> > >());
	for (int i=0; i<heap.size(); i++)
		order[heap[i].second] = i;
}

inline bool EdgeCollapse::belong(int point, int face)
{
	if (m_pTriangleList[face][0] == m_pTriangleList[face][1] || m_pTriangleList[face][0] == m_pTriangleList[face][2] || m_pTriangleList[face][1] == m_pTriangleList[face][2]) return false;
	for (int i=0; i<3; ++i)
		if (m_pTriangleList[face][i] == point)
			return true;
	return false;
}

void EdgeCollapse::_simplify()
{
	if (heap.empty()) return;
	int n_v1, n_v2;
	n_v1 = heap[0].second.first;
	n_v2 = heap[0].second.second;
	SimpleOBJ::Vec3f v = (m_pVertexList[n_v1] + m_pVertexList[n_v2])/2;
	v_Q[n_v1] = MyMath::Matrix(4,4);
	set<int> changePoints;
	changePoints.insert(n_v1);
	changePoints.insert(n_v2);

	//维护点的Q矩阵及面信息
	//遍历与n_v1关联的面
	set<int> v1_deletedFaces;
	for (set<int>::iterator iter = faces[n_v1].begin(); iter != faces[n_v1].end(); ++iter)
	{
		int i = *iter;
		if (belong(n_v2, i))
		{
			int temp;
			for (int j=0; j<3; ++j)
				if ((temp = m_pTriangleList[i][j]) != n_v1 && m_pTriangleList[i][j] != n_v2)
					break;
			v_Q[temp] -= countFaceMatrix(i);
			changePoints.insert(temp);
			faces[temp].erase(i);
			for (int j=0; j<3; ++j)
				if (m_pTriangleList[i][j] == n_v2)
					m_pTriangleList[i][j] = n_v1;
			deletedFaces.insert(i);
			v1_deletedFaces.insert(i);
			faces[n_v2].erase(i);
		}
		else
		{
			MyMath::Matrix temp0 = countFaceMatrix(i);
			SimpleOBJ::Vec3f v1 = m_pVertexList[n_v1];
			m_pVertexList[n_v1] = v;
			MyMath::Matrix temp1 = countFaceMatrix(i);
			m_pVertexList[n_v1] = v1;
			for (int j=0, k=0; j<3; ++j)
				if ((k = m_pTriangleList[i][j]) != n_v1)
				{
					v_Q[k] -= temp0;
					v_Q[k] += temp1;
					changePoints.insert(k);
				}
				v_Q[n_v1] += temp1;
		}
	}
	//取出与n_v1关联的被删除面
	for (set<int>::iterator iter = v1_deletedFaces.begin(); iter != v1_deletedFaces.end(); ++iter)
		faces[n_v1].erase(*iter);
	v1_deletedFaces.clear();
	//遍历与n_v2关联的面
	for (set<int>::iterator iter = faces[n_v2].begin(); iter != faces[n_v2].end(); ++iter)
	{
		int i = *iter;
		MyMath::Matrix temp0 = countFaceMatrix(i);
		SimpleOBJ::Vec3f v2 = m_pVertexList[n_v2];
		m_pVertexList[n_v2] = v;
		MyMath::Matrix temp1 = countFaceMatrix(i);
		m_pVertexList[n_v2] = v2;
		for (int j=0, k=0; j<3; ++j)
			if ((k = m_pTriangleList[i][j]) != n_v2)
			{
				v_Q[k] -= temp0;
				v_Q[k] += temp1;
				changePoints.insert(k);
			}
			else
				m_pTriangleList[i][j] = n_v1;
		v_Q[n_v1] += temp1;
	}
	faces[n_v1].insert(faces[n_v2].begin(), faces[n_v2].end());
	faces[n_v2].clear();

	//更新v1坐标
	m_pVertexList[n_v1] = v;

	//更新堆
	//删除(v1,v2)
	MyMath::del_elem(heap, heap[0].second, order);
	pairs[n_v1].erase(n_v2);
	pairs[n_v2].erase(n_v1);
	deletedPoints.insert(n_v2);
	projection[n_v2] = n_v1;

	//更新其它pair
	for (set<int>::iterator i = changePoints.begin(); i != changePoints.end(); ++i)
		for (set<int>::iterator j = pairs[*i].begin(); j != pairs[*i].end(); ++j)
		{
			if (*j < *i && changePoints.find(*j) != changePoints.end()) continue;
			int tempi, tempj;
			if (*i < *j)
			{
				tempi = *i;
				tempj = *j;
			}
			else
			{
				tempi = *j;
				tempj = *i;
			}
			if (tempi == n_v2)
			{
				int _tempi, _tempj;
				if (n_v1 < tempj)
				{
					_tempi = n_v1;
					_tempj = tempj;
				}
				else
				{
					_tempi = tempj;
					_tempj = n_v1;
				}
				map<pair<int, int>, int>::iterator tempIter = order.find(pair<int, int>(_tempi, _tempj));
				if (tempIter != order.end())
					MyMath::del_elem(heap, pair<int, int>(n_v2, tempj), order);
				else
					MyMath::change_elem(heap, order[pair<int, int>(n_v2, tempj)], pair<float, pair<int, int> >(countDelta(_tempi, _tempj), pair<int, int>(_tempi, _tempj)), order);
			}
			else if (tempj == n_v2)
			{
				int _tempi, _tempj;
				if (n_v1 < tempi)
				{
					_tempi = n_v1;
					_tempj = tempi;
				}
				else
				{
					_tempi = tempi;
					_tempj = n_v1;
				}
				map<pair<int, int>, int>::iterator tempIter = order.find(pair<int, int>(_tempi, _tempj));
				if (tempIter != order.end())
					MyMath::del_elem(heap, pair<int, int>(tempi, n_v2), order);
				else
					MyMath::change_elem(heap, order[pair<int, int>(tempi, n_v2)], pair<float, pair<int, int> >(countDelta(_tempi, _tempj), pair<int, int>(_tempi, _tempj)), order);
			}
			else
				MyMath::change_elem(heap, order[pair<int, int>(tempi, tempj)], pair<float, pair<int, int> >(countDelta(tempi, tempj), pair<int, int>(tempi, tempj)), order);
		}
	for (set<int>::iterator iter = pairs[n_v2].begin(); iter != pairs[n_v2].end(); ++iter)
	{
		pairs[*iter].erase(n_v2);
		pairs[*iter].insert(n_v1);
	}
	pairs[n_v1].insert(pairs[n_v2].begin(), pairs[n_v2].end());
	pairs[n_v2].clear();
}

void EdgeCollapse::simplify(float percentage)
{
	if (!IsLoaded()) return;
	printf("Initializing...	");
	initialize();
	printf("End\n");
	int deleteFaceNumber = m_nTriangles - (int)(m_nTriangles * percentage);
	printf("Simplify percentage:%f\nMinium deleting number:%d\nSimplifying...\nDeleted face number:\n", percentage, deleteFaceNumber);
	int printControl = 1;
	clock_t time = clock();
	printf("0");
	while (deletedFaces.size() < deleteFaceNumber)
	{
		_simplify();
		if (printControl * 100 <= deletedFaces.size())
			printf("\r%d00 Speed:%.2lf/s", printControl++, deletedFaces.size()/(double)(clock()-time)*CLOCKS_PER_SEC);
	}
	time = clock()-time;
	printf("\r%d Speed:%.2lf/s Time in total:%.2lf seconds\nRelease...	", deletedFaces.size(), deletedFaces.size()/(double)time*CLOCKS_PER_SEC, (double)time/CLOCKS_PER_SEC);

	//release
	heap.clear();
	order.clear();
	delete[] pairs;
	pairs = 0;
	delete[] faces;
	faces = 0;
	delete[] v_Q;
	v_Q = 0;

	printf("End\nConverting...	");
	//更新点列表和片列表
	SimpleOBJ::Vec3f *_m_pVertexList = new SimpleOBJ::Vec3f[m_nVertices-deletedPoints.size()];
	SimpleOBJ::Array<int,3> *_m_pTriangleList = new SimpleOBJ::Array<int,3>[m_nTriangles-deletedFaces.size()];
	for (int i=0, k=0; i<m_nVertices; ++i)
		if (deletedPoints.find(i) == deletedPoints.end())
		{
			projection[i] = k;
			_m_pVertexList[k++] = m_pVertexList[i];
		}
	for (set<int>::iterator iter = deletedPoints.begin(); iter != deletedPoints.end(); ++iter)
	{
		while (deletedPoints.find(projection[*iter]) != deletedPoints.end())
			projection[*iter] = projection[projection[*iter]];
		projection[*iter] = projection[projection[*iter]];
	}
	for (int i=0, k=0; i<m_nTriangles; ++i)
		if (deletedFaces.find(i) == deletedFaces.end())
		{
			_m_pTriangleList[k] = m_pTriangleList[i];
			_m_pTriangleList[k][0] = projection[_m_pTriangleList[k][0]];
			_m_pTriangleList[k][1] = projection[_m_pTriangleList[k][1]];
			_m_pTriangleList[k][2] = projection[_m_pTriangleList[k][2]];
			++k;
		}
	delete[] m_pVertexList;
	delete[] m_pTriangleList;
	m_pVertexList = _m_pVertexList;
	m_pTriangleList = _m_pTriangleList;
	m_nVertices -= deletedPoints.size();
	m_nTriangles -= deletedFaces.size();

	//release
	deletedPoints.clear();
	deletedFaces.clear();
	delete[] projection;
	projection = 0;
	printf("End\n");
}