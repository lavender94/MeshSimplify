#include <memory.h>
#include "Matrix.h"
using namespace std;

namespace MyMath
{
	Matrix::Matrix(int line, int row, const float* element /* = 0 */) : Line(line), Row(row)
	{
		Element = new float[Line*Row];
		if (element == 0)
			memset(Element, 0, sizeof(float)*Line*Row);
		else
			memcpy(Element, element, sizeof(float)*Line*Row);
	}

	Matrix::Matrix(Matrix& v): Line(v.Line), Row(v.Row)
	{
		Element = new float[Line*Row];
		memcpy(Element, v.Element, sizeof(float)*Line*Row);
	}

	Matrix::~Matrix()
	{
		if (Element)
			delete[] Element;
	}

	//Operaters
	Matrix& Matrix::operator= (const Matrix& v)
	{
		if (!(Line == v.Line && Row == v.Row))
		{
			delete[] Element;
			Line = v.Line;
			Row = v.Row;
			Element = new float[Line*Row];
		}
		memcpy(Element, v.Element, sizeof(float)*Line*Row);
		return *this;
	}

	void Matrix::operator+=(const Matrix& v)
	{
		assert(Line == v.Line && Row == v.Row);
		float *const &_elem = v.Element;
		for (int i=0, s=Line*Row; i<s; ++i)
			Element[i] += _elem[i];
	}

	void Matrix::operator-=(const Matrix& v)
	{
		assert(Line == v.Line && Row == v.Row);
		float *const &_elem = v.Element;
		for (int i=0, s=Line*Row; i<s; ++i)
			Element[i] -= _elem[i];
	}

	Matrix Matrix::operator +(const Matrix& v) const
	{
		assert(Line == v.Line && Row == v.Row);
		Matrix res(Line, Row);
		float *&res_elem = res.Element;
		float *const &v_elem = v.Element;
		for (int i=0, s=Line*Row; i<s; ++i)
			res_elem[i] = Element[i] + v_elem[i];
		return res;
	}

	Matrix Matrix::operator *(const Matrix& v) const
	{
		assert(Row == v.Line);
		Matrix res(Line, v.Row);
		float *&res_elem = res.Element;
		float *const &v_elem = v.Element;
		for (int i=0, index = 0; i<Line; ++i)
			for (int j=0; j<v.Row; ++j, ++index)
				for (int k=0; k<Row; ++k)
					res_elem[index] += Element[i*Row+k] * v_elem[k*v.Row+j];
		return res;
	}

	//Tools
	Matrix Matrix::transposition()
	{
		Matrix res(Row, Line);
		float *&res_elem = res.Element;
		for (int i=0; i<Line; ++i)
			for (int j=0; j<Row; ++j)
				res_elem[j*Line+i] = Element[i*Row+j];
		return res;
	}

	void Matrix::print()
	{
		for (int i=0; i<Line; ++i)
		{
			for (int j=0; j<Row; ++j)
				printf("%f ", Element[i*Row+j]);
			printf("\n");
		}
	}
}
