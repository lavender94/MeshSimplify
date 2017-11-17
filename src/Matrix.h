#pragma once
#include <stdio.h>
#include <assert.h>

namespace MyMath
{
	class Matrix
	{
	public:
		//Constructors
		Matrix(int line = 4, int row = 4, const float* element = 0);
		Matrix(Matrix&);
		~Matrix();

		int Line, Row;
		float *Element;

		//Operators
		Matrix& operator = (const Matrix& v);

		//Operator []
		__forceinline float* operator [](int index)
		{
			assert(Element&&index>=0&&index<Line);
			return &Element[index*Row];
		}
		__forceinline const float* operator [](int index) const
		{
			assert(Element&&index>=0&&index<Line);
			return &Element[index*Row];
		}

 		//Operators +=,-=, *=, /=
 		void operator +=(const Matrix& v);
// 		void operator +=(float f);
 		void operator -=(const Matrix& v);
// 		void operator -=(float f);
// 		void operator *=(const Matrix& v);
// 		void operator *=(float f);
// 		void operator /=(const Matrix& v);
// 		void operator /=(float f);

 		//Operators +,-.*,/
 		Matrix operator +(const Matrix&v) const;
// 		Matrix operator +(float f) const;
// 		Matrix operator -(const Matrix&v) const;
// 		Matrix operator -(float f) const;
 		Matrix operator *(const Matrix&v) const;
// 		Matrix operator *(float f) const;
// 		Matrix operator /(float f) const;
// 
// 		Matrix operator -() const;

		//Tools
		Matrix transposition();
		void print();
	};
}
