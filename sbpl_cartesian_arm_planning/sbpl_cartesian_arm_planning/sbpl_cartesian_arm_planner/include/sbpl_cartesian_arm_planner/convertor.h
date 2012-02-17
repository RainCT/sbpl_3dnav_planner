#include <stdio.h>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <limits.h>

#ifndef __cplusplus
#error Must use C++ for the type matrix.
#endif

namespace std {

template <class T>
class matrix
{
public:
   // Constructors
   matrix (const matrix<T>& m);
   matrix (size_t row = 6, size_t col = 6);

   // Destructor
   ~matrix ();

   // Assignment operators
   matrix<T>& operator = (const matrix<T>& m);

   // Value extraction method
   size_t RowNo () const { return _m->Row; }
   size_t ColNo () const { return _m->Col; }

   // Subscript operator
   T& operator () (size_t row, size_t col);
   T  operator () (size_t row, size_t col) const;

   // Unary operators
   matrix<T> operator + ()  { return *this; }
   matrix<T> operator - () ;

   // Combined assignment - calculation operators
   matrix<T>& operator += (const matrix<T>& m) ;
   matrix<T>& operator -= (const matrix<T>& m) ;
   matrix<T>& operator *= (const matrix<T>& m) ;
   matrix<T>& operator *= (const T& c) ;
   matrix<T>& operator /= (const T& c) ;
   matrix<T>& operator ^= (const size_t& pow) ;

   // Miscellaneous -methods
   void Null (const size_t& row, const size_t& col);
   void Null ();
   void Unit (const size_t& row) ;
   void Unit () ;
   void SetSize (size_t row, size_t col) ;

   // Utility methods
   matrix<T> Solve (const matrix<T>& v) const;
   matrix<T> Adj ();
   matrix<T> Inv ();
   T Det () const;
   T Norm ();
   T Cofact (size_t row, size_t col) ;
   T Cond ();

   // Type of matrices
   bool IsSquare ()  { return (_m->Row == _m->Col); } 
   bool IsSingular () ;
   bool IsDiagonal () ;
   bool IsScalar () ;
   bool IsUnit () ;
   bool IsNull () ;
   bool IsSymmetric () ;
   bool IsSkewSymmetric () ;
   bool IsUpperTriangular () ;
   bool IsLowerTriangular () ;

private:
    struct base_mat
    {
	T **Val;
	size_t Row, Col, RowSiz, ColSiz;
	int Refcnt;

	base_mat (size_t row, size_t col, T** v)
	{
	    Row = row; RowSiz = row;
	    Col = col; ColSiz = col;
	    Refcnt = 1;

	    Val = new T* [row];
	    size_t rowlen = col * sizeof(T);

	    for (size_t i=0; i < row; i++)
	    {
		Val[i] = new T [col];
		if (v) memcpy( Val[i], v[i], rowlen);
	    }
	}
	~base_mat ()
	{
	    for (size_t i=0; i < RowSiz; i++)
		delete [] Val[i];
	    delete [] Val;
	}
    };
    base_mat *_m;

    void clone ();
    void realloc (size_t row, size_t col);
    int pivot (size_t row);
};

// constructor
template <class T> inline
matrix<T>::matrix (size_t row, size_t col)
{
  _m = new base_mat( row, col, 0);
}

// copy constructor
template <class T> inline
matrix<T>::matrix (const matrix<T>& m)
{
    _m = m._m;
    _m->Refcnt++;
}

// Internal copy constructor
template <class T> inline void
matrix<T>::clone ()
{
    _m->Refcnt--;
    _m = new base_mat( _m->Row, _m->Col, _m->Val);
}

// destructor
template <class T> inline
matrix<T>::~matrix ()
{
   if (--_m->Refcnt == 0) delete _m;
}

// assignment operator
template <class T> inline matrix<T>&
matrix<T>::operator = (const matrix<T>& m) 
{
    m._m->Refcnt++;
    if (--_m->Refcnt == 0) delete _m;
    _m = m._m;
    return *this;
}

//  reallocation method
template <class T> inline void 
matrix<T>::realloc (size_t row, size_t col)
{
   if (row == _m->RowSiz && col == _m->ColSiz)
   {
      _m->Row = _m->RowSiz;
      _m->Col = _m->ColSiz;
      return;
   }

   base_mat *m1 = new base_mat( row, col, NULL);
   size_t colSize = min(_m->Col,col) * sizeof(T);
   size_t minRow = min(_m->Row,row);

   for (size_t i=0; i < minRow; i++)
      memcpy( m1->Val[i], _m->Val[i], colSize);

   if (--_m->Refcnt == 0) 
       delete _m;
   _m = m1;

   return;
}

// public method for resizing matrix
template <class T> inline void
matrix<T>::SetSize (size_t row, size_t col) 
{
   size_t i,j;
   size_t oldRow = _m->Row;
   size_t oldCol = _m->Col;

   if (row != _m->RowSiz || col != _m->ColSiz)
      realloc( row, col);

   for (i=oldRow; i < row; i++)
      for (j=0; j < col; j++)
	 _m->Val[i][j] = T(0);

   for (i=0; i < row; i++)                      
      for (j=oldCol; j < col; j++)
	 _m->Val[i][j] = T(0);

   return;
}

// subscript operator to get/set individual elements
template <class T> inline T&
matrix<T>::operator () (size_t row, size_t col) 
{
   if (row >= _m->Row || col >= _m->Col)
      printf( "matrix<T>::operator(): Index out of range!");
   if (_m->Refcnt > 1) clone();
   return _m->Val[row][col];
}

// subscript operator to get/set individual elements
template <class T> inline T
matrix<T>::operator () (size_t row, size_t col) const
{
   if (row >= _m->Row || col >= _m->Col)
      printf( "matrix<T>::operator(): Index out of range!");
   return _m->Val[row][col];
}

// logical equal-to operator
template <class T> inline bool
operator == (const matrix<T>& m1, const matrix<T>& m2) 
{
   if (m1.RowNo() != m2.RowNo() || m1.ColNo() != m2.ColNo())
      return false;

   for (size_t i=0; i < m1.RowNo(); i++)
      for (size_t j=0; j < m1.ColNo(); j++)
	      if (m1(i,j) != m2(i,j))
	         return false;

   return true;
}

// logical no-equal-to operator
template <class T> inline bool
operator != (const matrix<T>& m1, const matrix<T>& m2) 
{
    return (m1 == m2) ? false : true;
}

// combined addition and assignment operator
template <class T> inline matrix<T>&
matrix<T>::operator += (const matrix<T>& m) 
{
   if (_m->Row != m._m->Row || _m->Col != m._m->Col)
      printf( "matrix<T>::operator+= : Inconsistent matrix sizes in addition!");
   if (_m->Refcnt > 1) clone();
   for (size_t i=0; i < m._m->Row; i++)
      for (size_t j=0; j < m._m->Col; j++)
	 _m->Val[i][j] += m._m->Val[i][j];
   return *this;
}

// combined subtraction and assignment operator
template <class T> inline matrix<T>&
matrix<T>::operator -= (const matrix<T>& m) 
{
   if (_m->Row != m._m->Row || _m->Col != m._m->Col)
      printf( "matrix<T>::operator-= : Inconsistent matrix sizes in subtraction!");
   if (_m->Refcnt > 1) clone();
   for (size_t i=0; i < m._m->Row; i++)
      for (size_t j=0; j < m._m->Col; j++)
	 _m->Val[i][j] -= m._m->Val[i][j];
   return *this;
}

// combined scalar multiplication and assignment operator
template <class T> inline matrix<T>&
matrix<T>::operator *= (const T& c) 
{
    if (_m->Refcnt > 1) clone();
    for (size_t i=0; i < _m->Row; i++)
	for (size_t j=0; j < _m->Col; j++)
	    _m->Val[i][j] *= c;
    return *this;
}

// combined matrix multiplication and assignment operator
template <class T> inline matrix<T>&
matrix<T>::operator *= (const matrix<T>& m) 
{
   if (_m->Col != m._m->Row)
      printf( "matrix<T>::operator*= : Inconsistent matrix sizes in multiplication!");

   matrix<T> temp(_m->Row,m._m->Col);

   for (size_t i=0; i < _m->Row; i++)
      for (size_t j=0; j < m._m->Col; j++)
      {
         temp._m->Val[i][j] = T(0);
         for (size_t k=0; k < _m->Col; k++)
            temp._m->Val[i][j] += _m->Val[i][k] * m._m->Val[k][j];
      }
   *this = temp;

   return *this;
}

// combined scalar division and assignment operator
template <class T> inline matrix<T>&
matrix<T>::operator /= (const T& c) 
{
    if (_m->Refcnt > 1) clone();
    for (size_t i=0; i < _m->Row; i++)
	for (size_t j=0; j < _m->Col; j++)
	    _m->Val[i][j] /= c;

    return *this;
}

// combined power and assignment operator
template <class T> inline matrix<T>&
matrix<T>::operator ^= (const size_t& pow) 
{
	matrix<T> temp(*this);

	for (size_t i=2; i <= pow; i++)
      *this = *this * temp;

	return *this;
}

// unary negation operator
template <class T> inline matrix<T>
matrix<T>::operator - () 
{
   matrix<T> temp(_m->Row,_m->Col);

   for (size_t i=0; i < _m->Row; i++)
      for (size_t j=0; j < _m->Col; j++)
	 temp._m->Val[i][j] = - _m->Val[i][j];

   return temp;
}

// binary addition operator
template <class T> inline matrix<T>
operator + (const matrix<T>& m1, const matrix<T>& m2) 
{
   matrix<T> temp = m1;
   temp += m2;
   return temp;
}

// binary subtraction operator
template <class T> inline matrix<T>
operator - (const matrix<T>& m1, const matrix<T>& m2) 
{
   matrix<T> temp = m1;
   temp -= m2;
   return temp;
}

// binary scalar multiplication operator
template <class T> inline matrix<T>
operator * (const matrix<T>& m, const T& no) 
{
   matrix<T> temp = m;
   temp *= no;
   return temp;
}


// binary scalar multiplication operator
template <class T> inline matrix<T>
operator * (const T& no, const matrix<T>& m) 
{
   return (m * no);
}

// binary matrix multiplication operator
template <class T> inline matrix<T>
operator * (const matrix<T>& m1, const matrix<T>& m2) 
{
   matrix<T> temp = m1;
   temp *= m2;
   return temp;
}

// binary scalar division operator
template <class T> inline matrix<T>
operator / (const matrix<T>& m, const T& no) 
{
    return (m * (T(1) / no));
}


// binary scalar division operator
template <class T> inline matrix<T>
operator / (const T& no, const matrix<T>& m) 
{
    return (!m * no);
}

// binary matrix division operator
template <class T> inline matrix<T>
operator / (const matrix<T>& m1, const matrix<T>& m2) 
{
    return (m1 * !m2);
}

// binary power operator
template <class T> inline matrix<T>
operator ^ (const matrix<T>& m, const size_t& pow) 
{
   matrix<T> temp = m;
   temp ^= pow;
   return temp;
}

// unary transpose operator
template <class T> inline matrix<T>
operator ~ (const matrix<T>& m) 
{
   matrix<T> temp(m.ColNo(),m.RowNo());

   for (size_t i=0; i < m.RowNo(); i++)
      for (size_t j=0; j < m.ColNo(); j++)
      {
         T x = m(i,j);
	      temp(j,i) = x;
      }
   return temp;
}

// unary inversion operator
template <class T> inline matrix<T>
operator ! (const matrix<T> m) 
{
   matrix<T> temp = m;
   return temp.Inv();
}

// inversion function
template <class T> inline matrix<T>
matrix<T>::Inv () 
{
   size_t i,j,k;
   T a1,a2,*rowptr;

   if (_m->Row != _m->Col)
      printf( "matrix<T>::operator!: Inversion of a non-square matrix");

   matrix<T> temp(_m->Row,_m->Col);
   if (_m->Refcnt > 1) clone();


   temp.Unit();
   for (k=0; k < _m->Row; k++)
   {
      int indx = pivot(k);
      if (indx == -1)
	      printf( "matrix<T>::operator!: Inversion of a singular matrix");

      if (indx != 0)
      {
	      rowptr = temp._m->Val[k];
	      temp._m->Val[k] = temp._m->Val[indx];
	      temp._m->Val[indx] = rowptr;
      }
      a1 = _m->Val[k][k];
      for (j=0; j < _m->Row; j++)
      {
	      _m->Val[k][j] /= a1;
	      temp._m->Val[k][j] /= a1;
      }
      for (i=0; i < _m->Row; i++)
	   if (i != k)
	   {
	      a2 = _m->Val[i][k];
	      for (j=0; j < _m->Row; j++)
	      {
	         _m->Val[i][j] -= a2 * _m->Val[k][j];
	         temp._m->Val[i][j] -= a2 * temp._m->Val[k][j];
	      }
	   }
   }
   return temp;
}

// solve simultaneous equation
template <class T> inline matrix<T>
matrix<T>::Solve (const matrix<T>& v) const
{
   size_t i,j,k;
   T a1;

   if (!(_m->Row == _m->Col && _m->Col == v._m->Row)){}
      printf( "matrix<T>::Solve():Inconsistent matrices!");

   matrix<T> temp(_m->Row,_m->Col+v._m->Col);
   for (i=0; i < _m->Row; i++)
   {
      for (j=0; j < _m->Col; j++)
	 temp._m->Val[i][j] = _m->Val[i][j];
      for (k=0; k < v._m->Col; k++)
	 temp._m->Val[i][_m->Col+k] = v._m->Val[i][k];
   }
   for (k=0; k < _m->Row; k++)
   {
      int indx = temp.pivot(k);
      if (indx == -1){}
	 printf( "matrix<T>::Solve(): Singular matrix!");

      a1 = temp._m->Val[k][k];
      for (j=k; j < temp._m->Col; j++)
	 temp._m->Val[k][j] /= a1;

      for (i=k+1; i < _m->Row; i++)
      {
	 a1 = temp._m->Val[i][k];
	 for (j=k; j < temp._m->Col; j++)
	   temp._m->Val[i][j] -= a1 * temp._m->Val[k][j];
      }
   }
   matrix<T> s(v._m->Row,v._m->Col);
   for (k=0; k < v._m->Col; k++)
      for (int m=int(_m->Row)-1; m >= 0; m--)
      {
	 s._m->Val[m][k] = temp._m->Val[m][_m->Col+k];
	 for (j=m+1; j < _m->Col; j++)
	    s._m->Val[m][k] -= temp._m->Val[m][j] * s._m->Val[j][k];
      }
   return s;
}

// set zero to all elements of this matrix
template <class T> inline void
matrix<T>::Null (const size_t& row, const size_t& col) 
{
    if (row != _m->Row || col != _m->Col)
	realloc( row,col);

    if (_m->Refcnt > 1) 
	clone();

    for (size_t i=0; i < _m->Row; i++)
	for (size_t j=0; j < _m->Col; j++)
	    _m->Val[i][j] = T(0);
    return;
}

// set zero to all elements of this matrix
template <class T> inline void
matrix<T>::Null() 
{
    if (_m->Refcnt > 1) clone();   
    for (size_t i=0; i < _m->Row; i++)
	for (size_t j=0; j < _m->Col; j++)
		_m->Val[i][j] = T(0);
    return;
}

// set this matrix to unity
template <class T> inline void
matrix<T>::Unit (const size_t& row) 
{
    if (row != _m->Row || row != _m->Col)
	realloc( row, row);
	
    if (_m->Refcnt > 1) 
	clone();

    for (size_t i=0; i < _m->Row; i++)
	for (size_t j=0; j < _m->Col; j++)
	    _m->Val[i][j] = i == j ? T(1) : T(0);
    return;
}

// set this matrix to unity
template <class T> inline void
matrix<T>::Unit () 
{
    if (_m->Refcnt > 1) clone();   
    size_t row = min(_m->Row,_m->Col);
    _m->Row = _m->Col = row;

    for (size_t i=0; i < _m->Row; i++)
	for (size_t j=0; j < _m->Col; j++)
	    _m->Val[i][j] = i == j ? T(1) : T(0);
    return;
}

// private partial pivoting method
template <class T> inline int
matrix<T>::pivot (size_t row)
{
  int k = int(row);
  double amax,temp;

  amax = -1;
  for (size_t i=row; i < _m->Row; i++)
    if ( (temp = abs( _m->Val[i][row])) > amax && temp != 0.0)
     {
       amax = temp;
       k = i;
     }
  if (_m->Val[k][row] == T(0))
     return -1;
  if (k != int(row))
  {
     T* rowptr = _m->Val[k];
     _m->Val[k] = _m->Val[row];
     _m->Val[row] = rowptr;
     return k;
  }
  return 0;
}

// calculate the determinant of a matrix
template <class T> inline T
matrix<T>::Det () const
{
   size_t i,j,k;
   T piv,detVal = T(1);

   if (_m->Row != _m->Col){}
      printf( "matrix<T>::Det(): Determinant a non-square matrix!");
   
   matrix<T> temp(*this);
   if (temp._m->Refcnt > 1) temp.clone();

   for (k=0; k < _m->Row; k++)
   {
      int indx = temp.pivot(k);
      if (indx == -1)
	 return 0;
      if (indx != 0)
	 detVal = - detVal;
      detVal = detVal * temp._m->Val[k][k];
      for (i=k+1; i < _m->Row; i++)
      {
	 piv = temp._m->Val[i][k] / temp._m->Val[k][k];
	 for (j=k+1; j < _m->Row; j++)
	    temp._m->Val[i][j] -= piv * temp._m->Val[k][j];
      }
   }
   return detVal;
}

// calculate the norm of a matrix
template <class T> inline T
matrix<T>::Norm () 
{
   T retVal = T(0);

   for (size_t i=0; i < _m->Row; i++)
      for (size_t j=0; j < _m->Col; j++)
	 retVal += _m->Val[i][j] * _m->Val[i][j];
   retVal = sqrt( retVal);

   return retVal;
}

// calculate the condition number of a matrix
template <class T> inline T
matrix<T>::Cond () 
{
   matrix<T> inv = ! (*this);
   return (Norm() * inv.Norm());
}

// calculate the cofactor of a matrix for a given element
template <class T> inline T
matrix<T>::Cofact (size_t row, size_t col) 
{
   size_t i,i1,j,j1;

   if (_m->Row != _m->Col)
      printf( "matrix<T>::Cofact(): Cofactor of a non-square matrix!");

   if (row > _m->Row || col > _m->Col)
      printf( "matrix<T>::Cofact(): Index out of range!");

   matrix<T> temp (_m->Row-1,_m->Col-1);

   for (i=i1=0; i < _m->Row; i++)
   {
      if (i == row)
	continue;
      for (j=j1=0; j < _m->Col; j++)
      {
	 if (j == col)
	    continue;
	 temp._m->Val[i1][j1] = _m->Val[i][j];
	 j1++;
      }
      i1++;
   }
   T  cof = temp.Det();
   if ((row+col)%2 == 1)
      cof = -cof;

   return cof;
}


// calculate adjoin of a matrix
template <class T> inline matrix<T>
matrix<T>::Adj () 
{
   if (_m->Row != _m->Col)
      printf( "matrix<T>::Adj(): Adjoin of a non-square matrix.");

   matrix<T> temp(_m->Row,_m->Col);

   for (size_t i=0; i < _m->Row; i++)
      for (size_t j=0; j < _m->Col; j++)
	  temp._m->Val[j][i] = Cofact(i,j);
   return temp;
}

// Determine if the matrix is singular
template <class T> inline bool
matrix<T>::IsSingular () 
{
   if (_m->Row != _m->Col)
      return false;
   return (Det() == T(0));
}

// Determine if the matrix is diagonal
template <class T> inline bool
matrix<T>::IsDiagonal () 
{
   if (_m->Row != _m->Col)
      return false;
   for (size_t i=0; i < _m->Row; i++)
     for (size_t j=0; j < _m->Col; j++)
	if (i != j && _m->Val[i][j] != T(0))
	  return false;
   return true;
}

// Determine if the matrix is scalar
template <class T> inline bool
matrix<T>::IsScalar () 
{
   if (!IsDiagonal())
     return false;
   T v = _m->Val[0][0];
   for (size_t i=1; i < _m->Row; i++)
     if (_m->Val[i][i] != v)
	return false;
   return true;
}

// Determine if the matrix is a unit matrix
template <class T> inline bool
matrix<T>::IsUnit () 
{
   if (IsScalar() && _m->Val[0][0] == T(1))
     return true;
   return false;
}

// Determine if this is a null matrix
template <class T> inline bool
matrix<T>::IsNull () 
{
   for (size_t i=0; i < _m->Row; i++)
      for (size_t j=0; j < _m->Col; j++)
	 if (_m->Val[i][j] != T(0))
	    return false;
   return true;
}

// Determine if the matrix is symmetric
template <class T> inline bool
matrix<T>::IsSymmetric () 
{
   if (_m->Row != _m->Col)
      return false;
   for (size_t i=0; i < _m->Row; i++)
      for (size_t j=0; j < _m->Col; j++)
	 if (_m->Val[i][j] != _m->Val[j][i])
	    return false;
   return true;
}
	   
// Determine if the matrix is skew-symmetric
template <class T> inline bool
matrix<T>::IsSkewSymmetric () 
{
   if (_m->Row != _m->Col)
      return false;
   for (size_t i=0; i < _m->Row; i++)
      for (size_t j=0; j < _m->Col; j++)
	 if (_m->Val[i][j] != -_m->Val[j][i])
	    return false;
   return true;
}
   
// Determine if the matrix is upper triangular
template <class T> inline bool
matrix<T>::IsUpperTriangular () 
{
   if (_m->Row != _m->Col)
      return false;
   for (size_t i=1; i < _m->Row; i++)
      for (size_t j=0; j < i-1; j++)
	 if (_m->Val[i][j] != T(0))
	    return false;
   return true;
}

// Determine if the matrix is lower triangular
template <class T> inline bool
matrix<T>::IsLowerTriangular () 
{
   if (_m->Row != _m->Col)
      return false;

   for (size_t j=1; j < _m->Col; j++)
      for (size_t i=0; i < j-1; i++)
	 if (_m->Val[i][j] != T(0))
	    return false;

   return true;
}
}

#define ENFORCE_LIMITS 1

#define PI 3.14159265
#define TWO_PI 6.28318530

#define J1_MIN -2.2853981634
#define J1_MAX  0.7146018366 //0300003
#define J2_MIN -0.5236000000 //59999999999995
#define J2_MAX  1.3963000000 //000001
#define J3_MIN -3.9000000000 //8999999999999999
#define J3_MAX  0.8500000000 //0000004
#define J4_MIN -2.3000000000 //2999999999999998
#define J4_MAX  0.3000000000 //00001
#define J5_MIN  0
#define J5_MAX  6.28318530
#define J6_MIN -2.2000000000 //000002
#define J6_MAX  0.0000000000 //000001
#define J7_MIN  0
#define J7_MAX  6.28318530

class DOFconvertor
{
private:
float L1;
float L2;
int NUM_CELLS;

typedef struct {
	float m[16];
} mat4;

void mul(mat4 a, mat4 b, mat4 *out){
	out->m[0] = a.m[0] * b.m[0] + a.m[1] * b.m[4] + a.m[2] * b.m[8] + a.m[3] * b.m[12];
	out->m[1] = a.m[0] * b.m[1] + a.m[1] * b.m[5] + a.m[2] * b.m[9] + a.m[3] * b.m[13];
	out->m[2] = a.m[0] * b.m[2] + a.m[1] * b.m[6] + a.m[2] * b.m[10] + a.m[3] * b.m[14];
	out->m[3] = a.m[0] * b.m[3] + a.m[1] * b.m[7] + a.m[2] * b.m[11] + a.m[3] * b.m[15];
	out->m[4] = a.m[4] * b.m[0] + a.m[5] * b.m[4] + a.m[6] * b.m[8] + a.m[7] * b.m[12];
	out->m[5] = a.m[4] * b.m[1] + a.m[5] * b.m[5] + a.m[6] * b.m[9] + a.m[7] * b.m[13];
	out->m[6] = a.m[4] * b.m[2] + a.m[5] * b.m[6] + a.m[6] * b.m[10] + a.m[7] * b.m[14];
	out->m[7] = a.m[4] * b.m[3] + a.m[5] * b.m[7] + a.m[6] * b.m[11] + a.m[7] * b.m[15];
	out->m[8] = a.m[8] * b.m[0] + a.m[9] * b.m[4] + a.m[10] * b.m[8] + a.m[11] * b.m[12];
	out->m[9] = a.m[8] * b.m[1] + a.m[9] * b.m[5] + a.m[10] * b.m[9] + a.m[11] * b.m[13];
	out->m[10] = a.m[8] * b.m[2] + a.m[9] * b.m[6] + a.m[10] * b.m[10] + a.m[11] * b.m[14];
	out->m[11] = a.m[8] * b.m[3] + a.m[9] * b.m[7] + a.m[10] * b.m[11] + a.m[11] * b.m[15];
	out->m[12] = a.m[12] * b.m[0] + a.m[13] * b.m[4] + a.m[14] * b.m[8] + a.m[15] * b.m[12];
	out->m[13] = a.m[12] * b.m[1] + a.m[13] * b.m[5] + a.m[14] * b.m[9] + a.m[15] * b.m[13];
	out->m[14] = a.m[12] * b.m[2] + a.m[13] * b.m[6] + a.m[14] * b.m[10] + a.m[15] * b.m[14];
	out->m[15] = a.m[12] * b.m[3] + a.m[13] * b.m[7] + a.m[14] * b.m[11] + a.m[15] * b.m[15];
}

void mul(mat4 a, std::vector<float> b, std::vector<float> *out){
	if(b.size() == 3) b.push_back(1.00);
	if(out->size() == 3) out->push_back(1.00);
	out->at(0) = a.m[0] * b[0] + a.m[1] * b[1] + a.m[2] * b[2] + a.m[3] * b[3];
	out->at(1) = a.m[4] * b[0] + a.m[5] * b[1] + a.m[6] * b[2] + a.m[7] * b[3];
	out->at(2) = a.m[8] * b[0] + a.m[9] * b[1] + a.m[10] * b[2] + a.m[11] * b[3];
	out->at(3) = a.m[12] * b[0] + a.m[13] * b[1] + a.m[14] * b[2] + a.m[15] * b[3];
}

float norm(std::vector<float> v){
	return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

float dot(std::vector<float> v1, std::vector<float> v2){
	return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

std::vector<float> cross(std::vector<float> v1, std::vector<float> v2){
	std::vector<float> out (3,0);
	out[0] = -v1[2] * v2[1] + v1[1] * v2[2]; 
	out[1] =  v1[2] * v2[0] - v1[0] * v2[2]; 
	out[2] = -v1[1] * v2[0] + v1[0] * v2[1]; 
	return out;
}

void normalize(std::vector<float> *v){
	float n = norm(*v);
	v->at(0) = v->at(0) / n;
	v->at(1) = v->at(1) / n;
	v->at(2) = v->at(2) / n;
}

/*void quat_to_mat4(std::vector<float> q1, mat4 *out){
	//not tested
	out->m[0] = 1.00 - 2.00 * q1[2] * q1[2] - 2.00 * q1[3] * q1[3];
	out->m[1] = 2.00 * (q1[1] * q1[2] - q1[0] * q1[3]);
	out->m[2] = 2.00 * (q1[1] * q1[3] + q1[0] * q1[2]);
	out->m[3] = 0.00;
	out->m[4] = 2.00 * (q1[1] * q1[2] + q1[0] * q1[3]);
	out->m[5] = 1.00 - 2.00 * q1[1] * q1[1] - 2.00 * q1[3] * q1[3];
	out->m[6] = 2.00 * (q1[2] * q1[3] - q1[0] * q1[1]);
	out->m[7] = 0.00;
	out->m[8] =  2.00 * (q1[1] * q1[3] - q1[0] * q1[2]);
	out->m[9] =  2.00 * (q1[2] * q1[3] + q1[0] * q1[1]);
	out->m[10] = 1.00 - 2.00 * q1[1] * q1[1] - 2.00 * q1[2] * q1[2];
	out->m[11] = 0.00;
	out->m[12] = 0.00;
	out->m[13] = 0.00;
	out->m[14] = 0.00;
	out->m[15] = 1.00;
}

void mat4_to_euler_xyz(mat4 r, std::vector<float> *angles){
	//unreliable
	float psi = asin(r.m[2]);
	float theta = atan2(-r.m[1], r.m[0]);
	float phi = atan2(-r.m[6], r.m[10]);
	angles->at(0) = phi;
	angles->at(1) = psi;
	angles->at(2) = theta;
}*/

void quat_to_euler_xyz(std::vector<float> q, std::vector<float> *angles){
	//unreliable
	float phi = atan2( 2.00 * (q[0]*q[1] + q[2]*q[3]), 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]));
	float theta = asin(2.00 * (q[0]*q[2] - q[3]*q[1]));
	float psi = atan2( 2.00 * (q[0]*q[3] + q[1]*q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
	angles->at(0) = phi;
	angles->at(1) = theta;
	angles->at(2) = psi;
}


bool isValidState(std::vector<float> v){
	std::vector<float> ee_pos (3, 0.0);
	ee_pos[0] = v[0];
	ee_pos[1] = v[1];
	ee_pos[2] = v[2];
	float ee_dist = norm(ee_pos);
	if(ee_dist > L1 + L2) return false; //too far
	float ee_xz_dist = sqrt(ee_pos[0] * ee_pos[0] + ee_pos[2] * ee_pos[2]);
	if(ee_xz_dist < 0.3) return false; //too close to y-axis
	return true;
}
public:

DOFconvertor(float l1, float l2, int size){
	L1 = l1;
	L2 = l2;
	NUM_CELLS = size;
}

bool convert_to_joint_angles(std::vector<double> coords, std::vector<double> *out){ //, std::vector<float>* wrist, std::vector<float>* elbow){
        int start_t = clock();
	int elapsed_t;
	//#define DEBUG_CONV
	std::vector<float> tg (3,0);
	//coords[3] should come in as int (0 - 15)
	float swivel = float(coords[3]) ;//* (PI / 8.0); //make it in radians
	if(swivel > PI) {
		swivel = swivel - TWO_PI;
	}
	#ifdef DEBUG_CONV
	printf(">> SWIVEL: %.4f\n", swivel);
	#endif
	float cell_width = (L1 + L2) / (float) NUM_CELLS;
	tg[0] = float(coords[0]);// * cell_width; //make wrist position continuous from discrete
	tg[1] = float(coords[1]);// * cell_width;
	tg[2] = float(coords[2]);// * cell_width;
	float tg_norm = norm(tg);
	
	#ifdef DEBUG_CONV
	printf("tg: %.2f %.2f %.2f\n", tg[0], tg[1], tg[2]);
	printf("|tg|: %.2f\n", tg_norm);
	#endif
	std::vector<float> n (3,0);
	n[0] = tg[0] / tg_norm;
	n[1] = tg[1] / tg_norm;
	n[2] = tg[2] / tg_norm;
	#ifdef DEBUG_CONV
	printf("n: %.2f %.2f %.2f\n", n[0], n[1], n[2]);
	#endif
	std::vector<float> a (3,0);
	a[1] = -1; //axis (0,-1,0)
	std::vector<float> u (3,0);
	std::vector<float> v (3,0);
	// u = (a - (a dot n) n) / || a - (a dot n) n||
	float adotn = dot(a, n);
	std::vector<float> pu (3,0);
	pu[0] = a[0] - adotn * n[0];
	pu[1] = a[1] - adotn * n[1];
	pu[2] = a[2] - adotn * n[2];
	float pu_norm = norm(pu);
	u[0] = pu[0] / pu_norm;
	u[1] = pu[1] / pu_norm;
	u[2] = pu[2] / pu_norm;
	// v = n x u
	v = cross(n, u);
	//v[0] = -u[1] * n[2] + u[2] * n[1];
	//v[1] = u[0] * n[2] - u[2] * n[0];
	//v[2] = -u[0] * n[1] + u[1] * n[0];
	#ifdef DEBUG_CONV
	printf("u: %.2f %.2f %.2f\n", u[0], u[1], u[2]);
	printf("v: %.2f %.2f %.2f\n", v[0], v[1], v[2]);
	#endif

	float cos_alpha = ((tg_norm * tg_norm) + (L1 * L1) - (L2 * L2)) / (2.0 * L1 * tg_norm); 
	if(cos_alpha > 1.0 || cos_alpha < -1.0){
		return false;
	}
	std::vector<float> c (3,0);
	c[0] = cos_alpha * L1 * n[0];
	c[1] = cos_alpha * L1 * n[1];
	c[2] = cos_alpha * L1 * n[2];
	float sin_alpha = sqrt(1.00 - (cos_alpha * cos_alpha));
	float R = sin_alpha * L1;
	#ifdef DEBUG_CONV
	printf("cos_alpha: %.2f\n", cos_alpha);
	printf("c: %.2f %.2f %.2f\n", c[0], c[1], c[2]);
	printf("R: %.2f\n", R);
	#endif

	std::vector<float> e (3,0.0);
	e[0] = c[0] + R * (cos(swivel) * u[0] + sin(swivel) * v[0]);
	e[1] = c[1] + R * (cos(swivel) * u[1] + sin(swivel) * v[1]);
	e[2] = c[2] + R * (cos(swivel) * u[2] + sin(swivel) * v[2]);
	#ifdef DEBUG_CONV
	printf("elbow pos: %.2f %.2f %.2f\n", e[0], e[1], e[2]);
	#endif

	std::vector<float> e_u (3, 0.0); //elbow unit
	e_u[0] = e[0] / L1;
	e_u[1] = e[1] / L1;
	e_u[2] = e[2] / L1;

	//computing elbow angle - 0 is extended (as opposed to 180) and value is always negative
	float cos_theta4 = ((L1 * L1) + (L2 * L2) - (tg_norm * tg_norm)) / (2.00 * L1 * L2); //from law of cosines
	out->at(3) = -(PI - acos(cos_theta4));
	float theta_elbow = acos(cos_theta4);
	#ifdef DEBUG_CONV
	printf(">> Elbow angle: %.4f\n", out->at(3));
	#endif
	//enforce joint limits
	if(out->at(3) < J4_MIN) return false;
	if(out->at(3) > J4_MAX) return false;

	//computing the target coord sys.
	std::vector<float> xg_hat (3, 0.0);
	std::vector<float> yg_hat (3, 0.0);
	std::vector<float> zg_hat (3, 0.0);

	xg_hat = e_u;
	float wdotxg_hat = dot(tg, xg_hat);
	yg_hat[0] = tg[0] - wdotxg_hat * xg_hat[0];
	yg_hat[1] = tg[1] - wdotxg_hat * xg_hat[1];
	yg_hat[2] = tg[2] - wdotxg_hat * xg_hat[2];
	if(norm(yg_hat) == 0) {
		yg_hat[0] = 0; yg_hat[1] = 1; yg_hat[2] = 0;
	}
	normalize(&yg_hat);
	zg_hat = cross(xg_hat, yg_hat);

	matrix<double> H_s2wrld(4, 4); //shoulder to world (used to extract shoulder angles)
	H_s2wrld(0,0) = xg_hat[0]; H_s2wrld(0,1) = yg_hat[0]; H_s2wrld(0,2) = zg_hat[0]; H_s2wrld(0,3) = 0;
	H_s2wrld(1,0) = xg_hat[1]; H_s2wrld(1,1) = yg_hat[1]; H_s2wrld(1,2) = zg_hat[1]; H_s2wrld(1,3) = 0;
	H_s2wrld(2,0) = xg_hat[2]; H_s2wrld(2,1) = yg_hat[2]; H_s2wrld(2,2) = zg_hat[2]; H_s2wrld(2,3) = 0;
	H_s2wrld(3,0) =         0; H_s2wrld(3,1) =         0; H_s2wrld(3,2) =         0; H_s2wrld(3,3) = 1;

	//H_s2wrld represents the shoulder rotation. (SHOULD BE XZY ROTATION ORDER I THINK)
	//extract the 3 euler angles from the rotation matrix H_s2wrld

	float s_lift1 = asin(H_s2wrld(1,0));
	float s_lift2 = PI - s_lift1;

	float s_roll1 = atan2(-H_s2wrld(1,2) / cos(s_lift1), H_s2wrld(1,1) / cos(s_lift1));
	float s_roll2 = atan2(-H_s2wrld(1,2) / cos(s_lift2), H_s2wrld(1,1) / cos(s_lift2));
	
	float s_pan1 = atan2(-H_s2wrld(2,0) / cos(s_lift1), H_s2wrld(0,0) / cos(s_lift1));
	float s_pan2 = atan2(-H_s2wrld(2,0) / cos(s_lift2), H_s2wrld(0,0) / cos(s_lift2));

	//enforce joint limits on both solutions
	if(s_pan1 >= J1_MIN && s_pan1 <= J1_MAX && -s_lift1 >= J2_MIN && -s_lift1 <= J2_MAX && s_roll1 >= J3_MIN && s_roll1 <= J3_MAX){
		//use solution 1
		out->at(0) = s_pan1;
		out->at(1) = -s_lift1;
		out->at(2) = s_roll1;
	}
	else if(s_pan2 >= J1_MIN && s_pan2 <= J1_MAX && -s_lift2 >= J2_MIN && -s_lift2 <= J2_MAX && s_roll2 >= J3_MIN && s_roll2 <= J3_MAX){
		//use solution 2
		out->at(0) = s_pan2;
		out->at(1) = -s_lift2;
		out->at(2) = s_roll2;
	} else {
		//bugger
		elapsed_t = clock() - start_t;
		return false;
	}

//TODO: wrist rotation
#define USE_LOCAL
#ifdef USE_LOCAL
	out->at(4) = float(coords[4]);// * PI / 8.0f; //forearm roll 0:TWO_PI unlimited
	out->at(5) = float(coords[5]);//-coords[5] * PI / 8.0f; //wrist flex 0:-2.2
	if(out->at(5) <= -PI) { out->at(5)+=TWO_PI; }
	out->at(6) = float(coords[6]);// * PI / 8.0f; //wrist roll 0:TWO_PI unlimited
	if(out->at(5) < J6_MIN || out->at(5) > J6_MAX){
		//printf("Wrist flex BAD: %.3f\n", out->at(5));
		return false;
	}
	//printf("Wrist flex OK: %.3f\n", out->at(5));
	return true;
#else
	matrix<double> H_e2s(4,4); //elbow to shoulder
	H_e2s(0,0) = cos(theta_elbow); H_e2s(0,1) = -sin(theta_elbow); H_e2s(0,2) = 0; H_e2s(0,3) = L2;
	H_e2s(1,0) = sin(theta_elbow); H_e2s(1,1) = cos(theta_elbow);  H_e2s(1,2) = 0; H_e2s(1,3) = 0;
	H_e2s(2,0) =                0; H_e2s(2,1) =                0;  H_e2s(2,2) = 1; H_e2s(2,3) = 0;
	H_e2s(3,0) =                0; H_e2s(3,1) =                0;  H_e2s(3,2) = 0; H_e2s(3,3) = 1;

	matrix<double> H_w2wrld(4,4); //wrist to world (computed based on the input coords)
	float cx = cos(coords[4]);//*PI / 8.0f);
	float sx = sin(coords[4]);//*PI / 8.0f);
	float cy = cos(coords[5]);//*PI / 8.0f);
	float sy = sin(coords[5]);//*PI / 8.0f);
	float cz = cos(coords[6]);//*PI / 8.0f);
	float sz = sin(coords[6]);//*PI / 8.0f);
	//assuming (Y,Z,X) local order of rotations	
	H_w2wrld(0,0) =  cy*cz + sy*sx*sz; H_w2wrld(0,1) = -cy*sz + sy*sx*cz; H_w2wrld(0,2) = sy*cx; H_w2wrld(0,3) = tg[0];
	H_w2wrld(1,0) =             cx*sz; H_w2wrld(1,1) =             cx*cz; H_w2wrld(1,2) =   -sx; H_w2wrld(1,3) = tg[1];
	H_w2wrld(2,0) = -sy*cz + cy*sx*sz; H_w2wrld(2,1) =  sy*sz + cy*sx*cz; H_w2wrld(2,2) = cx*cy; H_w2wrld(2,3) = tg[2];
	H_w2wrld(3,0) =              0.0f; H_w2wrld(3,1) =              0.0f; H_w2wrld(3,2) =  0.0f; H_w2wrld(3,3) =  1.0f;
	
	matrix<double> H_w2e(4,4); //wrist to elbow (what we are looking for)

	//H_w2wrld = H_s2wrld * H_e2s * H_w2e
	H_w2e = (!H_e2s)*(!H_s2wrld)*H_w2wrld;

	printf("H_w2e:\n");
	printf("[%.3f %.3f %.3f %.3f]\t [cy   ,           -cz*sy,             sz*sy,  wp_e]\n", H_w2e(0,0), H_w2e(0,1), H_w2e(0,2), H_w2e(0,3));
	printf("[%.3f %.3f %.3f %.3f]\t [cx*sy, cx*cy*cz - sx*sz, -cy*cx*sz - cz*sx,  wp_e]\n", H_w2e(1,0), H_w2e(1,1), H_w2e(1,2), H_w2e(1,3));
	printf("[%.3f %.3f %.3f %.3f]\t [sy*sx, cx*sz + cz*cy*sx,  cx*cz - cy*sx*sz,  wp_e]\n", H_w2e(2,0), H_w2e(2,1), H_w2e(2,2), H_w2e(2,3));
	printf("[%.3f %.3f %.3f %.3f]\t [    0,                0,                 0,     1]\n", H_w2e(3,0), H_w2e(3,1), H_w2e(3,2), H_w2e(3,3));
	
//TODO: extract angles from H_w2e in (X,Z,X) order of rotations (roll(x), flex(y), roll(z))
//[cy   ,           -cz*sy,             sz*sy, tg[0]]
//[cx*sy, cx*cy*cz - sx*sz, -cy*cx*sz - cz*sx, tg[1]]
//[sy*sx, cx*sz + cz*cy*sx,  cx*cz - cy*sx*sz, tg[2]]
//[    0,                0,                 0,     1]
//extract the 3 euler angles from the rotation matrix H_w2e

	float w_flex1 = acos(H_w2e(0,0));
	float w_flex2 = PI - w_flex1;

	float w_roll1 = atan2(-H_w2e(0,2) / sin(w_flex1), H_w2e(0,1) / sin(w_flex1));
	if(w_roll1 > PI/2.0) w_roll1-=PI;
	float w_roll2 = atan2(-H_w2e(0,2) / sin(w_flex2), H_w2e(0,1) / sin(w_flex2));
	if(w_roll2 > PI/2.0) w_roll2-=PI;
	
	float fa_roll1 = atan2(H_w2e(2,0) / sin(w_flex1), -H_w2e(1,0) / sin(w_flex1));
	if(fa_roll1 > PI/2.0) fa_roll1-=PI;
	float fa_roll2 = atan2(H_w2e(2,0) / sin(w_flex2), -H_w2e(1,0) / sin(w_flex2));
	if(fa_roll2 > PI/2.0) fa_roll2-=PI;
	
	printf("Angles: %.3f, %.3f, %.3f\n", fa_roll1, -w_flex1, w_roll1);
	printf("Secondary Angles: %.3f, %.3f, %.3f\n", fa_roll2, -w_flex2, w_roll2);
	
	//enforce joint limits on both solutions
	if(-w_flex1+PI/2.0 >= J6_MIN && -w_flex1+PI/2.0 <= J6_MAX){
		//use solution 1
		out->at(4) = fa_roll1;
		out->at(5) = -w_flex1+PI/2.0;
		out->at(6) = w_roll1;
		printf("Using primary!\n");
	}
	else if(-w_flex2+PI/2.0 >= J6_MIN && -w_flex2+PI/2.0 <= J6_MAX){
		//use solution 2
		out->at(4) = fa_roll2;
		out->at(5) = -w_flex2+PI/2.0;
		out->at(6) = w_roll2;
		printf("Using secondary!\n");
	} else {
		//bugger
		elapsed_t = clock() - start_t;
		printf("Fail!\n");
		return false;
	}
	printf("CONVERSION TIME: %d\n", elapsed_t);
#endif
	elapsed_t = clock() - start_t;
	return true;
  }
};
