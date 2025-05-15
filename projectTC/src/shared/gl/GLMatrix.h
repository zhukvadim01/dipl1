#ifndef _GLMATRIX_H
#define _GLMATRIX_H

#include <QDateTime>

#include "GLFileLog.h"
#include "GLMath.h"

#include <cstring>
#include <memory>

constexpr int SIZE_M = 10; //size of matrix
constexpr int SIZE_ENLARGED_M = 30; //size of enlarged matrix
constexpr double ZERO_TOLERANCE_M = 1.E-14; //zero tolerance
constexpr double ZERO_TOLERANCE_INV = 1.E-80; //zero tolerance for the inverse matrix calculation
constexpr double EPSILON_M = 1.E-10;

namespace GL_MATR
{

constexpr int MAX_LEN_STR = 2000; // maximum string length
constexpr int STR_UNDENT = 50;    // string undent

}

#define GLMatrix  TMatrix<SIZE_M> // matrix with default size SIZE_M
#define GLVector  TVector<SIZE_M> // vector with default size SIZE_M
#define EnlargedMatrix  TMatrix<SIZE_ENLARGED_M> // matrix with default large size SIZE_ENLARGED_M
#define EnlargedVector  TVector<SIZE_ENLARGED_M> // vector with default large size SIZE_ENLARGED_M
#define CMatrix TCMatrixFunc<SIZE_M>
#define CEnlargedMatrix TCMatrixFunc<SIZE_ENLARGED_M>

typedef double Matrix3x3[3][3];
typedef double Vector3[3];

const Matrix3x3 UNITY_MATRIX = {{1,0,0}, {0,1,0}, {0,0,1}};
const Matrix3x3 ZERO_MATRIX = {{0,0,0}, {0,0,0}, {0,0,0}};
const Vector3	ZERO_VECTOR = {0,0,0};

//PACKAGE		:	GL
//TEMPLATE		:	TVector
//DESCRIPTION	:	Template class for storage vector
//PARAMETERS    :   _size_m - size of array Vec
template <int _size_m>
class TVector //template to use vector with arbitrarily given size
{
public:
    // PACKAGE		:   GL
    // FUNCTION 	:   TVector()
    // DESCRIPTION	:   Class constructor
    // INPUTS		:	NONE
    // RETURNS		:	NONE
    TVector();

    TVector( int size );

    //PACKAGE		: GL
    //FUNCTION		: TVector::Reset
    //DESCRIPTION	: Initialization
    //INPUTS		: None
    //RETURNS		: None
    void Reset();

    //PACKAGE		: GL
    //FUNCTION		: TVector::Reset
    //DESCRIPTION	: Initialization, with given size of vector
    //INPUTS		: Size of vector
    //RETURNS		: None
    void Reset(int size);

    //PACKAGE		: GL
    //FUNCTION		: TVector::operator=
    //DESCRIPTION	: Assignment operator
    //INPUTS		: Assignming data
    //RETURNS		: Result of assignment
    TVector<_size_m>& operator=(const TVector<_size_m>& Q);

    //PACKAGE		: GL
    //FUNCTION		: TVector::operator=
    //DESCRIPTION	: Assignment operator
    //INPUTS		: Assignming data
    //RETURNS		: Result of assignment
    TVector<_size_m>& operator+=(const TVector<_size_m>& Q);

    //PACKAGE		: GL
    //FUNCTION		: TVector::operator[]
    //DESCRIPTION	: Indexation operator
    //INPUTS		: Index
    //RETURNS		: Element at position determined by index
    double& operator[](int ind);

    //PACKAGE		: GL
    //FUNCTION		: TVector::operator+
    //DESCRIPTION	: Add operator
    //INPUTS		: Adding data
    //RETURNS		: Sum
    TVector<_size_m> operator+(const TVector<_size_m> &Q) const;

    //PACKAGE		: GL
    //FUNCTION		: TVector::operator-
    //DESCRIPTION	: Subtraction operator
    //INPUTS		: Subtrahend
    //RETURNS		: Difference
    TVector<_size_m> operator-(const TVector<_size_m> &Q) const;

    //PACKAGE		: GL
    //FUNCTION		: TVector::operator*
    //DESCRIPTION	: Multiplication of vector by a real number operator
    //INPUTS		: Factor (real number)
    //RETURNS		: Result of multiplication
    TVector<_size_m> operator*(const double &mult) const;

    // PACKAGE		:   GL
    // FUNCTION 	:   TVector::PickOut()
    // DESCRIPTION	:   Pick out part of vector
    // INPUTS		:	Number of first element, number of last element
    // RETURNS		:	Resulting vector; ItsOk - sign of the successful implementaion
    TVector<_size_m> PickOut(int from, int to, bool &ItsOk);

    // PACKAGE		:   GL
    // FUNCTION 	:   TVector::IsZero()
    // DESCRIPTION	:   Checks whether the vector is empty
    // INPUTS		:	None
    // RETURNS		:	True if vector is empty
    bool IsZero();

    // PACKAGE		:   GL
    // FUNCTION 	:   TVector::SetDimensions()
    // DESCRIPTION	:   Sets number of elements
    // INPUTS		:	Number of elements
    // RETURNS		:	None
    void SetDimension(int _dim);

    // PACKAGE		:   GL
    // FUNCTION 	:   TVector::Module()
    // DESCRIPTION	:   Returns module of current vector
    // INPUTS		:	None
    // RETURNS		:	Module of current vector
    double Module();

    // PACKAGE		:   GL
    // FUNCTION 	:   TVector::ScalProd()
    // DESCRIPTION	:   Scalar product
    // INPUTS		:	2nd vector
    // RETURNS		:	Scalar product of given and 2nd vectors
    double ScalProd(TVector<_size_m> &u);

    double Vec[_size_m];			// Vector of numbers
    int s{0};                  		// Vector size

};


//PACKAGE		:	GL
//TEMPLATE		:	TMatrix
//DESCRIPTION	:	Template class for storage matrix
//PARAMETERS    :   _size_m - both sizes of array M; this value determines size of allocated memory
//              :             and does not determine the number of rows or columns of the matrix used for computations
template <int _size_m>
class TMatrix //template to use matrix vith arbitrarily given size
{
public:
    double M[_size_m][_size_m];  // Matrix of numbers
    int s_m = 0;                 // Number of rows
    int s_n = 0;                 // Number of columns

    TMatrix();
    TMatrix(int _str, int _col);

    //PACKAGE		: GL
    //FUNCTION		: TVector::Reset
    //DESCRIPTION	: Initialization
    //INPUTS		: None
    //RETURNS		: None
    void Reset();

    //PACKAGE		: GL
    //FUNCTION		: TVector::Reset
    //DESCRIPTION	: Initialization, with given sizes of matrix
    //INPUTS		: Number of rows; number of columns
    //RETURNS		: None
    void Reset(int _str, int _col);

    TMatrix<_size_m>& operator=(const TMatrix<_size_m>& matr);
    TMatrix<_size_m> operator+(const TMatrix<_size_m> &Q) const;
    TMatrix<_size_m> operator-(const TMatrix<_size_m> &Q) const;
    TMatrix<_size_m> operator*(const double &mult) const;
	
    // PACKAGE		:   GL
    // FUNCTION 	:   TMatrix::PickOut()
    // DESCRIPTION	:   Pick out part of matrix
    // INPUTS		:	Number of first row, number of last row, number of first column, number of last columt
    // RETURNS		:	Resulting matrix; ItsOk - sign of the successful implementaion
    TMatrix<_size_m> PickOut(int from1, int to1, int from2, int to2, bool &ItsOk);

    // PACKAGE		:   GL
    // FUNCTION 	:   TMatrix::IsZero()
    // DESCRIPTION	:   Checks whether the matrix is empty
    // INPUTS		:	None
    // RETURNS		:	True if matrix is empty
    bool IsZero();

    // PACKAGE		:   GL
    // FUNCTION 	:   TMatrix::SetDimensions()
    // DESCRIPTION	:   Sets number of rows and number of columns
    // INPUTS		:	Number of rows, number of columns
    // RETURNS		:	None
    void SetDimensions(int _rows, int _cols);

    // PACKAGE		:   GL
    // FUNCTION 	:   TMatrix::ReflectNonZeroRelativeDiag()
    // DESCRIPTION	:   Reflect non-zero elements relative to the matrix diagonal
    // INPUTS		:	None
    // RETURNS		:	None
    void ReflectNonZeroRelativeDiag();

    // PACKAGE		:   GL
    // FUNCTION 	:   TMatrix::CovMatrAdaptation()
    // DESCRIPTION	:   Adaptation of covariance matrix
    // INPUTS		:	Value of dimension; minimum acceptable value of RMSE;
    //              :   correction value for covariance moments
    // RETURNS		:	NONE
    void FCovMatrAdaptation(short Dim, double MinRMSE, double CovCorr);

    // PACKAGE		:   GL
    // FUNCTION 	:   TMatrix::CovMatrAdaptation()
    // DESCRIPTION	:   Adaptation of covariance matrix
    // INPUTS		:	NONE
    // RETURNS		:	NONE
    void FCovMatrAdaptation();

    // PACKAGE		:   GL
    // FUNCTION 	:   TMatrix::GetMax()
    // DESCRIPTION	:   Returns maximum element
    // INPUTS		:	NONE
    // RETURNS		:	Maximum element
    double GetMax();

    // PACKAGE		:   GL
    // FUNCTION 	:   TMatrix::GetMaxAbs()
    // DESCRIPTION	:   Returns a maximum absolute value (modulus) of elements
    // INPUTS		:	NONE
    // RETURNS		:	Maximum absolute value (modulus) of elements
    double GetMaxAbs();

    // PACKAGE		:   GL
    // FUNCTION 	:   TMatrix::GetMaxDiag()
    // DESCRIPTION	:   Returns maximum diagonal element
    // INPUTS		:	NONE
    // RETURNS		:	Maximum diagonal element
    double GetMaxDiag();

    // PACKAGE		:   GL
    // FUNCTION 	:   TMatrix::GetMinDiagPositive()
    // DESCRIPTION	:   Returns minimum positive diagonal element
    // INPUTS		:	NONE
    // RETURNS		:	Minimum positive diagonal element
    double GetMinDiagPositive();

    // PACKAGE		:   GL
    // FUNCTION 	:   TMatrix::GetMaxDiag3()
    // DESCRIPTION	:   Returns maximum diagonal element from elements [0][0], [1][1], [2][2]
    // INPUTS		:	NONE
    // RETURNS		:	Maximum diagonal element from elements [0][0], [1][1], [2][2]
    double GetMaxDiag3();

    // PACKAGE		:   GL
    // FUNCTION 	:   TMatrix::GetTrace()
    // DESCRIPTION	:   Returns trace of the matrix (sum of the diagonal elements)
    // INPUTS		:	NONE
    // RETURNS		:	Trace of the matrix (sum of the diagonal elements)
    double GetTrace();

    // PACKAGE		:   GL
    // FUNCTION 	:   TMatrix::GetTraceVel()
    // DESCRIPTION	:   Returns trace of the matrix with reference to the velocity (sum of the elements [3][3], [4][4], [5][5])
    // INPUTS		:	NONE
    // RETURNS		:	Trace of the matrix with reference to the velocity (sum of the elements [3][3], [4][4], [5][5])
    double GetTraceVel();

    // PACKAGE		:   GL
    // FUNCTION 	:   TMatrix::IsSymmetricPositiveDefinite()
    // DESCRIPTION	:   Checks whether the current matrix is symmetric positive-definite
    // INPUTS		:	NONE
    // RETURNS		:	True if current matrix is square, symmetric and positive-definite
    bool IsSymmetricPositiveDefinite();

    // PACKAGE		:   GL
    // FUNCTION 	:   TMatrix::RestrictionRMSE()
    // DESCRIPTION	:   Restriction RMSE by given value of maximum RMSE
    // INPUTS		:	Value of maximum RMSE
    // RETURNS		:	True if current matrix is square, symmetric and positive-definite
    void RestrictionRMSE(double MaxRMSE);

    // PACKAGE		:   GL
    // FUNCTION 	:   TMatrix::FormIdentityMatrix()
    // DESCRIPTION	:   Forms identity matrix (unit matrix) of given size
    // INPUTS		:	Size of square matrix
    // RETURNS		:	None
    void FormIdentityMatrix(int _n);

    // PACKAGE		:   GL
    // FUNCTION 	:   TMatrix::FormRotationMatrix()
    // DESCRIPTION	:   Forms the matrix of rotation around the vector (X;Y;Z) through angle Theta, anticlockwise
    // INPUTS		:	Angle Theta (radians); coordinates of the vector (X;Y;Z)
    // RETURNS		:	None
    void FormRotationMatrix(double Theta, double X, double Y, double Z);
};


//PACKAGE           :	GL
//TEMPLATE OF CLASS	:	TCMatrixFunc
//DESCRIPTION       :	Template of class containing the functions for operating with matrix
template <int _size_m>
class TCMatrixFunc
{
public:
    TCMatrixFunc() { Reset();}

    ~TCMatrixFunc() {}

	// PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::InvMatrix
	// DESCRIPTION	:   Calculating inverse matrix 3x3
	// INPUTS		:	Reference to matrix M, reference to resulting matrix A
	// RETURNS		:	true if conversion is ok
	bool InvMatrix(Matrix3x3 &M, Matrix3x3 &A);

    // PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::InvMatrix
    // DESCRIPTION	:   Calculating inverse matrix 3x3
    // INPUTS		:	Reference to matrix M_in, reference to resulting matrix M_out
    // RETURNS		:	true if conversion is ok
    bool InvMatrix3x3(TMatrix<_size_m> &M_in, TMatrix<_size_m> &M_out);

	// PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::InvMatrix
	// DESCRIPTION	:   Calculating inverse matrix common
	// INPUTS		:	Reference to matrix M, reference to resulting matrix I 
	// RETURNS		:	true if conversion is ok
    bool InvMatrix(TMatrix<_size_m> &M, TMatrix<_size_m> &I);

    // PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::InvMatrix
    // DESCRIPTION	:   Calculating inverse matrix common
    // INPUTS		:	Reference to the matrix M; reference to the resulting matrix I; reference to the determinant of M
    // RETURNS		:	true if conversion is ok
    bool InvMatrix(TMatrix<_size_m> &M, TMatrix<_size_m> &I, double &DetM);

    // PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::InvMatrixLowerTriangular
    // DESCRIPTION	:   Calculating inverse matrix for lower triangular matrix
    // INPUTS		:	Reference to lower triangular matrix M, reference to resulting inverse matrix ResInv
    // RETURNS		:	true if inversion is ok
    bool InvMatrixLowerTriangular(TMatrix<_size_m> &M, TMatrix<_size_m> &ResInv);

    // PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::InvMatrixSymmPos
    // DESCRIPTION	:   Calculating inverse matrix for symmetric positive-definite matrix
    // INPUTS		:	Reference to symmetric positive-definite matrix M, reference to resulting inverse matrix ResInv
    // RETURNS		:	true if inversion is ok
    bool InvMatrixSymmPos(TMatrix<_size_m> &M, TMatrix<_size_m> &ResInv);

    // PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::InvMatrixSymmPos
    // DESCRIPTION	:   Calculating inverse matrix for symmetric positive-definite matrix
    // INPUTS		:	Reference to the symmetric positive-definite matrix M,
    //              :   reference to the resulting inverse matrix ResInv; reference to the calculated determinant of M
    // RETURNS		:	true if inversion is ok
    bool InvMatrixSymmPos(TMatrix<_size_m> &M, TMatrix<_size_m> &ResInv, double &DetM);

	// PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::MatrXMatr
	// DESCRIPTION	:   Multiplying of two matrices - common
    // INPUTS		:	Reference to the matrix A, reference to the matrix B for multiplication,
    //					reference to the resulting matrix Res
    // RETURNS		:	True if multiplication is ok
    bool MatrXMatr(const TMatrix<_size_m> &A, const TMatrix<_size_m> &B, TMatrix<_size_m> &Res);

    // PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::MatrXMatrT
    // DESCRIPTION	:   Multiplying of the matrix and the transposed matrix A*BT
    // INPUTS		:	Reference to the matrix A, reference to the transposed matrix BT,
    //					reference to the resulting matrix Res
    // RETURNS		:	True if multiplication is ok
    bool MatrXMatrT(const TMatrix<_size_m> &A, const TMatrix<_size_m> &BT, TMatrix<_size_m> &Res);

    // PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::MatrXMatrLowerTriangular
    // DESCRIPTION	:   Multiplying of two lower triangular matrices
    // INPUTS		:	Reference to the lower triangular matrix M1,
    //              :   reference to the lower triangular  matrix M2 for multiplication,
    //					reference to resulting matrix A
    // RETURNS		:	true if multiplication is ok
    bool MatrXMatrLowerTriangular(const TMatrix<_size_m> &M1, const TMatrix<_size_m> &M2, TMatrix<_size_m> &A);

    // PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::MatrXMatrUpperTriangToLowerTriang
    // DESCRIPTION	:   Multiplying upper triangular matrix to the lower triangular matrix, with the same dimension
    // INPUTS		:	Reference to the upper triangular matrix M1,
    //              :   reference to the lower triangular  matrix M2 for multiplication,
    //					reference to resulting matrix A
    // RETURNS		:	true if multiplication is ok
    bool MatrXMatrUpperTriangToLowerTriang(const TMatrix<_size_m> &M1, const TMatrix<_size_m> &M2, TMatrix<_size_m> &A);

    // PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::MatrXMatrUpperTriangToLowerTriang
    // DESCRIPTION	:   Multiplying upper triangular matrix LT (L transposed) to the lower triangular matrix L
    // INPUTS		:	Reference to the lower triangular  matrix L,
    //					reference to resulting matrix Res
    // RETURNS		:	True if multiplication is ok
    bool MatrLTL_UpperTriangToLowerTriang(const TMatrix<_size_m> &L, TMatrix<_size_m> &Res);

    // PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::MatrXMatrXMatr
    // DESCRIPTION	:   Multiplying of three matrices - common
    // INPUTS		:	Reference to matrix M1, reference to matrix M2,
    //                  reference to matrix M3 for multiplication,
    //					reference to resulting matrix A
    // RETURNS		:	true if multiplication is ok
    bool MatrXMatrXMatr(const TMatrix<_size_m> &M1, const TMatrix<_size_m> &M2, const TMatrix<_size_m> &M3, TMatrix<_size_m> &A);

    // PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::MatrMultAxBxAT
    // DESCRIPTION	:   Multiplying of the matrices A x B x AT
    // INPUTS		:	Reference to the matrix A, reference to the matrix B,
    //					reference to the resulting matrix Res
    // RETURNS		:	true if multiplication is ok
    bool MatrMultAxBxAT(const TMatrix<_size_m> &A, const TMatrix<_size_m> &B, TMatrix<_size_m> &Res);

    // PACKAGE		:   GL //Add TaniaP
    // FUNCTION 	:   TCMatrixFunc::MatrXVec
    // DESCRIPTION	:   Multiplying of matrix on a vector - common
    // INPUTS		:	Reference to matrix M, reference to vector V for multiplication,
    //					reference to resulting matrix A_out
    // RETURNS		:	true if multiplication is ok
    bool MatrXVec(TMatrix<_size_m> &M, TVector<_size_m> &V, TMatrix<_size_m> &A_out);

    // PACKAGE		:   GL //Add TaniaP
    // FUNCTION 	:   TCMatrixFunc::MatrXVec
    // DESCRIPTION	:   Multiplying of matrix on a vector - common
    // INPUTS		:	Reference to matrix M, reference to vector V for multiplication,
    //					reference to resulting vector A_out
    // RETURNS		:	true if multiplication is ok
    bool MatrXVec(TMatrix<_size_m> &M, TVector<_size_m> &V, TVector<_size_m> &A_out);

    // PACKAGE		:   GL //Add TaniaP
    // FUNCTION 	:   TCMatrixFunc::VecXMatr
    // DESCRIPTION	:   Multiplying of vector on a matrix - common
    // INPUTS		:	Reference to vector V, reference to matrix M for multiplication,
    //					reference to resulting matrix A_out
    // RETURNS		:	true if multiplication is ok
    bool VecXMatr(TVector<_size_m> &V, TMatrix<_size_m> &M, TMatrix<_size_m> &A_out);

    // PACKAGE		:   GL //Add TaniaP
    // FUNCTION 	:   TCMatrixFunc::VecXMatr
    // DESCRIPTION	:   Multiplying of vector on a matrix - common
    // INPUTS		:	Reference to vector V, reference to matrix M for multiplication,
    //					reference to resulting vector A_out
    // RETURNS		:	true if multiplication is ok
    bool VecXMatr(TVector<_size_m> &V, TMatrix<_size_m> &M, TVector<_size_m> &A_out);

    // PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::VecXMatrXVec
    // DESCRIPTION	:   Multiplies vector V1 transposed on matrix M on vector V2
    // INPUTS		:	Reference to vectors V1, V2, matrix M, reference to resulting double out
    // RETURNS		:	true if multiplication is ok
    bool VecXMatrXVec(TVector<_size_m> &V1, TMatrix<_size_m> &M,TVector<_size_m> &V2, double &out);

    // PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::CalcMahalanobisDist
    // DESCRIPTION	:   Calculates Mahalanobis distance using given vector and given covariance matrix,
    //              :   i.e. sqrt(VT x Inv_M x V), where V is given vector, VT is V transposed, Inv_M is given inverse matrix
    // INPUTS		:	Reference to vector V; reference to covariance matrix
    // RETURNS		:   Mahalanobis distance (-1 if result is wrong)
    double CalcMahalanobisDist(TVector<_size_m> &V, TMatrix<_size_m> &Inv_M);

	// PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::MatrXMatr
	// DESCRIPTION	:   Multiplying of two matrices  
	// INPUTS		:	Reference to matrix M1 3x3, reference to matrix  M2 3x3 for multiplication,
	//					reference to resulting matrix A 3x3
	// RETURNS		:	true if multiplication is ok
	bool MatrXMatr(Matrix3x3 &M1, Matrix3x3 &M2, Matrix3x3 &A);

	// PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::Matr3XVect3
	// DESCRIPTION	:   Multiplying of matrix on a vector  
	// INPUTS		:	Reference to matrix M 3x3, reference to vector V (3 dimension) for multiplication,
	//					reference to resulting vector Vec (3 dimension)
	// RETURNS		:	true if multiplication is ok
	bool Matr3XVect3(Matrix3x3 &M, const Vector3 &V, Vector3 &Vec);

	// PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::VecXVec
	// DESCRIPTION	:   Multiplying of a column on a row - common 
	// INPUTS		:	Reference to vector V1, reference to vector V2 for multiplication,
	//					reference to resulting matrix A
	// RETURNS		:	true if multiplication is ok
    bool VecXVec(TVector<_size_m> &V1, TVector<_size_m> &V2, TMatrix<_size_m> &A);

	// PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::VecXVec
	// DESCRIPTION	:   Multiplying of a column on a row - 3x3
	// INPUTS		:	Reference to vector V1 (3 dimension), reference to vector V2 (3 dimension) for multiplication,
	//					reference to resulting matrix A 3x3
	// RETURNS		:	true if multiplication is ok
	bool VecXVec(Vector3 &V1, Vector3 &V2, Matrix3x3 &A);

	// PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::VecXVec
	// DESCRIPTION	:   Multiplying of a row on a column - common
	// INPUTS		:	Reference to vector V1, reference to vector V2 for multiplication,
	//					reference to resulting number a
	// RETURNS		:	true if multiplication is ok
    bool VecXVec(TVector<_size_m> &V1, TVector<_size_m> &V2, double &a);

	// PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::VecXVec
	// DESCRIPTION	:   Multiplying of a row on a column - common - 3x3
	// INPUTS		:	Reference to vector V1, reference to vector V2 for multiplication,
	//					reference to resulting number a
	// RETURNS		:	true if multiplication is ok
	bool VecXVec(Vector3 &V1, Vector3 &V2, double &a);

    // PACKAGE		:   GL //Add TaniaP
    // FUNCTION 	:   TCMatrixFunc::Mult
    // DESCRIPTION	:   Multiplying of a scalar on a vector
    // INPUTS		:	Reference to scalar C, reference to vector V for multiplication,
    //              :   reference to resulting vector A_out
    // RETURNS		:	None
    void Mult(double &C, TVector<_size_m> &V, TVector<_size_m> &A_out);

    // PACKAGE		:   GL //Add TaniaP
    // FUNCTION 	:   TCMatrixFunc::Mult
    // DESCRIPTION	:   Multiplying of a scalar on a matrix
    // INPUTS		:	Reference to scalar C, reference to matrix M for multiplication,
    //              :   reference to resulting matrix A_out
    // RETURNS		:	None
    void Mult(double &C, TMatrix<_size_m> &M, TMatrix<_size_m> &A_out);

    // PACKAGE		:   GL //Add TaniaP
    // FUNCTION 	:   TCMatrixFunc::Negative
    // DESCRIPTION	:   Multiplying of a matrix on -1
    // INPUTS		:	Reference to matrix M, reference to resulting matrix A_out
    // RETURNS		:	None
    void Negative(TMatrix<_size_m> &M, TMatrix<_size_m> &A_out);

    // PACKAGE		:   GL //Add TaniaP
    // FUNCTION 	:   TCMatrixFunc::Negative
    // DESCRIPTION	:   Multiplying of a vector on -1
    // INPUTS		:	Reference to vector V, reference to resulting vector A_out
    // RETURNS		:	None
    void Negative(TVector<_size_m> &V, TVector<_size_m> &A_out);

    // PACKAGE		:   GL //Add TaniaP
    // FUNCTION 	:   TCMatrixFunc::Submatrix
    // DESCRIPTION	:   Extracting a submatrix from a matrix
    // INPUTS		:	Reference to matrix A,
    //                  r1, r2 - 1st and last row in the matrix M to extract a submatrix,
    //                  c1, c2 - 1st and last column in the matrix M to extract a submatrix
    //                  reference to resulting matrix A_out
    // RETURNS		:	True if extracting is ok
    bool Submatrix(TMatrix<_size_m> &M, int r1, int r2, int c1, int c2, TMatrix<_size_m> &A_out);

    // PACKAGE      :   GL //Add TaniaP
    // FUNCTION		:   TCMatrixFunc::CholeskyDecomposition
    // DESCRIPTION	:   Cholesky decomposition of matrix
    // INPUTS		:   Input matrix
    // RETURNS		:   Output matrix ("square root" of input matrix);    
    //              :   returns true if result is ok    
    bool CholeskyDecomposition(TMatrix<_size_m> &A_inp, TMatrix<_size_m> &L_out);

    // PACKAGE      :   GL //Add TaniaP
    // FUNCTION		:   TCMatrixFunc::CholeskyDecomposition
    // DESCRIPTION	:   Cholesky decomposition of matrix
    // INPUTS		:   Input matrix
    // RETURNS		:   Output matrix ("square root" of input matrix);
    //              :   determinant of the output matrix;
    //              :   returns true if result is ok
    bool CholeskyDecomposition(TMatrix<_size_m> &A_inp, TMatrix<_size_m> &L_out, double &DetCh);

    // PACKAGE      :   GL
    // FUNCTION		:   TCMatrixFunc::IminusMatrix
    // DESCRIPTION	:   Identity matrix minus M
    // INPUTS		:   Input matrix M
    // RETURNS		:   Output matrix IminusM;
    //              :   returns true if result is ok
    bool IminusMatrix(TMatrix<_size_m> &M, TMatrix<_size_m> &IminusM);

    // PACKAGE      :   GL
    // FUNCTION		:   TCMatrixFunc::OuterProduct
    // DESCRIPTION	:   outer product of two vectors. M[i][j]=V1[i]*V2[j]
    // INPUTS		:   Input vectors V1 and V2
    // RETURNS		:   Output matrix M;
    //              :   returns true if result is ok
    bool OuterProduct(TVector<_size_m> &V1, TVector<_size_m> &V2,TMatrix<_size_m> &M);

////======================================================================================
	//Add DiVA
	//PACKAGE		:GL
    //FUNCTION		:TCMatrixFunc::VecXVec
	//DESCRIPTION	:Multiplication of two 3-element vectors
	//INPUTS		:V1 - first vector
	//				:V2 - second vector
	//RETURN		:a - resulting number (matrix 1x1)
	double VecXVec(Vector3 &V1, Vector3 &V2);

	//Add DiVA
	//PACKAGE		:GL
    //FUNCTION		:TCMatrixFunc::VecXVec
	//DESCRIPTION	:Multiplication of two 3-element vectors
	//INPUTS		:V1 - first vector
	//				:V2 - second vector
	//RETURN		:a - resulting number (matrix 1x1)
    double VecXVec(TVector<_size_m> &V1, TVector<_size_m> &V2);

	//Add DiVA
	//PACKAGE		:GL
    //FUNCTION		:TCMatrixFunc::VecXVec_vect
	//DESCRIPTION	:Vector multiplication of two 3-element vectors
	//INPUTS		:V1 - first vector
	//				:V2 - second vector
	//				:a - resulting number (vector 3-element)
	//RETURN		:1 if the multiplication was successful, 0 otherwise
	bool VecXVec_vect(Vector3 &V1, Vector3 &V2, Vector3 &A);

	//Add DiVA
	//PACKAGE		:GL
    //FUNCTION		:TCMatrixFunc::VecXVec_vect
	//DESCRIPTION	:Vector multiplication of two 3-element vectors
	//INPUTS		:V1 - first vector
	//				:V2 - second vector
	//				:a - resulting number (vector 3-element)
	//RETURN		:1 if the multiplication was successful, 0 otherwise
    bool VecXVec_vect(TVector<_size_m> &V1, TVector<_size_m> &V2, TVector<_size_m> &A);

	//Add DiVA
	//PACKAGE		:GL
    //FUNCTION		:TCMatrixFunc::VecXVec_vect
	//DESCRIPTION	:Vector multiplication of two 3-element vectors
	//INPUTS		:V1 - first vector
	//				:V2 - second vector
	//RETURN		:a - resulting number (vector 3-element)
//	Vector3 VecXVec_vect(Vector3 &V1, Vector3 &V2);

	//Add DiVA
	//PACKAGE		:GL
    //FUNCTION		:TCMatrixFunc::VecXVec_vect
	//DESCRIPTION	:Vector multiplication of two vectors
	//INPUTS		:V1 - first vector
	//				:V2 - second vector
	//RETURN		:a - resulting number
    TVector<_size_m> VecXVec_vect(TVector<_size_m> &V1, TVector<_size_m> &V2);

	//Add DiVA
	//PACKAGE		:GL
    //FUNCTION		:TCMatrixFunc::Scalar of vector
	//DESCRIPTION	:
	//INPUTS		:V1 - first vector
	//				:a - resulting number (scalar of vector)
	//RETURN		:1 if the multiplication was successful, 0 otherwise
	bool ScalarVector(Vector3 &V1, double &a);

	//Add DiVA
	//PACKAGE		:GL
    //FUNCTION		:TCMatrixFunc::Scalar of vector
	//DESCRIPTION	:
	//INPUTS		:V1 - first vector
	//				:a - resulting number (scalar of vector)
	//RETURN		:1 if the multiplication was successful, 0 otherwise
    bool ScalarVector(TVector<_size_m> &V1, double &a);

	//Add DiVA
	//PACKAGE		:GL
    //FUNCTION		:TCMatrixFunc::Scalar of vector
	//DESCRIPTION	:
	//INPUTS		:V1 - first vector
	//RETURN		:a - resulting number (scalar of vector)
	double ScalarVector(Vector3 &V1);

	//Add DiVA
	//PACKAGE		:GL
    //FUNCTION		:TCMatrixFunc::Scalar of vector
	//DESCRIPTION	:
	//INPUTS		:V1 - first vector
	//RETURN		:a - resulting number (scalar of vector)
    double ScalarVector(TVector<_size_m> &V1);
////============================================================================================

	// PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::Det
	// DESCRIPTION	:   Calculate determinant of matrix - 3x3
	// INPUTS		:	Reference to matrix M, reference to resulting number d3
	// RETURNS		:	NONE
	void Det(Matrix3x3 &M, double &d3);

	double Det(Matrix3x3 &M);

	// PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::Det
	// DESCRIPTION	:   Calculate determinant of matrix
	// INPUTS		:	Reference to matrix M, reference to resulting number det
	// RETURNS		:	true if determinant was found
    bool Det(TMatrix<_size_m> &M, double &det);

    // PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::Det
    // DESCRIPTION	:   Calculate determinant of matrix
    // INPUTS		:	Reference to matrix M
    // RETURNS		:	resulting value of determinant
    double Det(TMatrix<_size_m> &M);

	// PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::Add
	// DESCRIPTION	:   Addition of matrices - common
	// INPUTS		:	Reference to matrix M1, reference to matrix M2 for addition,
	//					reference to resulting matrix A
	// RETURNS		:	true if addition is ok
    bool Add(TMatrix<_size_m> &M1, TMatrix<_size_m> &M2, TMatrix<_size_m> &A);

    // PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::Subtr
    // DESCRIPTION	:   Subtraction of matrices - common
    // INPUTS		:	Reference to matrix M1, reference to matrix M2 for subtraction,
    //					reference to resulting matrix A
    // RETURNS		:	true if subtraction is ok
    bool Subtr(TMatrix<_size_m> &M1, TMatrix<_size_m> &M2, TMatrix<_size_m> &A);

    // PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::Add
    // DESCRIPTION	:   Addition of matrices - common
    // INPUTS		:	Reference to matrix M1, reference to matrix M2 for addition	//
    // RETURNS		:	resulting matrix
    TMatrix<_size_m> Add(TMatrix<_size_m> &M1, TMatrix<_size_m> &M2);

	// PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::Add
	// DESCRIPTION	:   Addition of matrices - 3x3
	// INPUTS		:	Reference to matrix M1, reference to matrix M2 for addition,
	//					reference to resulting matrix A
	// RETURNS		:	true if addition is ok
	bool Add(Matrix3x3 &M1, Matrix3x3 &M2, Matrix3x3 &A);

////==================================================================================
	//Add DiVA
	//PACKAGE		:GL
    //FUNCTION		:TCMatrixFunc::Add
	//DESCRIPTION	:Addition of vectors
	//INPUTS		:V1 - first vector
	//				:V2 - second vector
	//RETURN		:A - resulting vector
    TVector<_size_m> Add(TVector<_size_m>& V1, TVector<_size_m>& V2);

    //Add TaniaP
    //PACKAGE		:GL
    //FUNCTION		:TCMatrixFunc::Add
    //DESCRIPTION	:Addition of vectors
    //INPUTS		:V1 - first vector
    //				:V2 - second vector
    //              :A_out - resulting vector
    //RETURN		:true if addition is ok
    bool Add(TVector<_size_m>& V1, TVector<_size_m>& V2, TVector<_size_m> &A_out);

    //PACKAGE		:GL
    //FUNCTION		:TCMatrixFunc::Subtr
    //DESCRIPTION	:Subtraction of vectors
    //INPUTS		:V1 - first vector
    //				:V2 - second vector
    //              :A_out - resulting vector
    //RETURN		:true if subtraction is ok
    bool Subtr(TVector<_size_m>& V1, TVector<_size_m>& V2, TVector<_size_m> &A_out);


	//Add DiVA
	//PACKAGE		:GL
    //FUNCTION		:TCMatrixFunc::Add
	//DESCRIPTION	:Addition of vectors (3x1)
	//INPUTS		:V1 - first vector
	//				:V2 - second vector
	//RETURN		:A - resulting vector
	//Vector3 Add(Vector3& V1, Vector3& V2);
////==================================================================================

	// PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::Transpon
	// DESCRIPTION	:   Matching of a matrix (transpon.)
	// INPUTS		:	Reference to matrix M1, reference to resulting matrix A
	// RETURNS		:	true if transpon. is ok

    bool Transpon(TMatrix<_size_m> &M1, TMatrix<_size_m> &A);

	// PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::Transpon
	// DESCRIPTION	:   Matching of a matrix 3x3
	// INPUTS		:	Reference to matrix M1, reference to resulting matrix A
	// RETURNS		:	true if transpon. is ok

	bool Transpon(Matrix3x3 &M1, Matrix3x3 &A);

	//PACKAGE		:GL
    //FUNCTION		:TCMatrixFunc::Transpon
	//DESCRIPTION	:Transposing a matrix (general)
	//INPUTS		:M1 - source matrix
	//RETURN		:A - transposed matrix
    TMatrix<_size_m> Transpon(TMatrix<_size_m> &M1);


////==================================================================================
    //concatenation //Add TaniaP

    //PACKAGE		:GL
    //FUNCTION		:TCMatrixFunc::ConcatLeftRight
    //DESCRIPTION	:Concatenation of matrices left to right
    //INPUTS		:References to MLeft, MRight - source matrices,
    //      		:reference to A - resulting concatenated matrix
    //RETURN        :true if concatenation is ok
    bool ConcatLeftRight(TMatrix<_size_m> &MLeft, TMatrix<_size_m> &MRight, TMatrix<_size_m> &A);

    //PACKAGE		:GL
    //FUNCTION		:TCMatrixFunc::ConcatUpDown
    //DESCRIPTION	:Concatenation of matrices up to down
    //INPUTS		:References to MUp, MDown - source matrices,
    //      		:reference to A - resulting concatenated matrix
    //RETURN        :true if concatenation is ok
    bool ConcatUpDown(TMatrix<_size_m> &MUp, TMatrix<_size_m> &MDown, TMatrix<_size_m> &A);

    //PACKAGE		:GL
    //FUNCTION		:TCMatrixFunc::ConcatLeftRight
    //DESCRIPTION	:Concatenation of matrices left to right
    //INPUTS		:References to MLeft, MCenter, MRight - source matrices,
    //      		:reference to A - resulting concatenated matrix
    //RETURN        :true if concatenation is ok
    bool ConcatLeftRight(TMatrix<_size_m> &MLeft, TMatrix<_size_m> &MCenter, TMatrix<_size_m> &MRight, TMatrix<_size_m> &A);

    //PACKAGE		:GL
    //FUNCTION		:TCMatrixFunc::ConcatUpDown
    //DESCRIPTION	:Concatenation of matrices up to down
    //INPUTS		:References to MUp, MCenter, MDown - source matrices,
    //      		:reference to A - resulting concatenated matrix
    //RETURN        :true if concatenation is ok
    bool ConcatUpDown(TMatrix<_size_m> &MUp, TMatrix<_size_m> &MCenter, TMatrix<_size_m> &MDown, TMatrix<_size_m> &A);

    //PACKAGE		:GL
    //FUNCTION		:TCMatrixFunc::Blkdiag2
    //DESCRIPTION	:Concatenate 2 matrices to a block diagonal matrix. (input matrices are not necessary to be square)
    //INPUTS		:References to M1, M2 - source matrices,
    //      		:reference to A - resulting concatenated matrix
    //RETURN        :true if concatenation is ok
    template<int _size_m1, int _size_m2, int _size_m3>
    bool Blkdiag2(TMatrix<_size_m1> &M1, TMatrix<_size_m2> &M2, TMatrix<_size_m3> &A);

    //PACKAGE		:GL
    //FUNCTION		:TCMatrixFunc::Blkdiag3
    //DESCRIPTION	:Concatenate 3 matrices to a block diagonal matrix. (input matrices are not necessary to be square)
    //INPUTS		:References to M1, M2, M3 - source matrices,
    //      		:reference to A - resulting concatenated matrix
    //RETURN        :true if concatenation is ok
    template<int _size_m1, int _size_m2, int _size_m3, int _size_m4>
    bool Blkdiag3(TMatrix<_size_m1> &M1, TMatrix<_size_m2> &M2, TMatrix<_size_m3> &M3, TMatrix<_size_m4> &A);

    //PACKAGE		:GL
    //FUNCTION		:TCMatrixFunc::Blkdiag3
    //DESCRIPTION	:Concatenate 4 matrices to a block diagonal matrix. (input matrices are not necessary to be square)
    //INPUTS		:References to M1, M2, M3, M4 - source matrices,
    //      		:reference to A - resulting concatenated matrix
    //RETURN        :true if concatenation is ok
    template<int _size_m1, int _size_m2, int _size_m3, int _size_m4, int _size_m5>
    bool Blkdiag4(TMatrix<_size_m1> &M1, TMatrix<_size_m2> &M2, TMatrix<_size_m3> &M3, TMatrix<_size_m4> &M4, TMatrix<_size_m5> &A);

    //PACKAGE		:GL
    //FUNCTION		:TCMatrixFunc::Copy
    //DESCRIPTION	:Copy From to To, if size(_size_m2) of From is less or equal to size of To(_size_m)
    //INPUTS		:Reference to TVector To and TVector From,
    //RETURN        :true if ok
    template<int _size_m1, int _size_m2>
    bool Copy(TVector<_size_m1> &To, TVector<_size_m2> &From);

    //PACKAGE		:GL
    //FUNCTION		:TCMatrixFunc::Copy
    //DESCRIPTION	:Copy From to To, if size(_size_m2) of From is less or equal to size of To(_size_m)
    //INPUTS		:Reference to TMatrix To and TMatrix From,
    //RETURN        :true if ok
    template <int _size_m1, int _size_m2>
    bool Copy(TMatrix<_size_m1> &To, TMatrix<_size_m2> &From);

    //PACKAGE		:GL
    //FUNCTION		:TCMatrixFunc::Copy
    //DESCRIPTION	:
    //INPUTS		:
    //RETURN        :true if ok
    //template <int _size_m1, int str, int col>
    //bool Copy(TMatrix<_size_m1> &To, double (&From)[str][col]);

    //PACKAGE		:GL
    //FUNCTION		:TCMatrixFunc::Copy
    //DESCRIPTION	:Copying
    //INPUTS		:the matrix (vector) inth which we copy;
    //              :the matrix (vector) from which we copy
    //RETURN        :true if ok
    template <int _size_m1, int col>
    bool Copy(TVector<_size_m1> &To, const double (&From)[col]);

    template <int _size_m1, int col>
    bool Copy(TVector<_size_m1> &To, double (&From)[col]);

    template <int _size_m1,int _size_m2>
    bool Copy(TVector<_size_m1> &To, TVector<_size_m2> &From, int c_from, int c_to);

    template <int _size_m1,int _size_m2>
    bool Copy(TMatrix<_size_m1> &To, TMatrix<_size_m2> &From, int r_from, int r_to, int c_from, int c_to);

    template <int _size_m1, int _size_m2>
    bool Copy(TMatrix<_size_m1> &To, TVector<_size_m2> &From);

    template <int _size_m1, int _size_m2>
    bool Copy(TMatrix<_size_m1> &To, TVector<_size_m2> &From, int r_from, int r_to);

    template <int _size_m1, int _size_m2>
    bool Copy(TVector<_size_m1> &To, TMatrix<_size_m2> &From);

    template <int _size_m1, int _size_m2>
    bool Copy(TVector<_size_m1> &To, TMatrix<_size_m2> &From, int n_from, int n_to);

    //PACKAGE		:GL
    //FUNCTION		:TCMatrixFunc::Copy
    //DESCRIPTION	:copy constant double array to matrix
    //INPUTS		:Reference to source array From[row][col], reference to resulting matrix To
    //RETURN        :true if ok
    template <int row, int col>
    bool Copy(TMatrix<_size_m> &To, const double (&From)[row][col]);

    template <int row, int col>
    bool Copy(TMatrix<_size_m> &To, double (&From)[row][col]);

    //PACKAGE		:GL
    //FUNCTION		:TCMatrixFunc::GramMatrix
    //DESCRIPTION	:Compute Gramian Matrix of vector V. Similar to OuterProduct.
    //INPUTS		:Reference to source vector V, reference to resulting matrix M
    //RETURN        :true if ok
    bool GramMatrix(TVector<_size_m> &V, TMatrix<_size_m> &M);

    //PACKAGE		:GL
    //FUNCTION		:TCMatrixFunc::CalcDet3x3
    //DESCRIPTION	:Calculates determinant for symmetric matrix 3x3, formed from rows numbered i0,i0+i,i0+2
    //              :and columns numbered i0,i0+1,i0+2
    //INPUTS		:Given matrix; number i0
    //RETURN        :Determinant for symmetric matrix 3x3, formed from rows numbered i0,i0+i,i0+2
    //              :and columns numbered i0,i0+1,i0+2
    double CalcDetSymm3x3(TMatrix<_size_m> &M, int i0);

    //PACKAGE		:GL
    //FUNCTION		:TCMatrixFunc::CalcDet2x2
    //DESCRIPTION	:Calculates determinant for symmetric matrix 2x2, formed from rows numbered i0,i0+i
    //              :and columns numbered i0,i0+1
    //INPUTS		:Given matrix; number i0
    //RETURN        :Determinant for symmetric matrix 2x2, formed from rows numbered i0,i0+i
    //              :and columns numbered i0,i0+1
    double CalcDetSymm2x2(TMatrix<_size_m> &M, int i0);

    //PACKAGE		:GL
    //FUNCTION		:TCMatrixFunc::Combine2Mod
    //DESCRIPTION	:Compute combination of 2 models of smoothed vectors
    //              :and covariance matrices, formed by IMM filter
    //INPUTS		:References to source vectors Alpha1, Alpha2 of 1st and 2nd models;
    //              :references to source covariance matrices R1, R2 of 1st and 2nd models;
    //              :reference to the vector of probabilities;
    //              :references to resulting combined vector Alpha comb and covariance matrix RComb
    //RETURN        :true if ok
    template <int _size_m1, int _size_m2>
    bool Combine2Mod(TVector<_size_m1> &Alpha1, TVector<_size_m2> &Alpha2,
                     TMatrix<_size_m1> &R1, TMatrix<_size_m2> &R2,
                     TVector<2> &ProbAposteriori,
                     TVector<_size_m> &AlphaComb, TMatrix<_size_m> &RComb);

    //PACKAGE		:GL
    //FUNCTION		:TCMatrixFunc::Combine3Mod
    //DESCRIPTION	:Compute combination of 3 models of smoothed vectors
    //              :and covariance matrices, formed by IMM filter
    //INPUTS		:References to source vectors Alpha1, Alpha2, Alpha3 of 1st, 2nd and 3rd models;
    //              :references to source covariance matrices R1, R2, R3 of 1st, 2nd and 3rd models;
    //              :reference to the vector of probabilities;
    //              :references to resulting combined vector Alpha comb and covariance matrix RComb
    //RETURN        :true if ok
    template <int _size_m1, int _size_m2, int _size_m3>
    bool Combine3Mod(TVector<_size_m1> &Alpha1, TVector<_size_m2> &Alpha2, TVector<_size_m3> &Alpha3,
                     TMatrix<_size_m1> &R1, TMatrix<_size_m2> &R2, TMatrix<_size_m3> &R3,
                     TVector<3> &ProbAposteriori,
                     TVector<_size_m> &AlphaComb, TMatrix<_size_m> &RComb);

    //PACKAGE		:GL
    //FUNCTION		:TCMatrixFunc::TransformTo2D
    //DESCRIPTION	:Transform given 3x3, 6x6 or 9x9 matrix to the 2x2 or 4x4 or 6x6
    //              :matrix, deleting rows and columns with number 2, 5, 7
    //INPUTS		:Reference to the result matrix To;
    //              :reference to the given matrix From
    //RETURN        :true if ok
    template <int _size_m1,int _size_m2>
    bool TransformTo2D(TMatrix<_size_m1> &To, TMatrix<_size_m2> &From);

    //PACKAGE		:GL
    //FUNCTION		:TCMatrixFunc::TransformTo2D
    //DESCRIPTION	:Transform given 3, 6 or 9 dimension vector to the 2 or 4 or 6 dimension
    //              :vector, deleting elements with number 2, 5, 7
    //INPUTS		:Reference to the result vector To;
    //              :reference to the given vector From
    //RETURN        :true if ok
    template <int _size_m1,int _size_m2>
    bool TransformTo2D(TVector<_size_m1> &To, TVector<_size_m2> &From);

    ////==================================================================================

    //PACKAGE		: GL
    //FUNCTION		: TCMatrixFunc::ExtrapolateMatrLin
    //DESCRIPTION	: Linear extrapolation for covariance matrix
    //INPUTS		: Reference to source matrix M; time interval;
    //              : reference to resulting matrix M
    //RETURN        : True if ok
    bool ExtrapolateMatrLin(TMatrix<_size_m> &M, double dt, TMatrix<_size_m> &MExtr);

    //PACKAGE		: GL
    //FUNCTION		: TCMatrixFunc::ExtrapolateMatrQuadr
    //DESCRIPTION	: Quadratic extrapolation for covariance matrix
    //INPUTS		: Reference to source matrix M; time interval;
    //              : reference to resulting matrix M
    //RETURN        : True if ok
    bool ExtrapolateMatrQuadr(TMatrix<_size_m> &M, double dt, TMatrix<_size_m> &MExtr);

    ////==================================================================================

    // PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::Matching
    // DESCRIPTION	:   Matching of a vector to a 1 column matrix (transform from structure Vector to structrue Matrix)
    // INPUTS		:	Reference to source vector V1, reference to resulting matrix A
    // RETURNS		:	true if matching is ok
    bool Matching(TVector<_size_m> &V1, TMatrix<_size_m> &A);

    // PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::Matching
    // DESCRIPTION	:   Matching of a 1 column matrix to a vector (transform from structure Matrix to structrue Vector)
    // INPUTS		:	Reference to source matrix M1, reference to resulting vector A
    // RETURNS		:	true if matching is ok
    bool Matching(TMatrix<_size_m> &M1, TVector<_size_m> &A);

////==================================================================================

	// PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::Reset
	// DESCRIPTION	:   Fill in matrices  by zero
	// INPUTS		:	NONE
	// RETURNS		:	NONE
	void Reset()
	{
		m_tmp1.Reset();
		m_tmp2.Reset();
		memset(rearr, 0, sizeof(rearr));
	}

protected:

	// PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::main_el
	// DESCRIPTION	:   Choice of a main unit
	// INPUTS		:	Reference to matrix A, reference to matrix E, dimension of matrix A
	// RETURNS		:	NONE
    void main_el(TMatrix<_size_m> &A, TMatrix<_size_m> &E, int k);

	// PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::Inv
	// DESCRIPTION	:   Retrace of Gauss' method
	// INPUTS		:	Reference to matrix A, reference to matrix E, reference to resulting matrix R
	// RETURNS		:	NONE
    void Inv(TMatrix<_size_m> &A, TMatrix<_size_m> &E, TMatrix<_size_m> &R);

    // PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::Inv_Gauss
    // DESCRIPTION	:   Gauss' method of the inverse matrix calculation, optimized version
    // INPUTS		:	Reference to input matrix M, reference to resulting matrix InvM, reference to sign of small determinant
    // RETURNS		:	True if result is OK
    bool Inv_Gauss(TMatrix<_size_m> &M, TMatrix<_size_m> &InvM, bool &bSmallDet);

    // PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::Inv_Gauss
    // DESCRIPTION	:   Gauss' method of the inverse matrix calculation, optimized version
    // INPUTS		:	Reference to the input matrix M; reference to the resulting matrix InvM;
    //              :   reference to the sign of small determinant; reference to the determinant of M
    // RETURNS		:	True if result is OK
    bool Inv_Gauss(TMatrix<_size_m> &M, TMatrix<_size_m> &InvM, bool &bSmallDet, double &DetM);

	// PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::det_rec
	// DESCRIPTION	:   The recurrence procedure of calculation of the determinant
	// INPUTS		:	Reference to matrix A, reference to vector V, number of recurrence circulation
	// RETURNS		:	Determinant
    double det_rec(const TMatrix<_size_m> &A, TVector<_size_m> &V, int n);

    // PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixFunc::det_Gauss
    // DESCRIPTION	:   The calculation of the determinant using method of Gauss
    // INPUTS		:	Reference to matrix M, reference to determinant value &det
    // RETURNS		:	True if result is ok
    bool det_Gauss(TMatrix<_size_m> &M, double &det);


    TMatrix<_size_m> m_tmp1,m_tmp2;			// Auxiliary matrices
    int rearr[_size_m];						//rearrangement of components of the given matrix
};


//PACKAGE           :	GL
//TEMPLATE OF CLASS	:	TCMatrixMatching
//DESCRIPTION       :	Template of class containing the functions for matching matrix from one size to the other
template <int _size_m1, int _size_m2>
class TCMatrixMatching
{
public:
    // PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixMatching::Matching
    // DESCRIPTION	:   Matching of a matrices (transform from structure Matrix with _size_m1 to structrue Matrix with _size_m2)
    // INPUTS		:	Reference to source matrix M1, reference to resulting matrix A_out
    // RETURNS		:	true if matching is ok
    bool Matching(TMatrix<_size_m1> &M1, TMatrix<_size_m2> &A_out)
    {
        if (M1.s_m > _size_m2 || M1.s_n > _size_m2)
            return false;

        A_out.Reset();

        A_out.s_m = M1.s_m;
        A_out.s_n = M1.s_n;
        for (int i=0; i<M1.s_m; i++)
            for (int j=0; j<M1.s_n; j++)
                A_out.M[i][j] = M1.M[i][j];

        return true;
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   TCMatrixMatching::Matching
    // DESCRIPTION	:   Matching of a vectors (transform from structure Vector with _size_m1 to structrue Vector with _size_m2)
    // INPUTS		:	Reference to source vector V1, reference to resulting vector A_out
    // RETURNS		:	true if matching is ok
    bool Matching(TVector<_size_m1> &V1, TVector<_size_m2> &A_out)
    {
        if (V1.s > _size_m2)
            return false;

        A_out.Reset();

        A_out.s = V1.s;
        for (int i=0; i<V1.s; i++)
            A_out.Vec[i] = V1.Vec[i];

        return true;
    }
};


namespace GLFileLog
{
    // PACKAGE		:   GL
    // FUNCTION 	:   GLFileLog::LogMatrix
    // DESCRIPTION	:   Output matrix and given time to the log file
    // INPUTS		:	Specified file; matrix name; time; given matrix
    // RETURNS		:	None
    template <int _size_m>
    void LogMatrix(FILE* _pLogFile, const char* _Name, double &Time, const TMatrix<_size_m> &Matr);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLFileLog::tLogMatrix
    // DESCRIPTION	:   Output matrix and current time to the log file
    // INPUTS		:	Specified file; matrix name; given matrix
    // RETURNS		:	None
    template <int _size_m>
    void tLogMatrix(FILE* _pLogFile, const char* _Name, const TMatrix<_size_m> &Matr);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLFileLog::tLogMatrixToLine
    // DESCRIPTION	:   Output matrix and current time to the log file, all elements of the upper triangle
    //              :   in one line: diagonal, 1st row above the diagonal, 2nd row above the diagonal etc.
    // INPUTS		:	Specified file; matrix name; given matrix; sign of text abscence;
    //              :   sign of precise output
    // RETURNS		:	None
    template <int _size_m>
    void tLogMatrixToLine(FILE* _pLogFile, const char* _Name, const TMatrix<_size_m> &Matr, bool bNoText=false, bool bPrecise=false);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLFileLog::OutMatrixToString
    // DESCRIPTION	:   Output matrix to the string, all elements of the upper triangle
    //              :   in one line: diagonal, 1st row above the diagonal, 2nd row above the diagonal etc.
    // INPUTS		:	Matrix name; given matrix; size of output data; reference to the output string;
    //              :   sign of precise output
    // RETURNS		:	None
    template <int _size_m>
    void OutMatrixToString(const char* _Name, const TMatrix<_size_m> &Matr, const int SizeOut, char*& _OutString, bool bPrecise=false);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLFileLog::LogVector
    // DESCRIPTION	:   Output vector and given time to the log file
    // INPUTS		:	Specified file; vector name; time; given vector
    // RETURNS		:	None
    template <int _size_m>
    void LogVector(FILE* _pLogFile, const char* _Name, double &Time, const TVector<_size_m> &Vec);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLFileLog::tLogVector
    // DESCRIPTION	:   Output vector and current time to the log file
    // INPUTS		:	Specified file; vector name; given vector
    // RETURNS		:	None
    template <int _size_m>
    void tLogVector(FILE* _pLogFile, const char* _Name, const TVector<_size_m> &Vec);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLFileLog::tLogVectorToLine
    // DESCRIPTION	:   Output vector and current time to the log file
    // INPUTS		:	Specified file; vector name; given vector
    // RETURNS		:	None
    template <int _size_m>
    void tLogVectorToLine(FILE* _pLogFile, const char* _Name, const TVector<_size_m> &Vec);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLFileLog::OutVectorToString
    // DESCRIPTION	:   Output vector to the output string
    // INPUTS		:	Vector name; given vector; size of output data; reference to the output string
    // RETURNS		:	None
    template <int _size_m>
    void OutVectorToString(const char* _Name, const TVector<_size_m> &Vec, const int SizeOut, char*& _OutString);
}


#include "GLMatrix_tmpl.h"

#endif
