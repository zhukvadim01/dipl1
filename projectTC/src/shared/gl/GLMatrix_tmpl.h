
#include "GLMatrix.h"

#include <cstdlib>

template <int _size_m>
TVector<_size_m>::TVector()
{
    Reset();
    //NOTE: do not write "s=_size_m" in this place. This can cause computation errors.
    //_size_m is the size of array in memory;
    //s is the size of vector in the mathematical sense, s may vary during calculations.
}


template <int _size_m>
TVector<_size_m>::TVector(int size)
{
    Reset(size);
}


template <int _size_m>
void TVector<_size_m>::Reset()
{
    s = 0; //NOTE: do not write "s=_size_m" in this place. This can cause computation errors.
           //_size_m is the size of array in memory;
           //s is the size of vector in the mathematical sense, s may vary during calculations.
    for (int i=0; i<_size_m; i++)
    {
        Vec[i] = 0;
    }
}


template <int _size_m>
void TVector<_size_m>::Reset(int size)
{
    s = size;
    for (int i=0; i<_size_m; i++)
    {
        Vec[i] = 0;
    }
}

template <int _size_m>
TVector<_size_m>& TVector<_size_m>::operator=(const TVector<_size_m>& Q)
{
    if (s == 0)
    {
        s = Q.s;
    }

    if (s == Q.s && Q.s > 0 && Q.s <= _size_m)
    {
        for (int i = 0; i < s; i++)
        {
            Vec[i] = Q.Vec[i];
        }
    }
    return	*this;
}


template <int _size_m>
TVector<_size_m>& TVector<_size_m>::operator+=(const TVector<_size_m>& Q)
{
    if (s == 0)
    {
        s = Q.s;
    }

    if (s == Q.s && Q.s > 0 && Q.s <= _size_m)
    {
        for (int i = 0; i < s; i++)
        {
            Vec[i] += Q.Vec[i];
        }
    }
    return	*this;
}


template <int _size_m>
double& TVector<_size_m>::operator[](int ind)
{
    if (ind >= 0 && ind < _size_m)
    {
        return Vec[ind];
    }
    return Vec[0];
}


template <int _size_m>
TVector<_size_m> TVector<_size_m>::PickOut(int from, int to, bool &ItsOk)
{
    TVector<_size_m> pick;

    if (from >= 0 && from < s && to>=from && to<s)
    {
        int S=to-from;
        pick.s=S+1;
        for(int i=0; i<=S;i++)
        {
            pick.Vec[i]=this->Vec[i+from];
        }
        ItsOk=true;
    }
    else ItsOk=false;

    return pick;
}


template <int _size_m>
bool TVector<_size_m>::IsZero()
{
    bool bIsZero = true;
    if (s > 0 && s <= _size_m)
    {
        for (int i=0; i<s; i++)
        {
            if (fabs(Vec[i]) > EPSILON_M)
            {
                bIsZero = false;
                break;
            }
        }
    }
    return bIsZero;
}


template <int _size_m>
TVector<_size_m> TVector<_size_m>::operator+(const TVector<_size_m> &Q) const
{
    TVector <_size_m> Res;
    Res.s = std::max(this->s, Q.s);
    if (Res.s > 0 && Res.s <= _size_m)
    {
        for (int i=0; i<Res.s; i++)
        {
            Res.Vec[i] = this->Vec[i] + Q.Vec[i];
        }
    }
    return Res;
}


template <int _size_m>
TVector<_size_m> TVector<_size_m>::operator-(const TVector<_size_m> &Q) const
{
    TVector<_size_m> Res;
    Res.s = std::max(this->s, Q.s);
    if (Res.s > 0 && Res.s <= _size_m)
    {
        for (int i=0; i<Res.s; i++)
        {
            Res.Vec[i] = this->Vec[i] - Q.Vec[i];
        }
    }
    return Res;
}


template <int _size_m>
TVector<_size_m> TVector<_size_m>::operator*(const double &mult) const
{
    TVector<_size_m> Res;
    Res.s = this->s;
    if (Res.s > 0 && Res.s <= _size_m)
    {
        for (int i=0; i<Res.s; i++)
        {
            Res.Vec[i] = this->Vec[i] * mult;
        }
    }
    return Res;
}


template <int _size_m>
void TVector<_size_m>::SetDimension(int _dim)
{
    s = _dim;
}


template <int _size_m>
double TVector<_size_m>::Module()
{
    double Mod = 0.;
    if (s > 0 && s <= _size_m)
    {
        for (int i=0; i<s; i++)
        {
            Mod += sqr(Vec[i]);
        }
    }
    if (Mod > 0)
    {
        Mod = sqrt(Mod);
    }
    return Mod;
}


template <int _size_m>
double TVector<_size_m>::ScalProd(TVector<_size_m> &u)
{
    double scal_prod = 0;
    if (s > 0 && s <= _size_m && s == u.s)
    {
        for (int i=0; i<s; i++)
        {
            scal_prod += Vec[i] * u[i];
        }
    }
    return scal_prod;
}


template <int _size_m>
TMatrix<_size_m>::TMatrix()
{
    Reset();
    //NOTE: do not write "s_m=_size_m", "s_n=_size_m" in this place. This can cause computation errors.
    //_size_m*_size_m is the size of array in memory;
    //s_m, s_n are the sizes of matrix in the mathematical sense, s_m and s_n may vary during calculations.
}


template <int _size_m>
TMatrix<_size_m>::TMatrix(int _str, int _col)
{
    Reset(_str, _col);
}


template <int _size_m>
void TMatrix<_size_m>::Reset()
{
    s_m = 0; //NOTE: do not write "s_m=_size_m", "s_n=_size_m" in this place. This can cause computation errors.
    s_n = 0; //_size_m*_size_m is the size of array in memory;
             //s_m, s_n are the sizes of matrix in the mathematical sense, s_m and s_n may vary during calculations.
    memset(&M,0,sizeof(M));
//    for(int i = 0; i < _size_m; i++)
//    {
//        for (int j = 0; j < _size_m; j++)
//        {
//            M[i][j] = 0;
//        }
//    }
}


template <int _size_m>
void TMatrix<_size_m>::Reset(int _str, int _col)
{
    s_m = _str;
    s_n = _col;
    memset(&M,0,sizeof(M));
//    for(int i = 0; i < _size_m; i++)
//    {
//        for (int j = 0; j < _size_m; j++)
//        {
//            M[i][j] = 0;
//        }
//    }
}


template <int _size_m>
TMatrix<_size_m>& TMatrix<_size_m>::operator=(const TMatrix<_size_m>& matr)
{
    if (s_m == 0 && s_n == 0)
    {
        s_m = matr.s_m;
        s_n = matr.s_n;
    }

    if (s_m == matr.s_m && s_n == matr.s_n
            && matr.s_m > 0 && matr.s_n > 0
            && matr.s_m <= _size_m && matr.s_n <= _size_m)
    {
        for (int i = 0; i < s_m; i++)
        {
            for (int j = 0; j < s_n; j++)
            {
                M[i][j] = matr.M[i][j];
            }
        }
    }
    return	*this;
}


template <int _size_m>
TMatrix<_size_m> TMatrix<_size_m>::operator+(const TMatrix<_size_m> &Q) const
{
    TMatrix<_size_m> Res;
    Res.s_m = std::max(this->s_m, Q.s_m);
    Res.s_n = std::max(this->s_n, Q.s_n);
    if (0 < Res.s_m && 0 < Res.s_n && Res.s_m <= _size_m && Res.s_n <= _size_m)
    {
        for (int i=0; i<Res.s_m; i++)
        {
            for (int j=0; j<Res.s_n; j++)
            {
                Res.M[i][j] = this->M[i][j] + Q.M[i][j];
            }
        }
    }
    return Res;
}


template <int _size_m>
TMatrix<_size_m> TMatrix<_size_m>::operator-(const TMatrix<_size_m> &Q) const
{
    TMatrix<_size_m> Res;
    Res.s_m = std::max(this->s_m, Q.s_m);
    Res.s_n = std::max(this->s_n, Q.s_n);
    if (0 < Res.s_m && 0 < Res.s_n && Res.s_m <= _size_m && Res.s_n <= _size_m)
    {
        for (int i=0; i<Res.s_m; i++)
        {
            for (int j=0; j<Res.s_n; j++)
            {
                Res.M[i][j] = this->M[i][j] - Q.M[i][j];
            }
        }
    }
    return Res;
}


template <int _size_m>
TMatrix<_size_m> TMatrix<_size_m>::operator*(const double &mult) const
{
    TMatrix<_size_m> Res;
    Res.Reset(this->s_m, this->s_n);
    if (0 < Res.s_m && 0 < Res.s_n && Res.s_m <= _size_m && Res.s_n <= _size_m)
    {
        for (int i=0; i<Res.s_m; i++)
        {
            for (int j=0; j<Res.s_n; j++)
            {
                Res.M[i][j] = this->M[i][j] * mult;
            }
        }
    }
    return Res;
}


template <int _size_m>
TMatrix<_size_m> TMatrix<_size_m>::PickOut(int from1, int to1, int from2, int to2, bool &ItsOk)
{
    TMatrix pick;

    if (from1 >= 0 && from1 < s_m && from2 >= 0 && from2 < s_n && to1>=from1 && to1<s_m && to2>=from2 && to2<s_n)
    {
        int S1=to1-from1;
        pick.s_m=S1+1;
        int S2=to2-from2;
        pick.s_n=S2+1;
        for(int i=0; i<=S1;i++)
        {
            for(int j=0; j<=S2;j++)
            {
                pick.M[i][j]=this->M[i+from1][j+from2];
            }
        }
        ItsOk=true;
    }
    else ItsOk=false;

    return pick;
}


template <int _size_m>
bool TMatrix<_size_m>::IsZero()
{
    bool bIsZero = true;
    if (0 < s_m && s_m <= _size_m
            && 0 < s_n && s_m <= _size_m)
    {
        for (int i=0; i<s_m; i++)
        {
            for (int j=0; j<s_n; j++)
            {
                if (fabs(M[i][j]) > ZERO_TOLERANCE_M)
                {
                    bIsZero = false;
                    break;
                }
            }
        }
    }
    return bIsZero;
}


template <int _size_m>
void TMatrix<_size_m>::SetDimensions(int _rows, int _cols)
{
    s_m = _rows;
    s_n = _cols;
}


template <int _size_m>
void TMatrix<_size_m>::ReflectNonZeroRelativeDiag()
{
    if (s_m == s_n && s_m > 1)
    {
        for (int i=0; i<s_m; i++)
        {
            for (int j=i+1; j<s_n; j++)
            {
                if (fabs(M[i][j]) >= ZERO_TOLERANCE_M)
                {
                    if (fabs(M[j][i]) < ZERO_TOLERANCE_M)
                    {
                        M[j][i] = M[i][j];
                    }
                }
                else
                {
                    if (fabs(M[j][i]) >= ZERO_TOLERANCE_M)
                    {
                        M[i][j] = M[j][i];
                    }
                }
            }
        }
    }
}


template <int _size_m>
void TMatrix<_size_m>::FCovMatrAdaptation(short Dim, double MinRMSE, double CovCorr)
{
    if (Dim > 0 && Dim <= _size_m && MinRMSE > 0 && CovCorr > 0)
    {
        if (s_m != Dim)
            s_m = Dim;
        if (s_n != Dim)
            s_n = Dim;

        int i, j;
        for (i=0; i<Dim; i++)
        {
            if (M[i][i] < sqr(MinRMSE))
                M[i][i] = sqr(MinRMSE);
        }

        for (i=0; i<Dim; i++)
        {
            for (j=0; j<Dim; j++)
            {
                if (i != j)
                {
                    double sign = 0; //signum of M[i][j]
                    if (M[i][j] > 0)
                    {
                        sign = 1;
                    }
                    else if (M[i][j] < 0)
                    {
                        sign = -1;
                    }
                    else
                    {
                    }

                    double ProdDiag = sqrt(M[i][i] * M[j][j]) - CovCorr; //sqrt(Product of diagonal elements) minus correction
                    if (fabs(M[i][j]) >  ProdDiag)
                    {
                        M[i][j] = sign * ProdDiag;
                    }
                }
            }
        }
    }
}


template<int _size_m>
double TMatrix<_size_m>::GetMax()
{
    double Result = - 1000000000;
    if (0 < s_m && s_m <= _size_m
            && 0 < s_n && s_n <= _size_m)
    {
        for (int i=0; i<s_m; i++)
        {
            for (int j=0; j<s_n; j++)
            {
                if (M[i][j] > Result)
                {
                    Result = M[i][j];
                }
            }
        }
    }
    return Result;
}


template<int _size_m>
double TMatrix<_size_m>::GetMaxAbs()
{
    double Result = 0;
    if (0 < s_m && s_m <= _size_m
            && 0 < s_n && s_n <= _size_m)
    {
        for (int i=0; i<s_m; i++)
        {
            for (int j=0; j<s_n; j++)
            {
                double CurrAbs = fabs(M[i][j]);
                if (CurrAbs > Result)
                {
                    Result = CurrAbs;
                }
            }
        }
    }
    return Result;
}


template <int _size_m>
double TMatrix<_size_m>::GetMaxDiag()
{
    double Result = - 1000000000;
    if (0 < s_m && s_m <= _size_m
            && 0 < s_n && s_n <= _size_m)
    {
        int Dim = std::min(s_m, s_n);
        for (int i=0; i<Dim; i++)
        {
            if (M[i][i] > Result)
            {
                Result = M[i][i];
            }
        }
    }
    return Result;
}


template <int _size_m>
double TMatrix<_size_m>::GetMinDiagPositive()
{
    double Result = 0;
    if (0 < s_m && s_m <= _size_m
            && 0 < s_n && s_n <= _size_m)
    {
        int Dim = std::min(s_m, s_n);
        for (int i=0; i<Dim; i++)
        {
            if (Result < EPSILON_M
                    || (M[i][i] < Result && M[i][i] > EPSILON_M))
            {
                Result = M[i][i];
            }
        }
    }
    return Result;
}


template <int _size_m>
double TMatrix<_size_m>::GetMaxDiag3()
{
    double Result = - 1000000000;
    if (0 < s_m && s_m <= _size_m
            && 0 < s_n && s_n <= _size_m)
    {
        int Dim = std::min(s_m, s_n);
        if (Dim >= 3)
        {
            for (int i=0; i<3; i++)
            {
                if (M[i][i] > Result)
                {
                    Result = M[i][i];
                }
            }
        }
    }
    return Result;
}


template <int _size_m>
double TMatrix<_size_m>::GetTrace()
{
    double Trace = 0;
    int N = std::min(_size_m, std::min(s_n, s_m));
    for (int i=0; i<N; i++)
    {
        Trace += M[i][i];
    }
    return Trace;
}


template <int _size_m>
double TMatrix<_size_m>::GetTraceVel()
{
    double Trace = 0;
    if (_size_m >= 6 && s_n >= 6 && s_m >= 6)
    {
        Trace = M[3][3] + M[4][4] + M[5][5];
    }
    return Trace;
}


template <int _size_m>
void TMatrix<_size_m>::FCovMatrAdaptation()
{
    if (s_m == s_n && s_m > 0 && s_m <= _size_m)
    {
        int Dim = s_m;
        int i,j;
        double Kmax;
        for (i=0; i<Dim-1; i++)
        {
            if (M[i][i] < 0)
            {
                M[i][i] = fabs(M[i][i]);
            }
            for (j=i+1; j<Dim; j++)
            {
                Kmax = sqrt(fabs(M[i][i] * M[j][j])); //maximum value of covariation
                if (fabs(M[i][j] - M[j][i]) > ZERO_TOLERANCE_M)
                {
                    if (M[i][j] > M[j][i])
                    {
                        if (M[j][i] > Kmax)
                        {
                            M[j][i] = Kmax;
                        }
                        M[i][j] = M[j][i];
                    }
                    else
                    {
                        if (M[i][j] > Kmax)
                        {
                            M[i][j] = Kmax;
                        }
                        M[j][i] = M[i][j];
                    }
                }
                else
                {
                    if (M[j][i] > Kmax)
                    {
                        M[j][i] = Kmax;
                        M[i][j] = Kmax;
                    }
                }
            }
        }
    }
}


template <int _size_m>
bool TMatrix<_size_m>::IsSymmetricPositiveDefinite()
{
    bool bRes = true;

    if (s_m > 0 && s_m <= _size_m && s_m == s_n)
    {
        int i, j, k;
        TMatrix<_size_m> CopyM(s_m, s_n);
        CopyM = *this;

        //transforming the matrix to the upper triangular form
        for (i = 0; i < s_m-1; i++)
        {
            if (CopyM.M[i][i] >= EPSILON_M) //check the dyagonal elements
            {
                for (j = i+1; j < s_m; j++)
                {
                    if (fabs(M[j][i] - M[i][j]) < EPSILON_M) //check the symmetry
                    {
                        if (fabs(CopyM.M[j][i]) > EPSILON_M)
                        {       //linear combination "row(j) = row(j) + Lambda * row(i)", to set element [j][i] to zero
                            double Lambda = -CopyM.M[j][i]/CopyM.M[i][i];
                            for (k = i; k < s_m; k++)
                            {
                                CopyM.M[j][k] += Lambda * CopyM.M[i][k];
                            }
                        }
                    }
                    else
                    {
                        bRes = false;
                        break;
                    }
                }
            }
            else
            {
                bRes = false;
                break;
            }
        }

        if (bRes)
        {
            if (CopyM.M[s_m-1][s_m-1] < EPSILON_M) //check the last dyagonal element
            {
                bRes = false;
            }
        }
    }
    else
    {
        bRes = false;
    }

    return bRes;
}


template <int _size_m>
void TMatrix<_size_m>::RestrictionRMSE(double MaxRMSE)
{
    if (MaxRMSE > 0)
    {
        double SqrMaxRMSE = sqr(MaxRMSE);
        double maxDiagEl = this->GetMaxDiag(); //maximum diagonal element
        if (maxDiagEl > SqrMaxRMSE)
        {
            double mult = SqrMaxRMSE / maxDiagEl;
            (*this) = (*this) * mult;
        }
    }
}


template <int _size_m>
void TMatrix<_size_m>::FormIdentityMatrix(int _n)
{
    this->Reset();
    if (0 < _n && _n <= _size_m)
    {
        this->SetDimensions(_n, _n);
        for (int i=0; i<_n; i++)
        {
            this->M[i][i] = 1.;
        }
    }
}


template <int _size_m>
void TMatrix<_size_m>::FormRotationMatrix(double Theta, double X, double Y, double Z)
{
    this->Reset();
    if (_size_m >= 3)
    {
        this->FormIdentityMatrix(3);

        double modVect = sqrt(sqr(X) + sqr(Y) + sqr(Z));
        if (modVect > EPSILON_M)
        {
            double cos_Theta = cos(Theta);

            if (cos_Theta < 1. - EPSILON_M)
            {
                double sin_Theta = sin(Theta);
                double OneMinusCosTheta = 1. - cos_Theta;

                //vector normalization
                double Xn = X / modVect;
                double Yn = Y / modVect;
                double Zn = Z / modVect;

                double OneMinusCosTheta_XnYn = OneMinusCosTheta * Xn * Yn;
                double OneMinusCosTheta_XnZn = OneMinusCosTheta * Xn * Zn;
                double OneMinusCosTheta_YnZn = OneMinusCosTheta * Yn * Zn;

                double sin_Theta_Xn = sin_Theta * Xn;
                double sin_Theta_Yn = sin_Theta * Yn;
                double sin_Theta_Zn = sin_Theta * Zn;

                this->M[0][0] = cos_Theta + OneMinusCosTheta * sqr(Xn);
                this->M[0][1] = OneMinusCosTheta_XnYn - sin_Theta_Zn;
                this->M[0][2] = OneMinusCosTheta_XnZn + sin_Theta_Yn;

                this->M[1][0] = OneMinusCosTheta_XnYn + sin_Theta_Zn;
                this->M[1][1] = cos_Theta + OneMinusCosTheta * sqr(Yn);
                this->M[1][2] = OneMinusCosTheta_YnZn - sin_Theta_Xn;

                this->M[2][0] = OneMinusCosTheta_XnZn - sin_Theta_Yn;
                this->M[2][1] = OneMinusCosTheta_YnZn + sin_Theta_Xn;
                this->M[2][2] = cos_Theta + OneMinusCosTheta * sqr(Zn);
            }
        }
    }
}


template <int _size_m>
bool TCMatrixFunc<_size_m>::MatrXMatr(const TMatrix<_size_m> &A, const TMatrix<_size_m> &B, TMatrix<_size_m> &Res)
{
    bool bOK = true;
    if (A.s_m > 0 && A.s_n > 0
            && B.s_m > 0 && B.s_n > 0
            && A.s_m <= _size_m && A.s_n <= _size_m
            && B.s_m <= _size_m && B.s_n <= _size_m
            && A.s_n == B.s_m)
    {
        Res.Reset(A.s_m, B.s_n);
        qint32 i=0;
        qint32 j=0;
        qint32 k=0;
        const qint32 arows = A.s_m;
        const qint32 acols = A.s_n;
        const qint32 bcols = B.s_n;

        const double *a_data = &A.M[0][0];
        const double *b_data = &B.M[0][0];
        double *d_data = &Res.M[0][0];

        const qint32 a_step = _size_m;
        const qint32 b_step = _size_m;
        const qint32 d_step = _size_m;

        for( i = 0; i < arows; i++, a_data += a_step, d_data += d_step )
        {
            for( j = 0; j <= bcols - 4; j += 4 )
            {
                const double* b = b_data + j;
                double s0(0), s1(0), s2(0), s3(0);

                for( k = 0; k < acols; k++, b += b_step )
                {
                    const double a(a_data[k]);
                    s0 += a * b[0];
                    s1 += a * b[1];
                    s2 += a * b[2];
                    s3 += a * b[3];
                }

                d_data[j] = s0;
                d_data[j+1] = s1;
                d_data[j+2] = s2;
                d_data[j+3] = s3;
            }

            for( ; j < bcols; j++ )
            {
                const double* b = b_data + j;
                double s0(0);

                for( k = 0; k < acols; k++, b += b_step )
                {
                    s0 += a_data[k] * b[0];
                }

                d_data[j] = s0;
            }
        }
    }
    else
    {
        bOK = false;
    }
    return bOK;
}


template <int _size_m>
bool TCMatrixFunc<_size_m>::MatrXMatrT(const TMatrix<_size_m> &A, const TMatrix<_size_m> &BT, TMatrix<_size_m> &Res)
{
    bool bOK = true;
    if (A.s_m > 0 && A.s_n > 0
            && BT.s_m > 0 && BT.s_n > 0
            && A.s_m <= _size_m && A.s_n <= _size_m
            && BT.s_m <= _size_m && BT.s_n <= _size_m
            && A.s_n == BT.s_n)
    {
        Res.Reset(A.s_m, BT.s_m);
        qint32 i=0;
        qint32 j=0;
        qint32 k=0;
        const qint32 arows = A.s_m;
        const qint32 acols = A.s_n;
        const qint32 bTrows = BT.s_m;

        const double *a_data = &A.M[0][0];
        const double *b_data = &BT.M[0][0];
        double *d_data = &Res.M[0][0];

        const qint32 a_step = _size_m;
        const qint32 b_step = _size_m;
        const qint32 d_step = _size_m;

        for( i = 0; i < arows; i++, a_data += a_step, d_data += d_step )
        {
            for( j = 0; j <= bTrows - 4; j += 4 )
            {
                const double* b = b_data + j*b_step;
                double s0(0), s1(0), s2(0), s3(0);

                for( k = 0; k < acols; k++, b++ )
                {
                    const double a(a_data[k]);
                    s0 += a * b[0];
                    s1 += a * b[b_step];
                    s2 += a * b[2*b_step];
                    s3 += a * b[3*b_step];
                }

                d_data[j] = s0;
                d_data[j+1] = s1;
                d_data[j+2] = s2;
                d_data[j+3] = s3;
            }

            for( ; j < bTrows; j++ )
            {
                const double* b = b_data + j*b_step;
                double s0(0);

                for( k = 0; k < acols; k++, b++ )
                {
                    s0 += a_data[k] * b[0];
                }

                d_data[j] = s0;
            }
        }
    }
    else
    {
        bOK = false;
    }
    return bOK;
}


template <int _size_m>
bool TCMatrixFunc<_size_m>::MatrXMatrLowerTriangular(const TMatrix<_size_m> &M1, const TMatrix<_size_m> &M2, TMatrix<_size_m> &A)
{
    bool bOK = true; //true if result is ok
    if (M1.s_m == M1.s_n && M2.s_m == M2.s_n && M1.s_m == M2.s_m
            && M1.s_m > 0 && M1.s_m <= _size_m)
    {
        int Dim = M1.s_m;
        A.Reset(Dim, Dim);
        int i, j, k;

        for (i=0; i<Dim; i++)
        {
            for (j=0; j<=i; j++)
            {
                for (k=j; k<=i; k++)
                {
                    A.M[i][j] += M1.M[i][k] * M2.M[k][j];
                }
            }
        }
    }
    else
    {
        bOK = false;
    }
    return bOK;
}


template <int _size_m>
bool TCMatrixFunc<_size_m>::MatrXMatrUpperTriangToLowerTriang(const TMatrix<_size_m> &M1, const TMatrix<_size_m> &M2, TMatrix<_size_m> &A)
{
    bool bOK = true; //true if result is ok
    if (M1.s_m == M1.s_n && M2.s_m == M2.s_n && M1.s_m == M2.s_m
            && M1.s_m > 0 && M1.s_m <= _size_m)
    {
        int Dim = M1.s_m;
        A.Reset(Dim, Dim);
        int i, j, k, k0;
        for (i=0; i<Dim; i++)
        {
            k0 = i;
            for (j=0; j<Dim; j++)
            {
                if (j>k0)
                {
                    k0 = j;
                }

                for (k=k0; k<Dim; k++)
                {
                    A.M[i][j] += M1.M[i][k]*M2.M[k][j];
                }
            }
        }
    }
    else
    {
        bOK = false;
    }
    return bOK;
}


template <int _size_m>
bool TCMatrixFunc<_size_m>::MatrLTL_UpperTriangToLowerTriang(const TMatrix<_size_m> &L, TMatrix<_size_m> &Res)
{
    bool bOK = true; //true if result is ok
    if (L.s_m == L.s_n
            && L.s_m > 0 && L.s_m <= _size_m)
    {
        int Dim = L.s_m;
        int i,j,k;
        Res.Reset(Dim, Dim);
        int Step = _size_m;
        double *ResDi = &Res.M[0][0];
        double *Res_ij = ResDi;
        double *Res_ji = ResDi;
        const double *LDi = &L.M[0][0];
        const double *Lk0i = LDi;
        const double *Lki = Lk0i;
        const double *Lk0j = LDi;
        const double *Lkj = Lk0j;
        for (i=0; i<Dim; i++, LDi+=Step+1, ResDi+=Step+1)
        {
            Res_ij = ResDi;
            Res_ji = ResDi;
            Lk0i = LDi;
            Lk0j = LDi;
            for (j=i; j<Dim; j++, Res_ij++, Res_ji+=Step, Lk0i+=Step, Lk0j+=Step+1)
            {
                Lki = Lk0i;
                Lkj = Lk0j;
                for (k=j; k<Dim; k++, Lki+=Step, Lkj+=Step)
                {
                    *Res_ij += (*Lki) * (*Lkj); //L.M[k][i] * L.M[k][j];
                }
                if (j>i)
                {
                    *Res_ji = *Res_ij;
                }
            }
        }
    }
    else
    {
        bOK = false;
    }
    return bOK;
}


template <int _size_m>
bool TCMatrixFunc<_size_m>::MatrXMatrXMatr(const TMatrix<_size_m> &M1, const TMatrix<_size_m> &M2, const TMatrix<_size_m> &M3, TMatrix<_size_m> &A_out)
{
    TMatrix<_size_m> Mcurr;

    bool ItsOk;

    ItsOk = MatrXMatr(M1, M2, Mcurr); //Mcurr = M1 * M2
    if (!ItsOk)
        return false;

    ItsOk = MatrXMatr(Mcurr, M3, A_out); //A_out = M1 * M2 * M3
    return ItsOk;
}


template <int _size_m>
bool TCMatrixFunc<_size_m>::MatrMultAxBxAT(const TMatrix<_size_m> &A, const TMatrix<_size_m> &B, TMatrix<_size_m> &Res)
{
    bool bOK = true;
    TMatrix<_size_m> Mtmp;
    bOK &= MatrXMatr(A, B, Mtmp);
    bOK &= MatrXMatrT(Mtmp, A, Res);
    return bOK;
}


template <int _size_m>
bool TCMatrixFunc<_size_m>::MatrXVec(TMatrix<_size_m> &M, TVector<_size_m> &V, TMatrix<_size_m> &A_out)
{
    TMatrix<_size_m> Vmatr;
    bool Result;

    Result = Matching(V, Vmatr);
    if (!Result)
        return false;

    Result = MatrXMatr(M, Vmatr, A_out);
    if (!Result)
        return false;

    return true;
}


template <int _size_m>
bool TCMatrixFunc<_size_m>::MatrXVec( TMatrix<_size_m> &M, TVector<_size_m> &V, TVector<_size_m> &A_out)
{
    bool bOK = true;
    if (M.s_m > 0 && M.s_m <= _size_m
            && M.s_n > 0 && M.s_n <= _size_m
            && V.s == M.s_n)
    {
        A_out.Reset(M.s_m);
        int Step = _size_m;
        int i=0;
        int j=0;
        double *Mi = &M.M[0][0];
        double *Mij = Mi;
        double *ResVi = &A_out.Vec[0];
        for (i=0; i<M.s_m; i++, Mi+=Step, ResVi++)
        {
            double *Vj = &V.Vec[0];
            Mij = Mi;
            double s0(0);
            for (j=0; j<M.s_n; j++, Mij++, Vj++)
            {
                s0 += Mij[0] * Vj[0];
            }
            ResVi[0] = s0;
        }
    }
    else
    {
        bOK = false;
    }
    return bOK;
}


template <int _size_m>
bool TCMatrixFunc<_size_m>::VecXMatr(TVector<_size_m> &V, TMatrix<_size_m> &M, TMatrix<_size_m> &A_out)
{
    bool Result;
    TMatrix<_size_m> Vmatr;

    Result = Matching(V, Vmatr);
    if (!Result)
        return false;

    Result = MatrXMatr(Vmatr, M, A_out);
    if (!Result)
        return false;

    A_out.Reset();

    return true;
}


template <int _size_m>
bool TCMatrixFunc<_size_m>::VecXMatr(TVector<_size_m> &V, TMatrix<_size_m> &M, TVector<_size_m> &A_out)
{
//    bool Result;
//    TMatrix<_size_m> Amatr;

//    Result = VecXMatr(V, M, Amatr);
//    if (!Result)
//        return false;

//    Result = Matching(Amatr, A_out);
//    if (!Result)
//        return false;

//    return true;

    if(V.s != M.s_m)
        return false;

    A_out.s = V.s;
    int i = 0, j = 0;
    while(i < M.s_n)
    {
        A_out.Vec[i] = 0;
        while(j < V.s)
        {
            A_out.Vec[i]+=V.Vec[j]*M.M[i][j];
            ++j;
        }
        j=0;
        ++i;
    }
    return true;
}


template <int _size_m>
bool TCMatrixFunc<_size_m>::VecXMatrXVec(TVector<_size_m> &V1, TMatrix<_size_m> &M,TVector<_size_m> &V2, double &out)
{
    if(V1.s != M.s_m || V2.s != M.s_n)
        return false;

    out = 0;
    double temp;
    int i = 0, j = 0;
    while(i < V2.s)
    {
        temp = 0;
        while(j < V1.s)
        {
            temp += V1.Vec[j]*M.M[i][j];
            ++j;
        }
        out += temp*V2.Vec[i];
        j=0;
        ++i;
    }
    return true;
}


template <int _size_m>
double TCMatrixFunc<_size_m>::CalcMahalanobisDist(TVector<_size_m> &V, TMatrix<_size_m> &Inv_M)
{
    double dM = -1;
    if (Inv_M.s_m > 0 && Inv_M.s_m <= _size_m
            && Inv_M.s_m == Inv_M.s_n && V.s == Inv_M.s_m)
    {
        double Sum = 0;
        qint32 i=0;
        qint32 j=0;
        qint32 a_step = _size_m;
        const double *a_data = &Inv_M.M[0][0];
        const double *v_data = &V[0];
        for (i=0; i<V.s; i++, a_data+=a_step+1, v_data++)
        {
            double V_i = v_data[0];
            Sum += a_data[0] * sqr(V_i);
            if (i < V.s-1)
            {
                const double *a_data_curr = a_data+1;
                const double *v_data_curr = v_data+1;
                for (j=i+1; j<V.s; j++, a_data_curr++, v_data_curr++)
                {
                    Sum += 2. * a_data_curr[0] * V_i * v_data_curr[0];
                }
            }
        }

        if (Sum > ZERO_TOLERANCE_M)
        {
            dM = sqrt(Sum);
        }
        else
        {
            if (Sum > -ZERO_TOLERANCE_M)
            {
                dM = 0;
            }
        }
    }
    return dM;
}


template <int _size_m>
bool TCMatrixFunc<_size_m>::MatrXMatr(Matrix3x3 &M1, Matrix3x3 &M2, Matrix3x3 &A)
{

    A[0][0] = M1[0][0] * M2[0][0] + M1[0][1] * M2[1][0] + M1[0][2] * M2[2][0];
    A[0][1] = M1[0][0] * M2[0][1] + M1[0][1] * M2[1][1] + M1[0][2] * M2[2][1];
    A[0][2] = M1[0][0] * M2[0][2] + M1[0][1] * M2[1][2] + M1[0][2] * M2[2][2];

    A[1][0] = M1[1][0] * M2[0][0] + M1[1][1] * M2[1][0] + M1[1][2] * M2[2][0];
    A[1][1] = M1[1][0] * M2[0][1] + M1[1][1] * M2[1][1] + M1[1][2] * M2[2][1];
    A[1][2] = M1[1][0] * M2[0][2] + M1[1][1] * M2[1][2] + M1[1][2] * M2[2][2];

    A[2][0] = M1[2][0] * M2[0][0] + M1[2][1] * M2[1][0] + M1[2][2] * M2[2][0];
    A[2][1] = M1[2][0] * M2[0][1] + M1[2][1] * M2[1][1] + M1[2][2] * M2[2][1];
    A[2][2] = M1[2][0] * M2[0][2] + M1[2][1] * M2[1][2] + M1[2][2] * M2[2][2];

    return true;
}

template <int _size_m>
bool TCMatrixFunc<_size_m>::Matr3XVect3(Matrix3x3 &M, const Vector3 &V, Vector3 &Vec)
{

    Vec[0] = M[0][0] * V[0] + M[0][1] * V[1] + M[0][2] * V[2];
    Vec[1] = M[1][0] * V[0] + M[1][1] * V[1] + M[1][2] * V[2];
    Vec[2] = M[2][0] * V[0] + M[2][1] * V[1] + M[2][2] * V[2];

    return true;
}

template <int _size_m>
double TCMatrixFunc<_size_m>::det_rec(const TMatrix<_size_m> &A, TVector<_size_m> &V, int n)
{
    //The recurrence procedure of calculation of the determinant
    int i,j=0;
    double s=0;
    int d = A.s_m-1;
    if (n>0)
    {
        for (i = 0; i <= d; i++)
        {
            if (V.Vec[i] != 1)
            {
                V.Vec[i] = 1;
                s = s + A.M[i][d-n] * (1-2 * ((i-j) % 2)) * det_rec(A, V, n-1);
                V.Vec[i] = 0;
            }
            else ++j;
        }
    }
    else
    {
        for (i = 0; i <= d; i++)
        {
            if (V.Vec[i] == 0)
            {
                s = A.M[i][d];
            }
        }
    }
    return s;
}


template <int _size_m>
bool TCMatrixFunc<_size_m>::det_Gauss(TMatrix<_size_m> &M, double &det)
{
    det = 0;
    bool bOK = true;
    if (M.s_m > 0
            && M.s_m <=_size_m
            && M.s_m == M.s_n)
    {
        int i=0;
        int j=0;
        int k=0;
        int N = M.s_m;
        TMatrix<_size_m> Mcurr = M;

        double *M_ii = &Mcurr.M[0][0];
        qint32 Step = _size_m;
        bool bZero = false;

        for (i=0; i<N; i++, M_ii+=Step+1) //transformation of the matrix to triangular form
        {
            if (fabs(M_ii[0]) < ZERO_TOLERANCE_M) //if diagonal element [i][i] is very close to zero
            {
                bool flag_all0 = true; //sign "all elements [i][i],[i+1][i],... in the column [i] are zeroes"
                const double *M_ji = M_ii+Step;
                for (j=i+1; j<N; j++, M_ji+=Step)
                {
                    if (fabs(M_ji[0]) > ZERO_TOLERANCE_M ) //search of non-zero element [j][i] in the column [i]
                    {
                        const double *M_jk = M_ji;
                        double *M_ik = M_ii;
                        for (k=i; k<N; k++, M_ik++, M_jk++)
                        {
                            M_ik[0] += M_jk[0]; //addition of row [j] with non-zero element [j][i] to the row [i]
                        }
                        flag_all0 = false;
                        break;
                    }
                }
                if (flag_all0) //if all elements in the column [i] under row [i] (including row [i]) are zeroes
                {
                    det = 0;
                    bZero = true;
                }
            }

            if (!bZero)
            {
                //now element [i][i] is not zero
                const double M_i_i = M_ii[0];
                /*const*/ double *M_ji = M_ii+Step; //&Mcurr.M[i+1][i];
                for (j=i+1; j<N; j++, M_ji+=Step)
                {
                    const double M_j_i = M_ji[0]; //Mcurr.M[j][i];
                    if (fabs(M_j_i) > ZERO_TOLERANCE_M) //non-zero element [j][i] in the column [i]
                    {
                        const double Multiplicator = M_j_i/M_i_i;
                        double *M_jk = M_ji;
                        const double *M_ik = M_ii;
                        for (k=i; k<N; k++, M_jk++, M_ik++) //subtraction from row [j] of row [i] multiplied par Mcurr.M[j][i]/Mcurr.M[i][i]
                        {
                            M_jk[0] -= M_ik[0]*Multiplicator; //now elements [j][i] are 0, j>i
                        }
                    }
                }
            }
        }

        if (!bZero)
        {
            M_ii = &Mcurr.M[0][0];
            det = 1;
            for (i=0; i<N; i++, M_ii+=Step+1)
            {
                det = det * M_ii[0]; //determinant is the product of diagonal elements of triangular matrix
            }
        }
    }
    else
    {
        bOK = false;
    }

    return bOK;
}


template <int _size_m>
bool TCMatrixFunc<_size_m>::Det(TMatrix<_size_m> &M, double &det)
{
    if(M.s_m != M.s_n)
        return false;
    //TVector<_size_m> dop;

    //dop.s = M.s_m;
    //det = det_rec(M, dop, dop.s-1);
    bool ItsOK = det_Gauss(M, det);
    if (!ItsOK)
        return false;

    return true;
}


template <int _size_m>
double TCMatrixFunc<_size_m>::Det(TMatrix<_size_m> &M)
{
    double det = 0;

    if(M.s_m != M.s_n)
        return det;

    //TVector<_size_m> dop;

    //dop.s = M.s_m;
    //det = det_rec(M, dop, dop.s-1);
    det_Gauss(M, det);

    return det;
}


template <int _size_m>
void TCMatrixFunc<_size_m>::Det(Matrix3x3 &M, double &d3)
{
    /*d3 = (M[0][0]*M[1][1]*M[2][2] + M[0][1]*M[1][2]*M[2][1] + M[1][0]*M[2][1]*M[0][2]) -
        (M[2][0]*M[1][1]*M[0][2] - M[2][1]*M[1][2]*M[0][0] - M[1][0]*M[0][1]*M[2][2]);*/
    d3 = M[0][0]*M[1][1]*M[2][2] - M[0][0]*M[1][2]*M[2][1] - M[0][1]*M[1][0]*M[2][2] +
        M[0][1]*M[2][0]*M[1][2] + M[1][0]*M[0][2]*M[2][1] - M[0][2]*M[1][1]*M[2][0];

    return;
}

template <int _size_m>
double TCMatrixFunc<_size_m>::Det(Matrix3x3 &M)
{

    double det = ((M[0][0]*M[1][1]*M[2][2] + M[0][1]*M[1][2]*M[2][1] + M[1][0]*M[2][1]*M[0][2]) -
        (M[2][0]*M[1][1]*M[0][2] - M[2][1]*M[1][2]*M[0][0] - M[1][0]*M[0][1]*M[2][2]));

    return det;
}

template <int _size_m>
void TCMatrixFunc<_size_m>::main_el(TMatrix<_size_m> &A, TMatrix<_size_m> &E, int k)
{
    //choice of a main unit
    int i,j,i1,j1,tmp;
    double w,max;
    int size = A.s_m;
    max = fabs(A.M[k][k]);
    i1 = k;
    j1 = k;
    for(i = k;i<size;i++)
        for(j = k;j<size;j++)
        {
            w = fabs(A.M[i][j]);
            if(w > max)
            {
                max = w;
                i1 = i;
                j1 = j;
            }
        }

    for(i=0;i<size;i++)
    {
        w = A.M[i][k];
        A.M[i][k] = A.M[i][j1];
        A.M[i][j1] = w;
    }
    tmp = rearr[k];
    rearr[k] = rearr[j1];
    rearr[j1] = tmp;
    for(j=k;j<size;j++)
    {
        w = A.M[k][j];
        A.M[k][j] = A.M[i1][j];
        A.M[i1][j] = w;
    }
    for(j=0;j<size;j++)
    {
        w = E.M[k][j];
        E.M[k][j] = E.M[i1][j];
        E.M[i1][j] = w;
    }
}

template <int _size_m>
void TCMatrixFunc<_size_m>::Inv(TMatrix<_size_m> &A, TMatrix<_size_m> &E, TMatrix<_size_m> &R)
{
    //Retrace of Gauss' method
    int i,j,k,l;
    int size = A.s_m;
    R.s_m = A.s_m;
    R.s_n = A.s_n;
    double z=0;
    TVector<_size_m> x;
    int n = size-1;
    for(k=0;k<size;k++)
    {
        x.Vec[n] = E.M[n][k];
        for(i=n-1;i>=0;i--)
        {
            for(j=n;j>=i+1;j--)
                z += A.M[i][j]*x.Vec[j];
            x.Vec[i] = E.M[i][k] - z;
            z = 0;
        }
        for(i=0;i<size;i++)
        {
            l = rearr[i];
            R.M[l][k] = x.Vec[i];
        }
    }
}


template <int _size_m>
bool TCMatrixFunc<_size_m>::Inv_Gauss(TMatrix<_size_m> &M, TMatrix<_size_m> &InvM, bool &bSmallDet)
{
    bool bOK = true;
    bSmallDet = false;
    if (M.s_m > 0
            && M.s_m <=_size_m
            && M.s_m == M.s_n)
    {
        qint32 i=0;
        qint32 j=0;
        qint32 k=0;
        qint32 N = M.s_m;
        qint32 Step = _size_m;
        TMatrix<_size_m> Mcurr = M;
        InvM.Reset(M.s_m, M.s_m);
        double *InvM_i = &InvM.M[0][0];

        for (i=0; i<N; i++, InvM_i+=Step+1)
        {
            InvM_i[0] = 1.;
        }

        double *M_ii = &Mcurr.M[0][0];
        InvM_i = &InvM.M[0][0];
        qint32 ColMax = 0; //current maximum changeable column in the resulting matrix

        for (i=0; i<N; i++, M_ii+=Step+1, InvM_i+=Step) //transformation of the matrix to triangular form
        {
            if (ColMax<i)
            {
                ColMax = i;
            }
            if (fabs(M_ii[0]) < ZERO_TOLERANCE_M) //if diagonal element [i][i] is very close to zero
            {
                bool flag_all0 = true; //sign "all elements [i][i],[i+1][i],... in the column [i] are zeroes"
                const double *M_ji = M_ii+Step;
                const double *InvM_j = InvM_i+Step;
                for (j=i+1; j<N; j++, M_ji+=Step, InvM_j+=Step)
                {
                    if (fabs(M_ji[0]) > ZERO_TOLERANCE_M ) //search of non-zero element [j][i] in the column [i]
                    {
                        if (ColMax<j)
                        {
                            ColMax = j;
                        }
                        const double *M_jk = M_ji;
                        double *M_ik = M_ii;
                        for (k=i; k<N; k++, M_ik++, M_jk++)
                        {
                            M_ik[0] += M_jk[0]; //addition of row [j] with non-zero element [j][i] to the row [i]
                        }

                        const double *InvM_jk = InvM_j;
                        double *InvM_ik = InvM_i;
                        for (k=0; k<=ColMax; k++, InvM_ik++, InvM_jk++)
                        {
                            InvM_ik[0] += InvM_jk[0];//same actions for the future inverse matrix
                        }

                        flag_all0 = false;
                        break;
                    }
                }

                if (flag_all0) //if all elements in the column [i] under row [i] (including row [i]) are zeroes
                {
                    bOK = false; //Determinant = 0
                    bSmallDet = true;
                    break;
                }
            }

            //now element [i][i] is not zero
            const double M_i_i = M_ii[0];
            double *M_ij = M_ii;
            double *InvM_ij = InvM_i;
            for (j=i; j<N; j++, M_ij++)
            {
                *M_ij /= M_i_i;
            }
            for (j=0; j<=ColMax; j++, InvM_ij++)
            {
                *InvM_ij /= M_i_i;
            }

            double *M_ji = &Mcurr.M[0][0]+i; //&Mcurr.M[0][i];
            double *InvM_j = &InvM.M[0][0];
            for (j=0; j<N; j++, M_ji+=Step, InvM_j+=Step)
            {
                if (j != i)
                {
                    const double M_j_i = M_ji[0]; //Mcurr.M[j][i];
                    if (fabs(M_j_i) > ZERO_TOLERANCE_M) //non-zero element [j][i] in the column [i]
                    {
                        const double Multiplicator = M_j_i;
                        double *M_jk = M_ji;
                        const double *M_ik = M_ii;
                        for (k=i; k<N; k++, M_jk++, M_ik++) //subtraction from row [j] of row [i] multiplied par Mcurr.M[j][i]/Mcurr.M[i][i]
                        {
                            M_jk[0] -= M_ik[0]*Multiplicator; //now elements [j][i] are 0, j>i
                        }

                        double *InvM_jk = InvM_j;
                        const double *InvM_ik = InvM_i;
                        for (k=0; k<=ColMax; k++, InvM_jk++, InvM_ik++)
                        {
                            InvM_jk[0] -= InvM_ik[0]*Multiplicator; //same actions for the future inverse matrix
                        }
                    }
                }
            }
        }
    }
    else
    {
        bOK = false;
    }
    return bOK;
}


template <int _size_m>
bool TCMatrixFunc<_size_m>::Inv_Gauss(TMatrix<_size_m> &M, TMatrix<_size_m> &InvM, bool &bSmallDet, double &DetM)
{
    bool bOK = true;
    bSmallDet = false;
    DetM = 1.;
    if (M.s_m > 0
            && M.s_m <=_size_m
            && M.s_m == M.s_n)
    {
        qint32 i=0;
        qint32 j=0;
        qint32 k=0;
        qint32 N = M.s_m;
        qint32 Step = _size_m;
        TMatrix<_size_m> Mcurr = M;
        InvM.Reset(M.s_m, M.s_m);
        double *InvM_i = &InvM.M[0][0];

        for (i=0; i<N; i++, InvM_i+=Step+1)
        {
            InvM_i[0] = 1.;
        }

        double *M_ii = &Mcurr.M[0][0];
        InvM_i = &InvM.M[0][0];
        qint32 ColMax = 0; //current maximum changeable column in the resulting matrix

        for (i=0; i<N; i++, M_ii+=Step+1, InvM_i+=Step) //transformation of the matrix to triangular form
        {
            if (ColMax<i)
            {
                ColMax = i;
            }
            if (fabs(M_ii[0]) < ZERO_TOLERANCE_M) //if diagonal element [i][i] is very close to zero
            {
                bool flag_all0 = true; //sign "all elements [i][i],[i+1][i],... in the column [i] are zeroes"
                const double *M_ji = M_ii+Step;
                const double *InvM_j = InvM_i+Step;
                for (j=i+1; j<N; j++, M_ji+=Step, InvM_j+=Step)
                {
                    if (fabs(M_ji[0]) > ZERO_TOLERANCE_M ) //search of non-zero element [j][i] in the column [i]
                    {
                        if (ColMax<j)
                        {
                            ColMax = j;
                        }
                        const double *M_jk = M_ji;
                        double *M_ik = M_ii;
                        for (k=i; k<N; k++, M_ik++, M_jk++)
                        {
                            M_ik[0] += M_jk[0]; //addition of row [j] with non-zero element [j][i] to the row [i]
                        }

                        const double *InvM_jk = InvM_j;
                        double *InvM_ik = InvM_i;
                        for (k=0; k<=ColMax; k++, InvM_ik++, InvM_jk++)
                        {
                            InvM_ik[0] += InvM_jk[0];//same actions for the future inverse matrix
                        }

                        flag_all0 = false;
                        break;
                    }
                }

                if (flag_all0) //if all elements in the column [i] under row [i] (including row [i]) are zeroes
                {
                    bOK = false; //Determinant = 0
                    bSmallDet = true;
                    break;
                }
            }

            //now element [i][i] is not zero
            const double M_i_i = M_ii[0];
            double *M_ij = M_ii;
            double *InvM_ij = InvM_i;
            for (j=i; j<N; j++, M_ij++)
            {
                *M_ij /= M_i_i;
            }
            for (j=0; j<=ColMax; j++, InvM_ij++)
            {
                *InvM_ij /= M_i_i;
            }
            DetM *= M_i_i;

            double *M_ji = &Mcurr.M[0][0]+i; //&Mcurr.M[0][i];
            double *InvM_j = &InvM.M[0][0];
            for (j=0; j<N; j++, M_ji+=Step, InvM_j+=Step)
            {
                if (j != i)
                {
                    const double M_j_i = M_ji[0]; //Mcurr.M[j][i];
                    if (fabs(M_j_i) > ZERO_TOLERANCE_M) //non-zero element [j][i] in the column [i]
                    {
                        const double Multiplicator = M_j_i;
                        double *M_jk = M_ji;
                        const double *M_ik = M_ii;
                        for (k=i; k<N; k++, M_jk++, M_ik++) //subtraction from row [j] of row [i] multiplied par Mcurr.M[j][i]/Mcurr.M[i][i]
                        {
                            M_jk[0] -= M_ik[0]*Multiplicator; //now elements [j][i] are 0, j>i
                        }

                        double *InvM_jk = InvM_j;
                        const double *InvM_ik = InvM_i;
                        for (k=0; k<=ColMax; k++, InvM_jk++, InvM_ik++)
                        {
                            InvM_jk[0] -= InvM_ik[0]*Multiplicator; //same actions for the future inverse matrix
                        }
                    }
                }
            }
        }
    }
    else
    {
        bOK = false;
    }
    return bOK;
}


//template <int _size_m>
//bool TCMatrixFunc<_size_m>::InvMatrix(TMatrix<_size_m> &M, TMatrix<_size_m> &I)
//{
//    //Gauss' method with a choice of a main unit on all matrix
//    int i,j,k;
//    double first, det=0, multiplicator = 0.;

//    bool ItsOK = true, b_mult = false;
//    ItsOK = Det(M,det);
//    if(!ItsOK)
//    {
//        return false;
//    }

//    if (fabs(det) < ZERO_TOLERANCE_INV)
//    {
//        double max_el = M.GetMax();
//        if (max_el > ZERO_TOLERANCE_M)
//        {
//            b_mult = true;
//            if (fabs(max_el) > 1.)
//            {
//                multiplicator = max_el;
//            }
//            else
//            {
//                multiplicator = 1./max_el;
//            }
//        }
//        else
//        {
//            return false;
//        }
//    }


//    if(M.s_m != M.s_n)
//    {
//        return false;
//    }

//    int size = M.s_m;

//    m_tmp1.Reset();
//    m_tmp2.Reset();

//    if (!b_mult)
//    {
//        memcpy(m_tmp1.M, M.M, sizeof(M.M));//!!!!
//    }
//    else
//    {
//        m_tmp1 = M * multiplicator;
//        ItsOK = Det(m_tmp1, det);
//        if (!ItsOK || fabs(det) < ZERO_TOLERANCE_INV)
//        {
//            return false;
//        }
//    }

//    //memset(I.M, 0, sizeof(I.M));//!!!!
//    I.Reset();

//    m_tmp1.s_m = m_tmp1.s_n = size;


//    for(i = 0; i < size;i++)
//    {
//        rearr[i] = i;
//    }

//    //	memset(m_tmp2.M, 0, sizeof(m_tmp2.M));

//    for(i = 0; i < size; i++)
//    {
//        m_tmp2.M[i][i] = 1;
//    }
//    m_tmp2.s_m = m_tmp2.s_n = size;

//    for(k = 0; k < size; k++)
//    {
//        main_el( m_tmp1, m_tmp2, k);
//        first = m_tmp1.M[k][k];
//        if (fabs(first) < ZERO_TOLERANCE_M)
//        {
//            return false;
//        }
//        for(j = k; j < size; j++)
//        {
//            m_tmp1.M[k][j] /= first;
//        }
//        for(j = 0; j < size; j++)
//        {
//            m_tmp2.M[k][j] /= first;
//        }
//        if(k < size-1)
//        {
//            for(i = k+1; i< size; i++)
//            {
//                first = m_tmp1.M[i][k];
//                for(j = k; j < size; j++)
//                {
//                    m_tmp1.M[i][j] -= m_tmp1.M[k][j]*first;
//                }
//                for(j = 0; j < size; j++)
//                {
//                    m_tmp2.M[i][j] -= m_tmp2.M[k][j]*first;
//                }
//            }
//        }
//    }
//    Inv(m_tmp1,m_tmp2,I);
//    if (b_mult)
//    {
//        I = I * multiplicator;
//    }
//    return true;
//}


template <int _size_m>
bool TCMatrixFunc<_size_m>::InvMatrix(TMatrix<_size_m> &M, TMatrix<_size_m> &I)
{
    bool bOK = true;
    bool bSmallDet = false;
    bOK = Inv_Gauss(M, I, bSmallDet);
    if (bSmallDet)
    {
        bOK = false;
        double max_el = M.GetMaxAbs();
        if (max_el > ZERO_TOLERANCE_INV)
        {
            double multiplicator = max_el;
            if (max_el < 1.)
            {
                multiplicator = 1./max_el;
            }

            TMatrix<_size_m> M1;
            TMatrix<_size_m> I1;
            M1 = M * multiplicator;
            bOK = Inv_Gauss(M1, I1, bSmallDet);
            if (bOK)
            {
                I = I1 * multiplicator;
            }
        }
    }
    return bOK;
}


template <int _size_m>
bool TCMatrixFunc<_size_m>::InvMatrix(TMatrix<_size_m> &M, TMatrix<_size_m> &I, double &DetM)
{
    bool bOK = true;
    bool bSmallDet = false;
    bOK = Inv_Gauss(M, I, bSmallDet, DetM);
    if (bSmallDet)
    {
        bOK = false;
        double max_el = M.GetMaxAbs();
        if (max_el > ZERO_TOLERANCE_INV)
        {
            double multiplicator = max_el;
            if (max_el < 1.)
            {
                multiplicator = 1./max_el;
            }

            TMatrix<_size_m> M1;
            TMatrix<_size_m> I1;
            M1 = M * multiplicator;
            bOK = Inv_Gauss(M1, I1, bSmallDet, DetM);
            if (bOK)
            {
                I = I1 * multiplicator;
                for (int i=0; i< M.s_m; i++)
                {
                    DetM /= multiplicator;
                }
            }
        }
    }
    return bOK;
}


template <int _size_m>
bool TCMatrixFunc<_size_m>::InvMatrixLowerTriangular(TMatrix<_size_m> &M, TMatrix<_size_m> &ResInv)
{
    bool bOK = true; //true if result is OK
    if (M.s_m == M.s_n && M.s_m > 0 && M.s_m <= _size_m)
    {
        ResInv.Reset(M.s_m, M.s_n);
        int i, j, k;
        int Step = _size_m;

        double *Res_ii = &ResInv.M[0][0];
        double *Res_i0 = Res_ii;
        double *Mii = &M.M[0][0];
        for (i=0; i<M.s_m; i++, Res_ii+=Step+1, Mii+=Step+1, Res_i0+=Step) //Gauss-Jordan method of inversion, applied for lower triangular matrix
        {
            *Res_ii = 1.;
            double dM_i_i = *Mii;
            if (fabs(dM_i_i) > ZERO_TOLERANCE_M)
            {
                double *Res_ik = Res_i0;
                for (k=0; k<=i; k++, Res_ik++)
                {
                    *Res_ik /= dM_i_i;
                }

                if (i < M.s_m - 1)
                {
                    double *Res_j0 = Res_i0+Step;
                    for (j=i+1; j<M.s_m; j++, Res_j0+=Step)
                    {
                        double dM_j_i = M.M[j][i];
                        double *Res_ik2 = Res_i0;
                        double *Res_jk = Res_j0;
                        for (k=0; k<=i; k++, Res_ik2++, Res_jk++)
                        {
                            *Res_jk -= *Res_ik2 * dM_j_i;
                        }
                    }
                }
            }
            else
            {
                bOK = false;
                break;
            }
        }
    }
    else
    {
        bOK = false;
    }

    return bOK;
}


template <int _size_m>
bool TCMatrixFunc<_size_m>::InvMatrixSymmPos(TMatrix<_size_m> &M, TMatrix<_size_m> &ResInv)
{
    bool bOK = true; //true if result is OK
    if (M.s_m == M.s_n && M.s_m > 0 && M.s_m <= _size_m)
    {
        TMatrix<_size_m> Asqrt; //"matrix square root", i.e. M = Asqrt * AsqrtT
        bOK = CholeskyDecomposition(M, Asqrt);
        if (bOK)
        {
            TMatrix<_size_m> AsqrtInv; //inverse matrix to Asqrt
            bOK = InvMatrixLowerTriangular(Asqrt, AsqrtInv);
            if (bOK)
            {                
                bOK = MatrLTL_UpperTriangToLowerTriang(AsqrtInv, ResInv);
            }
        }
    }
    else
    {
        bOK = false;
    }
    return bOK;
}


template <int _size_m>
bool TCMatrixFunc<_size_m>::InvMatrixSymmPos(TMatrix<_size_m> &M, TMatrix<_size_m> &ResInv, double &DetM)
{
    bool bOK = true; //true if result is OK
    if (M.s_m == M.s_n && M.s_m > 0 && M.s_m <= _size_m)
    {
        TMatrix<_size_m> Asqrt; //"matrix square root", i.e. M = Asqrt * AsqrtT
        double DetCh;
        bOK = CholeskyDecomposition(M, Asqrt, DetCh);
        if (bOK)
        {
            DetM = sqr(DetCh);
            TMatrix<_size_m> AsqrtInv; //inverse matrix to Asqrt
            bOK = InvMatrixLowerTriangular(Asqrt, AsqrtInv);
            if (bOK)
            {
                bOK = MatrLTL_UpperTriangToLowerTriang(AsqrtInv, ResInv);
            }
        }
    }
    else
    {
        bOK = false;
    }
    return bOK;
}


template <int _size_m>
bool TCMatrixFunc<_size_m>::InvMatrix(Matrix3x3 &M, Matrix3x3 &A)
{
//	double a,b,c,d;
    double det;
    Det(M,det);
    if(fabs(det) < ZERO_TOLERANCE_M) return false;
    memcpy(A, M, sizeof(Matrix3x3));//!!!!
    /*if(A[0][0] == 0)
        return false;

    for(int k = 0; k < 3; k++)
    {
        a = A[1][0]/A[0][0];
        b = -a;
        c = A[1][1] - a*A[0][1];
        d = A[1][2] - a*A[0][2];
        a = A[2][0]/A[0][0];
        A[1][2] = -a;
        A[1][0] = A[2][1] - a*A[0][1];
        A[1][1] = A[2][2] - a*A[0][2];
        A[2][0] = A[0][1]/A[0][0];
        A[2][1] = A[0][2]/A[0][0];
        A[2][2] = 1./A[0][0];
        A[0][0] = c;
        A[0][1] = d;
        A[0][2] = b;
    }*/
    A[0][0] = (M[1][1]*M[2][2] - M[1][2]*M[2][1]) / det;
    A[0][1] = (M[0][2]*M[2][1] - M[0][1]*M[2][2]) / det;
    A[0][2] = (M[0][1]*M[1][2] - M[0][2]*M[1][1]) / det;
    A[1][0] = (M[2][0]*M[1][2] - M[1][0]*M[2][2]) / det;
    A[1][1] = (M[0][0]*M[2][2] - M[0][2]*M[2][0]) / det;
    A[1][2] = (M[1][0]*M[0][2] - M[0][0]*M[1][2]) / det;
    A[2][0] = (M[1][0]*M[2][1] - M[1][1]*M[2][0]) / det;
    A[2][1] = (M[0][1]*M[2][0] - M[0][0]*M[2][1]) / det;
    A[2][2] = (M[0][0]*M[1][1] - M[0][1]*M[1][0]) / det;
    return true;
}
template <int _size_m>
bool TCMatrixFunc<_size_m>::InvMatrix3x3(TMatrix<_size_m> &M_in, TMatrix<_size_m> &M_out)
{
    if(M_in.s_m != 3 || M_in.s_n != 3) {
        return false;
    }

    double det =
            + M_in.M[0][0]*M_in.M[1][1]*M_in.M[2][2]
            - M_in.M[0][0]*M_in.M[1][2]*M_in.M[2][1]
            - M_in.M[0][1]*M_in.M[1][0]*M_in.M[2][2]
            + M_in.M[0][1]*M_in.M[2][0]*M_in.M[1][2]
            + M_in.M[1][0]*M_in.M[0][2]*M_in.M[2][1]
            - M_in.M[0][2]*M_in.M[1][1]*M_in.M[2][0];

    if(fabs(det) < ZERO_TOLERANCE_M) {
        return false;
    }

    M_out.M[0][0] = (M_in.M[1][1]*M_in.M[2][2] - M_in.M[1][2]*M_in.M[2][1]) / det;
    M_out.M[0][1] = (M_in.M[0][2]*M_in.M[2][1] - M_in.M[0][1]*M_in.M[2][2]) / det;
    M_out.M[0][2] = (M_in.M[0][1]*M_in.M[1][2] - M_in.M[0][2]*M_in.M[1][1]) / det;
    M_out.M[1][0] = (M_in.M[2][0]*M_in.M[1][2] - M_in.M[1][0]*M_in.M[2][2]) / det;
    M_out.M[1][1] = (M_in.M[0][0]*M_in.M[2][2] - M_in.M[0][2]*M_in.M[2][0]) / det;
    M_out.M[1][2] = (M_in.M[1][0]*M_in.M[0][2] - M_in.M[0][0]*M_in.M[1][2]) / det;
    M_out.M[2][0] = (M_in.M[1][0]*M_in.M[2][1] - M_in.M[1][1]*M_in.M[2][0]) / det;
    M_out.M[2][1] = (M_in.M[0][1]*M_in.M[2][0] - M_in.M[0][0]*M_in.M[2][1]) / det;
    M_out.M[2][2] = (M_in.M[0][0]*M_in.M[1][1] - M_in.M[0][1]*M_in.M[1][0]) / det;
    M_out.s_m = M_in.s_m;
    M_out.s_n = M_in.s_n;
    return true;

}

template <int _size_m>
bool TCMatrixFunc<_size_m>::VecXVec(Vector3 &V1, Vector3 &V2, Matrix3x3 &A)
{
    A[0][0] = V1[0] * V2[0];
    A[0][1] = V1[0] * V2[1];
    A[0][2] = V1[0] * V2[2];

    A[1][0] = V1[1] * V2[0];
    A[1][1] = V1[1] * V2[1];
    A[1][2] = V1[1] * V2[2];

    A[2][0] = V1[2] * V2[0];
    A[2][1] = V1[2] * V2[1];
    A[2][2] = V1[2] * V2[2];

    return true;
}

template <int _size_m>
bool TCMatrixFunc<_size_m>::VecXVec(TVector<_size_m> &V1, TVector<_size_m> &V2, TMatrix<_size_m> &A)
{
    int i,j;
    for(i = 0; i < V1.s; i++)
    {
        for(j = 0; j < V2.s; j++)
        {
            A.M[i][j] = V1.Vec[i] * V2.Vec[j];
        }
    }
    A.s_m = V1.s;
    A.s_n = V2.s;
    return true;
}

template <int _size_m>
bool TCMatrixFunc<_size_m>::Add(TMatrix<_size_m> &M1, TMatrix<_size_m> &M2, TMatrix<_size_m> &A_out)
{
    if (M1.s_m == 0 && M1.s_n == 0) //Add TaniaP
    {
        M1.s_m = M2.s_m;
        M1.s_n = M2.s_n;
    }

    if (M2.s_m == 0 && M2.s_n == 0)
    {
        M2.s_m = M1.s_m;
        M2.s_n = M1.s_n;
    }

    if((M1.s_m != M2.s_m)||(M1.s_n != M2.s_n))
        return false;

    int i,j;
    for(i=0;i<M1.s_m;i++)
    {
        for(j=0;j<M1.s_n;j++)
        {
            A_out.M[i][j] = M1.M[i][j] + M2.M[i][j];
        }
    }
    A_out.s_m = M1.s_m;
    A_out.s_n = M1.s_n;

    return true;
}


template <int _size_m>
bool TCMatrixFunc<_size_m>::Subtr(TMatrix<_size_m> &M1, TMatrix<_size_m> &M2, TMatrix<_size_m> &A_out)
{
    if (M1.s_m == 0 && M1.s_n == 0) //Add TaniaP
    {
        M1.s_m = M2.s_m;
        M1.s_n = M2.s_n;
    }

    if (M2.s_m == 0 && M2.s_n == 0)
    {
        M2.s_m = M1.s_m;
        M2.s_n = M1.s_n;
    }

    if((M1.s_m != M2.s_m)||(M1.s_n != M2.s_n))
        return false;

    int i,j;
    for(i=0;i<M1.s_m;i++)
    {
        for(j=0;j<M1.s_n;j++)
        {
            A_out.M[i][j] = M1.M[i][j] - M2.M[i][j];
        }
    }
    A_out.s_m = M1.s_m;
    A_out.s_n = M1.s_n;

    return true;
}


template <int _size_m>
TMatrix<_size_m> TCMatrixFunc<_size_m>::Add(TMatrix<_size_m> &M1, TMatrix<_size_m> &M2)
{
    TMatrix<_size_m> A_out;

    if (M1.s_m == 0 && M1.s_n == 0)
    {
        M1.s_m = M2.s_m;
        M1.s_n = M2.s_n;
    }

    if (M2.s_m == 0 && M2.s_n == 0)
    {
        M2.s_m = M1.s_m;
        M2.s_n = M1.s_n;
    }

    if((M1.s_m != M2.s_m)||(M1.s_n != M2.s_n))
        return A_out;

    int i,j;
    for(i=0;i<M1.s_m;i++)
    {
        for(j=0;j<M1.s_n;j++)
        {
            A_out.M[i][j] = M1.M[i][j] + M2.M[i][j];
        }
    }
    A_out.s_m = M1.s_m;
    A_out.s_n = M1.s_n;

    return A_out;
}


template <int _size_m>
bool TCMatrixFunc<_size_m>::Add(Matrix3x3 &M1, Matrix3x3 &M2, Matrix3x3 &A_out)
{

    A_out[0][0] = M1[0][0] + M2[0][0];
    A_out[0][1] = M1[0][1] + M2[0][1];
    A_out[0][2] = M1[0][2] + M2[0][2];

    A_out[1][0] = M1[1][0] + M2[1][0];
    A_out[1][1] = M1[1][1] + M2[1][1];
    A_out[1][2] = M1[1][2] + M2[1][2];

    A_out[2][0] = M1[2][0] + M2[2][0];
    A_out[2][1] = M1[2][1] + M2[2][1];
    A_out[2][2] = M1[2][2] + M2[2][2];

    return true;
}

template <int _size_m>
bool TCMatrixFunc<_size_m>::Transpon(TMatrix<_size_m> &M1, TMatrix<_size_m> &A_out)
{
    int i,j;
    for(i=0;i<M1.s_n;i++)
    {
        for(j=0;j<M1.s_m;j++)
        {
            A_out.M[i][j] = M1.M[j][i];
        }
    }
    A_out.s_m = M1.s_n;
    A_out.s_n = M1.s_m;
    return true;
}

template <int _size_m>
bool TCMatrixFunc<_size_m>::Transpon(Matrix3x3 &M1, Matrix3x3 &A_out)
{

    A_out[0][0] = M1[0][0];
    A_out[0][1] = M1[1][0];
    A_out[0][2] = M1[2][0];

    A_out[1][0] = M1[0][1];
    A_out[1][1] = M1[1][1];
    A_out[1][2] = M1[2][1];

    A_out[2][0] = M1[0][2];
    A_out[2][1] = M1[1][2];
    A_out[2][2] = M1[2][2];

    return true;
}

template <int _size_m>
TMatrix<_size_m> TCMatrixFunc<_size_m>::Transpon(TMatrix<_size_m> &M1)
{
    TMatrix<_size_m> A_out;
    int i,j;
    for(i=0;i<M1.s_n;i++)
    {
        for(j=0;j<M1.s_m;j++)
        {
            A_out.M[i][j] = M1.M[j][i];
        }
    }
    A_out.s_m = M1.s_n;
    A_out.s_n = M1.s_m;

    return A_out;
}

template <int _size_m>
bool TCMatrixFunc<_size_m>::VecXVec(TVector<_size_m> &V1, TVector<_size_m> &V2, double &a)
{
    if(V1.s != V2.s)
        return false;
    int i;
    a = 0;
    for(i=0;i<V1.s;i++)
    {
        a += V1.Vec[i]*V2.Vec[i];
    }
    return true;
}


template <int _size_m>
bool TCMatrixFunc<_size_m>::VecXVec(Vector3 &V1, Vector3 &V2, double &a)
{
    a = V1[0]*V2[0] + V1[1]*V2[1] + V1[2]*V2[2];
    return true;
}


template <int _size_m>
void TCMatrixFunc<_size_m>::Mult(double &C, TVector<_size_m> &V, TVector<_size_m> &A_out)
{
    A_out.s = V.s;

    for(int i=0; i<V.s; i++)
        A_out.Vec[i] = C * V.Vec[i];

    return;
}


template <int _size_m>
void TCMatrixFunc<_size_m>::Mult(double &C, TMatrix<_size_m> &M, TMatrix<_size_m> &A_out)
{
    A_out.s_m = M.s_m;
    A_out.s_n = M.s_n;

    for (int i=0; i<M.s_m; i++)
        for(int j=0; j<M.s_n; j++)
            A_out.M[i][j] = C * M.M[i][j];

    return;
}


template <int _size_m>
void TCMatrixFunc<_size_m>::Negative(TMatrix<_size_m> &M, TMatrix<_size_m> &A_out)
{
    A_out.s_m = M.s_m;
    A_out.s_n = M.s_n;

    for (int i=0; i<M.s_m; i++)
        for (int j=0; j<M.s_n; j++)
            A_out.M[i][j] = -M.M[i][j];

    return;
}


template <int _size_m>
void TCMatrixFunc<_size_m>::Negative(TVector<_size_m> &V, TVector<_size_m> &A_out)
{
    A_out.s = V.s;

    for (int i=0; i<V.s; i++)
        A_out.Vec[i] = -V.Vec[i];

    return;
}


template <int _size_m>
bool TCMatrixFunc<_size_m>::Submatrix(TMatrix<_size_m> &M, int r1, int r2, int c1, int c2, TMatrix<_size_m> &A_out)
{
    if ( r1>r2 || c1>c2 )
        return false;

    if ( r2 >= M.s_m || c2 >= M.s_n )
        return false;

    A_out.s_m = r2-r1+1;
    A_out.s_n = c2-c1+1;

    for (int i=r1; i<=r2; i++)
    {
        for (int j=c1; j<=c2; j++)
            A_out.M[i-r1][j-c1] = M.M[i][j];
    }

    return true;
}


template <int _size_m>
bool TCMatrixFunc<_size_m>::CholeskyDecomposition(TMatrix<_size_m> &A_inp, TMatrix<_size_m> &L_out)
{
    L_out.Reset(A_inp.s_m, A_inp.s_n);
    bool bOK = true;
    if (A_inp.s_m > 0 && A_inp.s_m <= _size_m && A_inp.s_n == A_inp.s_m)
    {
        int i=0;
        int j=0;
        int k=0;
        int Step = _size_m;
        const double *Ai0 = &A_inp.M[0][0];
        double *Li0 = &L_out.M[0][0];
        double *Lii = Li0;
        const double *Aii = Ai0;
        for (i=0; i<A_inp.s_m; i++, Lii+=Step+1, Aii+=Step+1, Ai0+=Step, Li0+=Step)
        {
            double dA_i_i = *Aii;
            if (i == 0)
            {
                if (dA_i_i > ZERO_TOLERANCE_M)
                {
                    *Lii = sqrt(dA_i_i);
                }
                else
                {
                    bOK = false;
                    break;
                }
            }
            else
            {
                const double *Aij = Ai0;
                double *Lij = Li0;
                const double *Ljj = &L_out.M[0][0];
                const double *Lj0 = &L_out.M[0][0];
                for (j=0; j<i; j++, Lij++, Aij++, Ljj+=Step+1, Lj0+=Step)
                {
                    if (j==0)
                    {
                        *Lij = *Aij / (*Ljj);
                    }
                    else
                    {
                        double curr_sum = 0;
                        const double *Lik = Li0;
                        const double *Ljk = Lj0;
                        for (k=0; k<j; k++, Lik++, Ljk++)
                        {
                            curr_sum += (*Lik) * (*Ljk);
                        }

                        *Lij = ( *Aij - curr_sum ) / (*Ljj);
                    }
                }

                double curr_sum1 = 0;
                const double *Lik1 = Li0;
                for (k=0; k<i; k++, Lik1++)
                {
                    curr_sum1 += (*Lik1) * (*Lik1);
                }

                double Delta = dA_i_i - curr_sum1;
                if (Delta > ZERO_TOLERANCE_M)
                {
                    *Lii = sqrt(Delta);
                }
                else
                {
                    bOK = false;
                    break;
                }
            }

            if (*Lii < ZERO_TOLERANCE_M)
            {
                bOK = false;
                break;
            }
        }
    }
    else
    {
        bOK = false;
    }
    return bOK;
}


template <int _size_m>
bool TCMatrixFunc<_size_m>::CholeskyDecomposition(TMatrix<_size_m> &A_inp, TMatrix<_size_m> &L_out, double &DetCh)
{
    L_out.Reset(A_inp.s_m, A_inp.s_n);
    bool bOK = true;
    DetCh = 1.;
    if (A_inp.s_m > 0 && A_inp.s_m <= _size_m && A_inp.s_n == A_inp.s_m)
    {
        int i=0;
        int j=0;
        int k=0;
        int Step = _size_m;
        const double *Ai0 = &A_inp.M[0][0];
        double *Li0 = &L_out.M[0][0];
        double *Lii = Li0;
        const double *Aii = Ai0;
        for (i=0; i<A_inp.s_m; i++, Lii+=Step+1, Aii+=Step+1, Ai0+=Step, Li0+=Step)
        {
            double dA_i_i = *Aii;
            if (i == 0)
            {
                if (dA_i_i > ZERO_TOLERANCE_M)
                {
                    *Lii = sqrt(dA_i_i);
                }
                else
                {
                    bOK = false;
                    break;
                }
            }
            else
            {
                const double *Aij = Ai0;
                double *Lij = Li0;
                const double *Ljj = &L_out.M[0][0];
                const double *Lj0 = &L_out.M[0][0];
                for (j=0; j<i; j++, Lij++, Aij++, Ljj+=Step+1, Lj0+=Step)
                {
                    if (j==0)
                    {
                        *Lij = *Aij / (*Ljj);
                    }
                    else
                    {
                        double curr_sum = 0;
                        const double *Lik = Li0;
                        const double *Ljk = Lj0;
                        for (k=0; k<j; k++, Lik++, Ljk++)
                        {
                            curr_sum += (*Lik) * (*Ljk);
                        }

                        *Lij = ( *Aij - curr_sum ) / (*Ljj);
                    }
                }

                double curr_sum1 = 0;
                const double *Lik1 = Li0;
                for (k=0; k<i; k++, Lik1++)
                {
                    curr_sum1 += (*Lik1) * (*Lik1);
                }

                double Delta = dA_i_i - curr_sum1;
                if (Delta > ZERO_TOLERANCE_M)
                {
                    *Lii = sqrt(Delta);
                }
                else
                {
                    bOK = false;
                    break;
                }
            }

            if (*Lii < ZERO_TOLERANCE_M)
            {
                bOK = false;
                break;
            }
            DetCh *= *Lii;
        }
    }
    else
    {
        bOK = false;
    }
    return bOK;
}


// DiVA Add---begin----------------------

template <int _size_m>
double TCMatrixFunc<_size_m>::VecXVec(TVector<_size_m> &V1, TVector<_size_m> &V2)
{
    double a;

    if(V1.s != V2.s)
    {
        a = 0; //!!!!!!!!!
        return a;
    }

    int i;
    a = 0;
    for(i=0;i<V1.s;i++)
    {
        a += V1.Vec[i]*V2.Vec[i];
    }
    return a;
}
template <int _size_m>
double TCMatrixFunc<_size_m>::VecXVec(Vector3 &V1, Vector3 &V2)
{
    double a;

    a = V1[0]*V2[0] + V1[1]*V2[1] + V1[2]*V2[2];

    return a;
}

template <int _size_m>
bool TCMatrixFunc<_size_m>::VecXVec_vect(Vector3 &V1, Vector3 &V2, Vector3 &A)
{
    A[0] = V1[1]*V2[2] - V1[2]*V2[1];   // Y1*Z2 - Y2*Z1
    A[1] = V1[2]*V2[0] - V1[0]*V2[2];	// Z1*X2 - X1*Z2
    A[2] = V1[0]*V2[1] - V1[1]*V2[0];	// X1*Y2 - Y1*X2

    return true;
}

template <int _size_m>
bool TCMatrixFunc<_size_m>::VecXVec_vect(TVector<_size_m> &V1, TVector<_size_m> &V2, TVector<_size_m> &A)
{
    if(V1.s != V2.s)
        return false;

    A.Vec[0] = V1.Vec[1]*V2.Vec[2] - V1.Vec[2]*V2.Vec[1];   // Y1*Z2 - Y2*Z1
    A.Vec[1] = V1.Vec[2]*V2.Vec[0] - V1.Vec[0]*V2.Vec[2];	// Z1*X2 - X1*Z2
    A.Vec[2] = V1.Vec[0]*V2.Vec[1] - V1.Vec[1]*V2.Vec[0];	// X1*Y2 - Y1*X2

    A.s = V1.s;

    return true;
}

/*Vector3 CMatrix::VecXVec_vect(Vector3 &V1, Vector3 &V2)
{
    Vector3 A;
    A[0] = V1[1]*V2[2] - V1[2]*V2[1];   // Y1*Z2 - Y2*Z1
    A[1] = V1[2]*V2[0] - V1[0]*V2[2];	// Z1*X2 - X1*Z2
    A[2] = V1[0]*V2[1] - V1[1]*V2[0];	// X1*Y2 - Y1*X2

    return A;
}*/

template <int _size_m>
TVector<_size_m> TCMatrixFunc<_size_m>::VecXVec_vect(TVector<_size_m> &V1, TVector<_size_m> &V2)
{
    TVector<_size_m> A;

    if (V1.s == V2.s)
    {
        A.Vec[0] = V1.Vec[1]*V2.Vec[2] - V1.Vec[2]*V2.Vec[1];   // Y1*Z2 - Y2*Z1
        A.Vec[1] = V1.Vec[2]*V2.Vec[0] - V1.Vec[0]*V2.Vec[2];	// Z1*X2 - X1*Z2
        A.Vec[2] = V1.Vec[0]*V2.Vec[1] - V1.Vec[1]*V2.Vec[0];	// X1*Y2 - Y1*X2

        A.s = V1.s;
    }

    return A;

}

template <int _size_m>
bool TCMatrixFunc<_size_m>::ScalarVector(Vector3 &V1, double &a)
{
    a = sqrt(sqr(V1[0]) + sqr(V1[1]) + sqr(V1[2]));

    return true;
}

template <int _size_m>
bool TCMatrixFunc<_size_m>::ScalarVector(TVector<_size_m> &V1, double &a)
{
    double sumEl = 0.0;

    for(int i = 0; i < V1.s; i++)
        sumEl += sqr(V1.Vec[i]);

    a = sqrt(sumEl);

    return true;
}

template <int _size_m>
double TCMatrixFunc<_size_m>::ScalarVector(Vector3 &V1)
{
    double a;

    a = sqrt(sqr(V1[0]) + sqr(V1[1]) + sqr(V1[2]));

    return a;
}

template <int _size_m>
double TCMatrixFunc<_size_m>::ScalarVector(TVector<_size_m> &V1)
{
    double sumEl = 0.0, a;

    for(int i = 0; i < V1.s; i++)
        sumEl += sqr(V1.Vec[i]);

    a = sqrt(sumEl);

    return a;
}

template <int _size_m>
TVector<_size_m> TCMatrixFunc<_size_m>::Add(TVector<_size_m>& V1, TVector<_size_m>& V2)
{
    TVector<_size_m> A;

//	if (V1.s != V2.s)
//		return A;

    if (V1.s == 0) //Add TaniaP
        V1.s = V2.s;

    if (V2.s == 0)
        V2.s = V1.s;

    if (V1.s == V2.s)
    {
        for (int i = 0; i < V1.s; i++)
            A.Vec[i] = V1.Vec[i] + V2.Vec[i];
        A.s = V1.s;
    }

    return A;
}


template <int _size_m>
bool TCMatrixFunc<_size_m>::Add(TVector<_size_m> &V1, TVector<_size_m> &V2, TVector<_size_m> &A_out)
{
    if (V1.s == 0) //Add TaniaP
        V1.s = V2.s;

    if (V2.s == 0)
        V2.s = V1.s;

    if (V1.s != V2.s)
        return false;

    A_out.s = V1.s;

    for (int i=0; i<V1.s; i++)
        A_out.Vec[i] = V1.Vec[i] + V2.Vec[i];

    return true;
}


template <int _size_m>
bool TCMatrixFunc<_size_m>::Subtr(TVector<_size_m> &V1, TVector<_size_m> &V2, TVector<_size_m> &A_out)
{
    if (V1.s == 0) //Add TaniaP
        V1.s = V2.s;

    if (V2.s == 0)
        V2.s = V1.s;

    if (V1.s != V2.s)
        return false;

    A_out.s = V1.s;

    for (int i=0; i<V1.s; i++)
        A_out.Vec[i] = V1.Vec[i] - V2.Vec[i];

    return true;
}

template <int _size_m>
bool TCMatrixFunc<_size_m>::IminusMatrix(TMatrix<_size_m> &M, TMatrix<_size_m> &IminusM)
{
    if(M.s_m!=M.s_n)
        return false;

    IminusM.s_m = M.s_m;
    IminusM.s_n = M.s_n;
    for(int i=0;i<M.s_m;i++)
    {
        for(int j=0;j<M.s_n;j++)
        {
            if(i==j)
            {
                IminusM.M[i][j]=1-M.M[i][j];
            }
            else
            {
                IminusM.M[i][j]=-M.M[i][j];
            }
        }
    }
    return true;
}

template <int _size_m>
bool TCMatrixFunc<_size_m>::OuterProduct(TVector<_size_m> &V1, TVector<_size_m> &V2,TMatrix<_size_m> &M)
{
    M.s_m = V1.s;
    M.s_n = V2.s;
    for(int i=0;i<M.s_m;i++)
    {
        for(int j=0;j<M.s_n;j++)
        {
            M.M[i][j] = V1.Vec[i]*V2.Vec[j];
        }
    }
    return true;
}

//-----Concatenation-----

template <int _size_m>
template<int _size_m1, int _size_m2, int _size_m3, int _size_m4>
bool TCMatrixFunc<_size_m>::Blkdiag3(TMatrix<_size_m1> &M1, TMatrix<_size_m2> &M2, TMatrix<_size_m3> &M3, TMatrix<_size_m4> &A)
{
    //if(_size_m1+_size_m2+_size_m3 >_size_m4)
    //    return false;
    int sm = M1.s_m+M2.s_m+M3.s_m;
    int sn = M1.s_n+M2.s_n+M3.s_n;

    if(sm >_size_m4 || sn >_size_m4 )
          return false;

    A.Reset();
    A.s_m=sm;
    A.s_n=sn;

    for (int i = 0; i <M1.s_m; i++)
    {
        for (int j = 0; j <M1.s_n; j++)
        {
            A.M[i][j] = M1.M[i][j];
        }
    }
    int off_m = M1.s_m, off_n = M1.s_n;
    for (int i = 0; i <M2.s_m; i++)
    {
        for (int j = 0; j <M2.s_n; j++)
        {
            A.M[i+off_m][j+off_n] = M2.M[i][j];
        }
    }
    off_m = M1.s_m+M2.s_m; off_n = M1.s_n+M2.s_n;
    for (int i = 0; i <M3.s_m; i++)
    {
        for (int j = 0; j <M3.s_n; j++)
        {
            A.M[i+off_m][j+off_n] = M3.M[i][j];
        }
    }
    return true;
}


template <int _size_m>
template <int _size_m1, int _size_m2, int _size_m3>
bool TCMatrixFunc<_size_m>::Blkdiag2(TMatrix<_size_m1> &M1, TMatrix<_size_m2> &M2, TMatrix<_size_m3> &A)
{
    int sm = M1.s_m + M2.s_m;
    int sn = M1.s_n + M2.s_n;

    if(sm >_size_m3 || sn >_size_m3 )
        return false;

    A.Reset(sm, sn);

    for (int i = 0; i <M1.s_m; i++)
    {
        for (int j = 0; j <M1.s_n; j++)
        {
            A.M[i][j] = M1.M[i][j];
        }
    }
    int off_m = M1.s_m, off_n = M1.s_n;
    for (int i = 0; i <M2.s_m; i++)
    {
        for (int j = 0; j <M2.s_n; j++)
        {
            A.M[i+off_m][j+off_n] = M2.M[i][j];
        }
    }
    return true;
}


template <int _size_m>
template <int _size_m1, int _size_m2, int _size_m3, int _size_m4, int _size_m5>
bool TCMatrixFunc<_size_m>::Blkdiag4(TMatrix<_size_m1> &M1, TMatrix<_size_m2> &M2, TMatrix<_size_m3> &M3, TMatrix<_size_m4> &M4, TMatrix<_size_m5> &A)
{
    bool bOK = true;

    TMatrix<_size_m1 + _size_m2 + _size_m3> Matr;

    bOK = Blkdiag3(M1, M2, M3, Matr);

    if (bOK)
    {
        bOK = Blkdiag2(Matr, M4, A);
    }

    return bOK;
}


template <int _size_m>
bool TCMatrixFunc<_size_m>::ConcatLeftRight(TMatrix<_size_m> &MLeft, TMatrix<_size_m> &MRight, TMatrix<_size_m> &A)
{
    if (MLeft.s_m != MRight.s_m)
        return false;

    int i, j;

    A.s_m = MLeft.s_m;
    A.s_n = MLeft.s_n + MRight.s_n;

    for (i=0; i<A.s_m; i++)
    {
        for (j=0; j<MLeft.s_n; j++)
            A.M[i][j] = MLeft.M[i][j];

        for (j=0; j<MRight.s_n; j++)
            A.M[i][j+MLeft.s_n] = MRight.M[i][j];
    }

    return true;
}


template <int _size_m>
bool TCMatrixFunc<_size_m>::ConcatUpDown(TMatrix<_size_m> &MUp, TMatrix<_size_m> &MDown, TMatrix<_size_m> &A)
{
    if (MUp.s_n != MDown.s_n)
        return false;

    int i, j;

    A.s_m = MUp.s_m + MDown.s_m;
    A.s_n = MUp.s_n;

    for (j=0; j<A.s_n; j++)
    {
        for (i=0; i<MUp.s_m; i++)
            A.M[i][j] = MUp.M[i][j];

        for (i=0; i<MDown.s_m; i++)
            A.M[i+MUp.s_m][j] = MDown.M[i][j];
    }

    return true;
}

template <int _size_m>
bool TCMatrixFunc<_size_m>::ConcatLeftRight(TMatrix<_size_m> &MLeft, TMatrix<_size_m> &MCenter, TMatrix<_size_m> &MRight, TMatrix<_size_m> &A)
{
    if (MLeft.s_m != MCenter.s_m || MLeft.s_m != MRight.s_m)
        return false;

    TMatrix<_size_m> MCurr;
    memset(&MCurr, 0, sizeof(MCurr));

    if (ConcatLeftRight(MLeft, MCenter, MCurr))
    {
        if (ConcatLeftRight(MCurr, MRight, A))
            return true;
        else
            return false;
    }
    else
        return false;
}


template <int _size_m>
bool TCMatrixFunc<_size_m>::ConcatUpDown(TMatrix<_size_m> &MUp, TMatrix<_size_m> &MCenter, TMatrix<_size_m> &MDown, TMatrix<_size_m> &A)
{
    if (MUp.s_n != MCenter.s_n || MUp.s_n != MDown.s_n)
        return false;

    TMatrix<_size_m> MCurr;
    memset(&MCurr, 0, sizeof(MCurr));

    if (ConcatUpDown(MUp, MCenter, MCurr))
    {
        if (ConcatUpDown(MCurr, MDown, A))
            return true;
        else
            return false;
    }
    else
        return false;
}

template<int _size_m>
template <int _size_m1,int _size_m2>
bool TCMatrixFunc<_size_m>::Copy(TVector<_size_m1> &To, TVector<_size_m2> &From)
{
 //copy From to To, if size(capacity) of From is less or equal to size of To
    if(From.s>_size_m1)
        return false;

    To.Reset();
    To.s = From.s;
    int i = 0;
    while(i < To.s)
    {
        To.Vec[i]=From.Vec[i];
        ++i;
    }
    return true;
}

template<int _size_m>
template <int _size_m1,int _size_m2>
bool TCMatrixFunc<_size_m>::Copy(TMatrix<_size_m1> &To, TMatrix<_size_m2> &From)
{
 //copy From to To, if size(capacity) of From is less or equal to size of To
//    if(_size_m1<_size_m2)
    if (_size_m1 < From.s_m || _size_m1 < From.s_n)
        return false;

    To.Reset();
    To.s_m = From.s_m;
    To.s_n = From.s_n;
    int i = 0, j=0;
    while(i < To.s_m)
    {
        while(j < To.s_n)
        {
            To.M[i][j]=From.M[i][j];
            ++j;
        }
        j=0;
        ++i;
    }
    return true;
}
//template<int _size_m>
//template <int _size_m1,int str, int col>
//bool TCMatrixFunc<_size_m>::Copy(TMatrix<_size_m1> &To, double (&From)[str][col])
//{
//    if(str>_size_m1 || col>_size_m1)
//        return false;

//    To.Reset();
//    To.s_m = str;
//    To.s_n = col;
//    int i = 0, j=0;
//    while(i < To.s_m)
//    {
//        while(j < To.s_n)
//        {
//            To.M[i][j]=From[i][j];
//            ++j;
//        }
//        j=0;
//        ++i;
//    }
//    return true;
//}

template<int _size_m>
template <int _size_m1, int col>
bool TCMatrixFunc<_size_m>::Copy(TVector<_size_m1> &To, const double (&From)[col])
{
    if(col>_size_m1)
        return false;

    To.Reset();
    To.s = col;
    int i = 0;
    while(i < To.s)
    {
        To.Vec[i]=From[i];
        ++i;
    }
    return true;
}


template<int _size_m>
template <int _size_m1, int col>
bool TCMatrixFunc<_size_m>::Copy(TVector<_size_m1> &To, double (&From)[col])
{
    if(col>_size_m1)
        return false;

    To.Reset();
    To.s = col;
    int i = 0;
    while(i < To.s)
    {
        To.Vec[i]=From[i];
        ++i;
    }
    return true;
}

template<int _size_m>
template <int _size_m1,int _size_m2>
bool TCMatrixFunc<_size_m>::Copy(TVector<_size_m1> &To, TVector<_size_m2> &From, int c_from, int c_to)
{
    int col=c_to-c_from+1;
    if(col>_size_m2)
        return false;
    if(col>_size_m1)
        return false;

    To.Reset();
    To.s = col;
    int i = 0;
    while(i < To.s)
    {
        To.Vec[i]=From.Vec[i+c_from];
        ++i;
    }
    return true;
}

template<int _size_m>
template <int _size_m1,int _size_m2>
bool TCMatrixFunc<_size_m>::Copy(TMatrix<_size_m1> &To, TMatrix<_size_m2> &From, int r_from, int r_to, int c_from, int c_to)
{
    int str_=r_to-r_from+1;
    int col=c_to-c_from+1;
    if(str_>_size_m2 || col>_size_m2)
        return false;
    if(str_>_size_m1 || col>_size_m1)
        return false;

    To.Reset();
    To.s_m = str_;
    To.s_n = col;
    int i = 0, j=0;
    while(i < To.s_m)
    {
        while(j < To.s_n)
        {
            To.M[i][j]=From.M[i+r_from][j+c_from];
            ++j;
        }
        j=0;
        ++i;
    }
    return true;
}


template<int _size_m>
template<int _size_m1, int _size_m2>
bool TCMatrixFunc<_size_m>::Copy(TMatrix<_size_m1> &To, TVector<_size_m2> &From)
{
    bool bRes = true;
    To.Reset();
    if (From.s > 0 && From.s <= _size_m1 && From.s <= _size_m2)
    {
        To.Reset(From.s, 1);
        for (int i=0; i<From.s; i++)
        {
            To.M[i][0] = From.Vec[i];
        }
    }
    else
    {
        bRes = false;
    }
    return bRes;
}


template<int _size_m>
template<int _size_m1, int _size_m2>
bool TCMatrixFunc<_size_m>::Copy(TMatrix<_size_m1> &To, TVector<_size_m2> &From, int r_from, int r_to)
{
    bool bRes = true;
    To.Reset();
    int str_ = r_to-r_from+1;
    if (str_ > 0 && str_ <= _size_m1
            && r_from >= 0 && r_from < _size_m2
            && r_to >= 0 && r_to < _size_m2)
    {
        To.Reset(str_, 1);
        for (int i=0; i<str_; i++)
        {
            To.M[i][0] = From.Vec[r_from+i];
        }
    }
    else
    {
        bRes = false;
    }
    return bRes;
}


template<int _size_m>
template<int _size_m1, int _size_m2>
bool TCMatrixFunc<_size_m>::Copy(TVector<_size_m1> &To, TMatrix<_size_m2> &From)
{
    bool bRes = true;
    To.Reset();
    int i;
    if (From.s_m == 1 && From.s_n > 0 && From.s_n <= _size_m1 && From.s_n <= _size_m2)
    {
        To.Reset(From.s_n);
        for (i=0; i<From.s_n; i++)
        {
            To.Vec[i] = From.M[0][i];
        }
    }
    else
    {
        if (From.s_n == 1 && From.s_m > 0 && From.s_m <= _size_m1 && From.s_m <= _size_m2)
        {
            To.Reset(From.s_m);
            for (i=0; i<From.s_m; i++)
            {
                To.Vec[i] = From.M[i][0];
            }
        }
        else
        {
            bRes = false;
        }
    }
    return bRes;
}


#include "GLMatrix.h"
template<int _size_m>
template<int _size_m1, int _size_m2>
bool TCMatrixFunc<_size_m>::Copy(TVector<_size_m1> &To, TMatrix<_size_m2> &From, int n_from, int n_to)
{
    bool bRes = true;
    To.Reset();
    int i;
    int str_ = n_to - n_from + 1;
    if (0 < str_ && str_ <= _size_m1
            && n_from >= 0 && n_from < _size_m2
            && n_to >= 0 && n_to < _size_m2)
    {
        if (From.s_m == 1 && From.s_n > 0 && From.s_n <= _size_m2)
        {
            To.Reset(From.s_n);
            for (i=0; i<str_; i++)
            {
                To.Vec[i] = From.M[0][i+n_from];
            }
        }
        else
        {
            if (From.s_n == 1 && From.s_m > 0 && From.s_m <= _size_m2)
            {
                To.Reset(From.s_m);
                for (i=0; i<str_; i++)
                {
                    To.Vec[i] = From.M[i+n_from][0];
                }
            }
            else
            {
                bRes = false;
            }
        }
    }
    else
    {
        bRes = false;
    }
    return bRes;
}


template<int _size_m>
template <int row, int col>
bool TCMatrixFunc<_size_m>::Copy(TMatrix<_size_m> &To, const double (&From)[row][col])
{
    if(row > _size_m || col > _size_m)
        return false;

    To.s_m = row;
    To.s_n = col;
    int i = 0, j=0;
    while(i < To.s_m)
    {
        while(j < To.s_n)
        {
            To.M[i][j]=From[i][j];
            ++j;
        }
        j=0;
        ++i;
    }
    return true;
}
template<int _size_m>
template <int row, int col>
bool TCMatrixFunc<_size_m>::Copy(TMatrix<_size_m> &To, double (&From)[row][col])
{
    if(row > _size_m || col > _size_m)
        return false;

    To.s_m = row;
    To.s_n = col;
    int i = 0, j=0;
    while(i < To.s_m)
    {
        while(j < To.s_n)
        {
            To.M[i][j]=From[i][j];
            ++j;
        }
        j=0;
        ++i;
    }
    return true;
}

//-----Correspondence between types "Vector" and "Matrice"-----

template <int _size_m>
bool TCMatrixFunc<_size_m>::Matching(TVector<_size_m> &V1, TMatrix<_size_m> &A)
{
    if (V1.s < 1)
        return false;

    A.s_m = V1.s;
    A.s_n = 1;

    for (int i=0; i<A.s_m; i++)
        A.M[i][0] = V1.Vec[i];

    return true;
}


template <int _size_m>
bool TCMatrixFunc<_size_m>::Matching(TMatrix<_size_m> &M1, TVector<_size_m> &A)
{
    if (M1.s_m < 1 || M1.s_n != 1)
        return false;

    A.s = M1.s_m;

    for (int i=0; i<A.s; i++)
        A.Vec[i] = M1.M[i][0];

    return true;
}

template <int _size_m>
bool TCMatrixFunc<_size_m>::GramMatrix(TVector<_size_m> &V, TMatrix<_size_m> &M)
{
    bool bOK = true;
    if (V.s > 0 && V.s <= _size_m)
    {
        M.s_m = V.s;
        M.s_n = V.s;
        int Step = _size_m;
        int i, j;
        double *Mii = &M.M[0][0];
        double *Mij = Mii;
        double *Mji = Mii;
        double *Vi = &V[0];
        double *Vj = Vi;
        for (i=0; i<V.s; i++, Vi++, Mii+=Step+1)
        {
            *Mii = (*Vi) * (*Vi);
            if (i < V.s - 1)
            {
                Mij = Mii + 1;
                Mji = Mii + Step;
                Vj = Vi+1;
                for (j=i+1; j<V.s; j++, Vj++, Mij++, Mji+=Step)
                {
                    *Mij = (*Vi) * (*Vj);
                    *Mji = *Mij;
                }
            }
        }
    }
    else
    {
        bOK = false;
    }
    return bOK;
}


template <int _size_m>
double TCMatrixFunc<_size_m>::CalcDetSymm3x3(TMatrix<_size_m> &M, int i0)
{
    double Det = 0.;
    if (i0 >= 0 && i0 <= M.s_m-3 && i0 <= M.s_n-3)
    {
        Det = M.M[i0][i0] * M.M[i0+1][i0+1] * M.M[i0+2][i0+2]
                + 2. * M.M[i0][i0+1] * M.M[i0][i0+2] * M.M[i0+1][i0+2]
                -     M.M[i0][i0] * sqr(M.M[i0+1][i0+2])
                - M.M[i0+1][i0+1] * sqr(M.M[i0][i0+2])
                - M.M[i0+2][i0+2] * sqr(M.M[i0][i0+1]);
    }
    return Det;
}


template <int _size_m>
double TCMatrixFunc<_size_m>::CalcDetSymm2x2(TMatrix<_size_m> &M, int i0)
{
    double Det = 0.;
    if (i0 >= 0 && i0 <= M.s_m-2 && i0 <= M.s_n-2)
    {
        Det = M.M[i0][i0] * M.M[i0+1][i0+1] - sqr(M.M[i0][i0+1]);
    }
    return Det;
}


template <int _size_m>
template <int _size_m1, int _size_m2>
bool TCMatrixFunc<_size_m>::Combine2Mod(TVector<_size_m1> &Alpha1, TVector<_size_m2> &Alpha2,
                 TMatrix<_size_m1> &R1, TMatrix<_size_m2> &R2, TVector<2> &ProbAposteriori,
                 TVector<_size_m> &AlphaComb, TMatrix<_size_m> &RComb)
{
    bool bOK = true;
    int ResDim = _size_m;
    int Dim = std::min(_size_m, std::max(_size_m1, _size_m2));
    const int NFilters = 2;
    AlphaComb.Reset(Dim);
    RComb.Reset(Dim,Dim);

    TVector<_size_m> Alpha_extend[NFilters];
    TMatrix<_size_m> R_extend[NFilters];

    bOK = bOK && Copy(Alpha_extend[0], Alpha1);
    bOK = bOK && Copy(Alpha_extend[1], Alpha2);
    bOK = bOK && Copy(R_extend[0], R1);
    bOK = bOK && Copy(R_extend[1], R2);
    for (int i=0; i<NFilters; i++)
    {
        Alpha_extend[i].SetDimension(Dim);
        R_extend[i].SetDimensions(Dim,Dim);
    }

    for (int i=0; i<NFilters; i++)
    {
        AlphaComb = AlphaComb + Alpha_extend[i] * ProbAposteriori[i];
    }

    for (int i=0; i<NFilters; i++)
    {
        TVector<3> Delta = Alpha_extend[i] - AlphaComb;
        TMatrix<3> RDelta;
        bOK = bOK && GramMatrix(Delta, RDelta);
        RComb = RComb + ((R_extend[i]+RDelta)*ProbAposteriori[i]);
    }
    AlphaComb.SetDimension(ResDim);
    RComb.SetDimensions(ResDim, ResDim);
    return bOK;
}


template <int _size_m>
template <int _size_m1, int _size_m2, int _size_m3>
bool TCMatrixFunc<_size_m>::Combine3Mod(TVector<_size_m1> &Alpha1, TVector<_size_m2> &Alpha2, TVector<_size_m3> &Alpha3,
                 TMatrix<_size_m1> &R1, TMatrix<_size_m2> &R2, TMatrix<_size_m3> &R3,
                 TVector<3> &ProbAposteriori,
                 TVector<_size_m> &AlphaComb, TMatrix<_size_m> &RComb)
{
    bool bOK = true;
    int ResDim = _size_m;
    int Dim = std::min(_size_m, std::max(_size_m1, std::max(_size_m2, _size_m3)));
    const int NFilters = 3;
    AlphaComb.Reset(Dim);
    RComb.Reset(Dim,Dim);

    TVector<_size_m> Alpha_extend[NFilters];
    TMatrix<_size_m> R_extend[NFilters];

    bOK = bOK && Copy(Alpha_extend[0], Alpha1);
    bOK = bOK && Copy(Alpha_extend[1], Alpha2);
    bOK = bOK && Copy(Alpha_extend[2], Alpha3);
    bOK = bOK && Copy(R_extend[0], R1);
    bOK = bOK && Copy(R_extend[1], R2);
    bOK = bOK && Copy(R_extend[2], R3);
    for (int i=0; i<NFilters; i++)
    {
        Alpha_extend[i].SetDimension(Dim);
        R_extend[i].SetDimensions(Dim,Dim);
    }

    for (int i=0; i<NFilters; i++)
    {
        AlphaComb = AlphaComb + Alpha_extend[i] * ProbAposteriori[i];
    }

    for (int i=0; i<NFilters; i++)
    {
        TVector<3> Delta = Alpha_extend[i] - AlphaComb;
        TMatrix<3> RDelta;
        bOK = bOK && GramMatrix(Delta, RDelta);
        RComb = RComb + ((R_extend[i]+RDelta)*ProbAposteriori[i]);
    }
    AlphaComb.SetDimension(ResDim);
    RComb.SetDimensions(ResDim, ResDim);
    return bOK;
}


template <int _size_m>
template <int _size_m1, int _size_m2>
bool TCMatrixFunc<_size_m>::TransformTo2D(TMatrix<_size_m1> &To, TMatrix<_size_m2> &From)
{
    short Dim = 2;
    const short Corresp[6] = {0, 2, 3, 5, 6, 8}; //array of correspondence between
                                //elements numbers of 3D and 2D parameters vectors
    bool bOK = true;

    if (_size_m1 == 2 && _size_m2 >= 3 && From.s_m >= 3 && From.s_n >=3)
    {
        Dim = 2;
    }
    else if (_size_m1 == 4 && _size_m2 >= 6 && From.s_m >= 6 && From.s_n >= 6)
    {
        Dim = 4;
    }
    else if (_size_m1 == 6 && _size_m2 >= 9 && From.s_m >= 9 && From.s_n >= 9)
    {
        Dim = 6;
    }
    else if (From.s_m == 3 && From.s_n == 3 && _size_m2 >= 3 && _size_m1 >= 2)
    {
        Dim = 2;
    }
    else if (From.s_m == 6 && From.s_n == 6 && _size_m2 >= 6 && _size_m1 >= 4)
    {
        Dim = 4;
    }
    else if (From.s_m == 9 && From.s_n == 9 && _size_m2 >= 9 && _size_m1 >= 6)
    {
        Dim = 6;
    }
    else
    {
        bOK = false;
    }

    if (bOK)
    {
        To.Reset(Dim, Dim);
        for (short i=0; i<Dim; i++)
        {
            for (short j=0; j<Dim; j++)
            {
                To.M[i][j] = From.M[Corresp[i]][Corresp[j]];
            }
        }
    }
    return bOK;
}


template <int _size_m>
template <int _size_m1, int _size_m2>
bool TCMatrixFunc<_size_m>::TransformTo2D(TVector<_size_m1> &To, TVector<_size_m2> &From)
{
    short Dim = 2;
    const short Corresp[6] = {0, 2, 3, 5, 6, 8}; //array of correspondence between
                                //elements numbers of 3D and 2D parameters vectors
    bool bOK = true;

    if (_size_m1 == 2 && _size_m2 >= 3 && From.s >= 3)
    {
        Dim = 2;
    }
    else if (_size_m1 == 4 && _size_m2 >= 6 && From.s >= 6)
    {
        Dim = 4;
    }
    else if (_size_m1 == 6 && _size_m2 >= 9 && From.s >= 9)
    {
        Dim = 6;
    }
    else if (From.s == 3 && _size_m2 >= 3 && _size_m1 >= 2)
    {
        Dim = 2;
    }
    else if (From.s == 6 && _size_m2 >= 6 && _size_m1 >= 4)
    {
        Dim = 4;
    }
    else if (From.s == 9 && _size_m2 >= 9 && _size_m1 >= 6)
    {
        Dim = 6;
    }
    else
    {
        bOK = false;
    }

    if (bOK)
    {
        To.Reset(Dim);
        for (short i=0; i<Dim; i++)
        {
            To.Vec[i] = From.Vec[Corresp[i]];
        }
    }
    return bOK;
}


template <int _size_m>
bool TCMatrixFunc<_size_m>::ExtrapolateMatrLin(TMatrix<_size_m> &M, double dt, TMatrix<_size_m> &MExtr)
{
    bool bRes = true;
    MExtr.Reset(6,6);

    if (M.s_m == 6 && M.s_n == 6)
    {
        TMatrix<_size_m> ExtrMatr(6,6), ExtrMatrT(6,6); //matrix of extrapolation

        int i;
        for (i=0; i<6; i++)
        {
            ExtrMatr.M[i][i] = 1.;
        }
        for (i=0; i<3; i++)
        {
            ExtrMatr.M[i][i+3] = dt;
        }

        bRes = Transpon(ExtrMatr, ExtrMatrT);
        if (bRes)
        {
            bRes = MatrXMatrXMatr(ExtrMatr, M, ExtrMatrT, MExtr);
        }
    }
    else
    {
        bRes = false;
    }

    return bRes;
}


template <int _size_m>
bool TCMatrixFunc<_size_m>::ExtrapolateMatrQuadr(TMatrix<_size_m> &M, double dt, TMatrix<_size_m> &MExtr)
{
    bool bRes = true;

    if (M.s_m == 9 && M.s_n == 9)
    {
        double dt2_2 = 0.5 * sqr(dt);

        TMatrix<_size_m> ExtrMatr(9,9), ExtrMatrT(9,9); //matrix of extrapolation

        int i;
        for (i=0; i<9; i++)
        {
            ExtrMatr.M[i][i] = 1.;
        }
        for (i=0; i<6; i++)
        {
            ExtrMatr.M[i][i+3] = dt;
        }
        for (i=0; i<3; i++)
        {
            ExtrMatr.M[i][i+6] = dt2_2;
        }
        bRes = Transpon(ExtrMatr, ExtrMatrT); //ExtrMatrT is ExtrMatr transposed
        if (bRes)
        {
            bRes = MatrXMatrXMatr(ExtrMatr, M, ExtrMatrT, MExtr);
        }
    }
    else
    {
        bRes = false;
    }

    return bRes;
}


template <int _size_m>
void GLFileLog::LogMatrix(FILE* _pLogFile, const char* _Name, double &Time, const TMatrix<_size_m> &Matr)
{
    if (_pLogFile != NULL)
    {
        Log(_pLogFile, "%f %s   %d rows  %d columns", Time, _Name, Matr.s_m, Matr.s_n);
        if (Matr.s_m > 0 && Matr.s_n > 0 && Matr.s_m <= _size_m && Matr.s_n <= _size_m)
        {
            for (int i=0; i<Matr.s_m; i++)
            {
                char OutString[GL_MATR::MAX_LEN_STR];
                sprintf(OutString, "   ");
                for (int j=0; j<Matr.s_n; j++)
                {
                    int Len = strlen(OutString);
                    if (Len > GL_MATR::MAX_LEN_STR - GL_MATR::STR_UNDENT)
                    {
                        break;
                    }
                    sprintf(OutString, "%s %f", OutString, Matr.M[i][j]);
                }
                //Log(_pLogFile, "%s", OutString);
                fprintf(_pLogFile, "%9.3lf %s\n", qreal(QDateTime::currentMSecsSinceEpoch()%86400000)/1000., OutString);
            }
        }
        else
        {
            Log(_pLogFile, "Wrong size");
        }
    }
}


template <int _size_m>
void GLFileLog::tLogMatrix(FILE* _pLogFile, const char *_Name, const TMatrix<_size_m> &Matr)
{
    if (_pLogFile != NULL)
    {
        tLog(_pLogFile, " %s   %d rows  %d columns", _Name, Matr.s_m, Matr.s_n);
        if (Matr.s_m > 0 && Matr.s_n > 0 && Matr.s_m <= _size_m && Matr.s_n <= _size_m)
        {
            for (int i=0; i<Matr.s_m; i++)
            {
                char OutString[GL_MATR::MAX_LEN_STR];
                sprintf(OutString, "   ");
                for (int j=0; j<Matr.s_n; j++)
                {
                    int Len = strlen(OutString);
                    if (Len > GL_MATR::MAX_LEN_STR - GL_MATR::STR_UNDENT)
                    {
                        break;
                    }
                    sprintf(OutString, "%s %f", OutString, Matr.M[i][j]);
                }
                //Log(_pLogFile, "%s", OutString);
                fprintf(_pLogFile, "%s\n", OutString);
            }
        }
        else
        {
            Log(_pLogFile, "Wrong size");
        }
    }
}


template <int _size_m>
void GLFileLog::tLogMatrixToLine(FILE* _pLogFile, const char* _Name, const TMatrix<_size_m> &Matr, bool bNoText, bool bPrecise)
{
    if (_pLogFile != NULL)
    {
        char OutString[GL_MATR::MAX_LEN_STR];
        if (bNoText)
        {
            sprintf(OutString, "%s %d %d", _Name, Matr.s_m, Matr.s_n);
        }
        else //text may be presented
        {
            sprintf(OutString, "%s %d rows %d columns", _Name, Matr.s_m, Matr.s_n);
        }
        if (Matr.s_m > 0 && Matr.s_n > 0 && Matr.s_m <= _size_m && Matr.s_n <= _size_m)
        {
            int dim = std::min(Matr.s_m, Matr.s_n);
            int i, j;
            for (i=0; i<dim; i++)
            {
                int Len = strlen(OutString);
                if (Len > GL_MATR::MAX_LEN_STR - GL_MATR::STR_UNDENT)
                {
                    break;
                }
                if (bPrecise)
                {
                    sprintf(OutString, "%s %.12f", OutString, Matr.M[i][i]);
                }
                else
                {
                    sprintf(OutString, "%s %f", OutString, Matr.M[i][i]);
                }
            }

            for (i=0; i<dim-1; i++)
            {
                for (j=i+1; j<dim; j++)
                {
                    int Len = strlen(OutString);
                    if (Len > GL_MATR::MAX_LEN_STR - GL_MATR::STR_UNDENT)
                    {
                        break;
                    }
                    if (bPrecise)
                    {
                        sprintf(OutString, "%s %.12f", OutString, Matr.M[i][j]);
                    }
                    else
                    {
                        sprintf(OutString, "%s %f", OutString, Matr.M[i][j]);
                    }
                }
            }
            //tLog(_pLogFile, "%s", OutString);
            fprintf(_pLogFile, "%9.3lf %s\n", qreal(QDateTime::currentMSecsSinceEpoch()%86400000)/1000., OutString);
        }
    }
}


template <int _size_m>
void GLFileLog::OutMatrixToString(const char* _Name, const TMatrix<_size_m> &Matr, const int SizeOut, char*& _OutString, bool bPrecise)
{
    char CurString[GL_MATR::MAX_LEN_STR];
    sprintf(CurString, "%s %d %d", _Name, Matr.s_m, Matr.s_n);

    if (SizeOut > 0 && SizeOut <= _size_m)
    {
        int dim = SizeOut;
        int i, j;
        for (i=0; i<dim; i++)
        {
            int Len = strlen(CurString);
            if (Len > GL_MATR::MAX_LEN_STR - GL_MATR::STR_UNDENT)
            {
                break;
            }
            if (bPrecise)
            {
                sprintf(CurString, "%s %.12f", CurString, Matr.M[i][i]);
            }
            else
            {
                sprintf(CurString, "%s %f", CurString, Matr.M[i][i]);
            }
        }

        for (i=0; i<dim-1; i++)
        {
            for (j=i+1; j<dim; j++)
            {
                int Len = strlen(CurString);
                if (Len > GL_MATR::MAX_LEN_STR - GL_MATR::STR_UNDENT)
                {
                    break;
                }
                if (bPrecise)
                {
                    sprintf(CurString, "%s %.12f", CurString, Matr.M[i][j]);
                }
                else
                {
                    sprintf(CurString, "%s %f", CurString, Matr.M[i][j]);
                }
            }
        }
    }
    sprintf(_OutString, "%s", CurString);
}


template <int _size_m>
void GLFileLog::LogVector(FILE* _pLogFile, const char* _Name, double &Time, const TVector<_size_m> &Vec)
{
    if (_pLogFile != NULL)
    {
        Log(_pLogFile, "%f %s   %d elements", Time, _Name, Vec.s);
        if (Vec.s > 0 && Vec.s <= _size_m)
        {
            char OutString[GL_MATR::MAX_LEN_STR];
            sprintf(OutString, "   ");
            for (int i=0; i<Vec.s; i++)
            {
                int Len = strlen(OutString);
                if (Len > GL_MATR::MAX_LEN_STR - GL_MATR::STR_UNDENT)
                {
                    break;
                }
                sprintf(OutString, "%s %f", OutString, Vec.Vec[i]);
            }
            //Log(_pLogFile, "%s", OutString);
            fprintf(_pLogFile, "%9.3lf %s\n", qreal(QDateTime::currentMSecsSinceEpoch()%86400000)/1000., OutString);
        }
        else
        {
            Log(_pLogFile, "Wrong size");
        }
    }
}


template <int _size_m>
void GLFileLog::tLogVector(FILE* _pLogFile, const char* _Name, const TVector<_size_m> &Vec)
{
    if (_pLogFile != NULL)
    {
        tLog(_pLogFile, " %s   %d elements", _Name, Vec.s);
        if (Vec.s > 0 && Vec.s <= _size_m)
        {
            char OutString[GL_MATR::MAX_LEN_STR];
            sprintf(OutString, "   ");
            for (int i=0; i<Vec.s; i++)
            {
                int Len = strlen(OutString);
                if (Len > GL_MATR::MAX_LEN_STR - GL_MATR::STR_UNDENT)
                {
                    break;
                }
                sprintf(OutString, "%s %f", OutString, Vec.Vec[i]);
            }
            //Log(_pLogFile, "%s", OutString);
            fprintf(_pLogFile, "%9.3lf %s\n", qreal(QDateTime::currentMSecsSinceEpoch()%86400000)/1000., OutString);
        }
        else
        {
            Log(_pLogFile, "Wrong size");
        }
    }
}


template <int _size_m>
void GLFileLog::tLogVectorToLine(FILE* _pLogFile, const char* _Name, const TVector<_size_m> &Vec)
{
    if (_pLogFile != NULL)
    {
        char OutString[GL_MATR::MAX_LEN_STR];
        sprintf(OutString, " %s   %d elements", _Name, Vec.s);
        if (Vec.s > 0 && Vec.s <= _size_m)
        {
            sprintf(OutString, "%s  | ", OutString);
            for (int i=0; i<Vec.s; i++)
            {
                int Len = strlen(OutString);
                if (Len > GL_MATR::MAX_LEN_STR - GL_MATR::STR_UNDENT)
                {
                    break;
                }
                sprintf(OutString, "%s %f", OutString, Vec.Vec[i]);
            }
        }
        else
        {
            if (Vec.s == 0)
            {
                sprintf(OutString, "%s %s", OutString, "Zero size");
            }
            else
            {
                sprintf(OutString, "%s %s", OutString, "Zero size");
            }
        }
        //tLog(_pLogFile, "%s", OutString);
        fprintf(_pLogFile, "%9.3lf %s\n", qreal(QDateTime::currentMSecsSinceEpoch()%86400000)/1000., OutString);
    }
}


template <int _size_m>
void GLFileLog::OutVectorToString(const char* _Name, const TVector<_size_m> &Vec, const int SizeOut, char*& _OutString)
{
    char CurString[GL_MATR::MAX_LEN_STR];
    sprintf(CurString, " %s %d", _Name, Vec.s);
    if (SizeOut > 0 && SizeOut <= _size_m)
    {
        for (int i=0; i<SizeOut; i++)
        {
            int Len = strlen(CurString);
            if (Len > GL_MATR::MAX_LEN_STR - GL_MATR::STR_UNDENT)
            {
                break;
            }
            sprintf(CurString, "%s %f", CurString, Vec.Vec[i]);
        }
    }
    sprintf(_OutString, "%s", CurString);
}
