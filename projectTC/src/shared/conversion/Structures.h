// PACKAGE		: Conversion
// FILE      : Structures.h
//
// AUTHOR	 : Marina Mukhortova, 2003
// DESCRIPTION		:Header file for Square_Matrix class and for inclusion of different structures


#ifndef __Structures_H__
#define __Structures_H__

#include <cstring>

/*// Structure of coordinates in GEO (geodesic) system 
struct SGEO
{
    double m_dLatitude{0.};  // Latitude
    double m_dLongitude{0.}; // Longitude
    double m_dAltitude{0.};  // Altitude
};       


// Structure of coordinates in Gauss-Kruger system 
struct SGAUS
{
    double m_dXg{0.};   // Xg - gauss coordinate X
    double m_dYgz{0.};  // Ygz - gauss coordinate Y in zone
    double m_dH{0.};    // H - altitude
    int    m_iNz{0.};   // iNz - number of zone (from 1 to 60)
};     


// Structure of coordinates in WGS (geocentric) system
struct SWGS
{
    double m_dX{0.};   // Coordinate X
    double m_dY{0.};   // Coordinate Y
    double m_dZ{0.};   // Coordinate Z
};


// Structure of coordinates in Topocentric system
struct STOPO
{
    double m_dXt{0.};  // Coordinate Xt
    double m_dYt{0.};  // Coordinate Yt
    double m_dZt{0.};  // Coordinate Zt
};


// Structure of coordinates in Start system 
struct SSTART
{
    double m_dXs{0.};        // Coordinate Xs
    double m_dYs{0.};        // Coordinate Ys
    double m_dZs{0.};        // Coordinate Zs
    double m_dAzimuth{0.};   // Azimuth
};


// Structures of coordinates in Parametric system 
struct SPARAM
{
    double m_dS{0.};   // Course distance of the target
    double m_dP{0.};   // Course parameter of the target
    double m_dQ{0.};   // Course angle of the target
    double m_dT{0.};   // Flying time of the object up to intersection with OP-axis
    double m_dH{0.};   // Altitude of the target
};


// Structure of coordinates in Spherical system 
struct SSFER
{
    double m_dR{0.};   // Slant range of the target
    double m_dE{0.};   // Elevation angle
    double m_dB{0.};   // Azimuth
};


// Structure of speed of the target in WGS system of coordinates
struct SSPEED_WGS
{
    double m_dVX{0.};   // X - constituent of the velocity
    double m_dVY{0.};   // Y - constituent of the velocity
    double m_dVZ{0.};   // Z - constituent of the velocity
};


// Structure of speed of the target in Topocentric system
struct SSPEED_TOPO
{
    double m_dVXt{0.};  // Xt - constituent of the velocity
    double m_dVYt{0.};  // Yt - constituent of the velocity
    double m_dVZt{0.};  // Zt - constituent of the velocity
};


// Structure of speed of the target in Spherical system 
struct SSPEED_SFER
{
    double m_dVR{0.};   // R - constituent of the velocity (with regard to slant range)
    double m_dVE{0.};   // E - constituent of the velocity (with regard to Elevation angle)
    double m_dVB{0.};   // B - constituent of the velocity (with regard to Azimuth)
};


// Structure of acceleration of the target in WGS system of coordinates
struct SACCELERATION_WGS
{
    double m_dAX{0.};   // X - constituent of the acceleration
    double m_dAY{0.};   // Y - constituent of the acceleration
    double m_dAZ{0.};   // Z - constituent of the acceleration
};


// Structure of acceleration of the target in Topocentric system
struct SACCELERATION_TOPO
{
    double m_dAXt{0.};  // Xt - constituent of the acceleration
    double m_dAYt{0.};  // Yt - constituent of the acceleration
    double m_dAZt{0.};  // Zt - constituent of the acceleration
};


// Square-law deviations in Topocentric system of coordinates
//typedef struct {double Cx,Cz,Cvx,Cvz;} SIGMAT;

// Square-law deviations in Parametric system of coordinates
//typedef struct {double Cs,Cp,Cq;} SIGMAP;
*/

// PACKAGE		: Conversion
// STRUCT    : SKoefMatrRotCos
//
// DESCRIPTION  : Structure of the matrix of rotational cosines
//
struct SKoefMatrRotCos
{
    double m_dKx1{0.};  // Coefficient Kx1
    double m_dKx2{0.};  // Coefficient Kx2
    double m_dKx3{0.};  // Coefficient Kx3
    double m_dKy1{0.};  // Coefficient Ky1
    double m_dKy2{0.};  // Coefficient Ky2
    double m_dKy3{0.};  // Coefficient Ky3
    double m_dKz1{0.};  // Coefficient Kz1
    double m_dKz2{0.};  // Coefficient Kz2
    double m_dKz3{0.};  // Coefficient Kz3
};

// PACKAGE		: Conversion
// STRUCT       : SKoef_TurnTOPOtoGEOCENTRIC
//
// DESCRIPTION  : Structure of the matrix of transfer coefficients from TOPO to WGS
//
//
struct SKoef_TurnTOPOtoGEOCENTRIC
{
    double m_dKx0{0.};  // Coefficient Kx0
    double m_dKy0{0.};  // Coefficient Ky0
    double m_dKz0{0.};  // Coefficient Kz0
};

// PACKAGE		: Conversion
// STRUCT       : SKoef_TurnGEOCENTRICtoTOPO
//
// DESCRIPTION  : Structure of the matrix of transfer of coefficients from WGS to TOPO
//
//
struct SKoef_TurnGEOCENTRICtoTOPO
{
    double m_dKx0{0.};  // Coefficient Kx0
    double m_dKy0{0.};  // Coefficient Ky0
    double m_dKz0{0.};  // Coefficient Kz0
};

// PACKAGE		: Conversion
// STRUCT       : SKoefTransposeMatr
//
// DESCRIPTION  : Structure of the transpose of a matrix of rotational cosines
//
//
struct SKoefTransposeMatr
{
    double m_dKx1{0.};  // Coefficient Kx1
    double m_dKx2{0.};  // Coefficient Kx2
    double m_dKx3{0.};  // Coefficient Kx3
    double m_dKy1{0.};  // Coefficient Ky1
    double m_dKy2{0.};  // Coefficient Ky2
    double m_dKy3{0.};  // Coefficient Ky3
    double m_dKz1{0.};  // Coefficient Kz1
    double m_dKz2{0.};  // Coefficient Kz2
    double m_dKz3{0.};  // Coefficient Kz3
};

// PACKAGE		: Conversion
// STRUCT       : SKoefRecal
//
// DESCRIPTION  : Structure of factors of recalculation from Topo1 to Topo2
//
//
struct SKoefRecal
{
    double m_dKx0{0.};  // Coefficient Kx0
    double m_dKy0{0.};  // Coefficient Ky0
    double m_dKz0{0.};  // Coefficient Kz0
    double m_dKx1{0.};  // Coefficient Kx1
    double m_dKx2{0.};  // Coefficient Kx2
    double m_dKx3{0.};  // Coefficient Kx3
    double m_dKy1{0.};  // Coefficient Ky1
    double m_dKy2{0.};  // Coefficient Ky2
    double m_dKy3{0.};  // Coefficient Ky3
    double m_dKz1{0.};  // Coefficient Kz1
    double m_dKz2{0.};  // Coefficient Kz2
    double m_dKz3{0.};  // Coefficient Kz3
}; 

// PACKAGE		: Conversion
// STRUCT       : SKoefRecalTopo_AxisMeridian
//
// DESCRIPTION  : Structure of factors of recalculation from Topo to TopoAxisMeridian
//
//
struct SKoefRecalTopo_AxisMeridian
{
    double m_dK11_T_AM{0.};  // Coefficients K11
    double m_dK12_T_AM{0.};  // Coefficients K12
    double m_dK21_T_AM{0.};  // Coefficients K21
    double m_dK22_T_AM{0.};  // Coefficients K22

};

// PACKAGE		: SKoefRecalAxisMeridian_Topo
// STRUCT       : SKoefMatrRotCos
//
// DESCRIPTION  : Structure of factors of recalculation from TopoAxisMeridian to Topo
//
//
struct SKoefRecalAxisMeridian_Topo
{
    double m_dK11_AM_T{0.};  // Coefficients K11
    double m_dK12_AM_T{0.};  // Coefficients K12
    double m_dK21_AM_T{0.};  // Coefficients K21
    double m_dK22_AM_T{0.};  // Coefficients K22
};


// PACKAGE		: Conversion
// STRUCT       : SLevelCoef_A
//
// DESCRIPTION  : Struct of levelling coefficients a
//
//
//
struct SLevelCoef_A
{
    // Coefficient A11
    //
    double m_dA11{0.};

    // Coefficient A12
    //
    double m_dA12{0.};

    // Coeffisient A13
    //
    double m_dA13{0.};

    // Coefficient A21
    //
    double m_dA21{0.};

    // Coefficient A22
    //
    double m_dA22{0.};

    // Coefficient A23
    //
    double m_dA23{0.};

    // Coefficient A31
    //
    double m_dA31{0.};

    // Coefficient A32
    //
    double m_dA32{0.};

    // Coefficient A33
    //
    double m_dA33{0.};

};


// PACKAGE		: Conversion
// STRUCT       : SLevelCoef_B
//
// DESCRIPTION  : Struct of levelling coefficients b
//
//
//
struct SLevelCoef_B
{
    // Coefficient DeltaX
    //
    double m_dDeltaX{0.};

    // Coefficient DeltaY
    //
    double m_dDeltaY{0.};

    // Coefficient DeltaZ
    //
    double m_dDeltaZ{0.};

};


// PACKAGE		: Conversion
// CLASS	    : Square_Matrix
//
// DESCRIPTION  : Class for square matrices. Implements transpose and matrix multiplication
//

template <const int _size_m> class Square_Matrix
{
public:
    Square_Matrix(int _str, int _col)
    {
        s_m = _str;
        s_n = _col;
        for(int i = 0; i < s_m; i++)
            for (int j = 0; j < s_n; j++)
                M[i][j] = 0;
    }
    Square_Matrix()
    {
        Reset(_size_m, _size_m);
    }
    void Reset()
    {
        memset(this, 0, sizeof(Square_Matrix<_size_m>));
    }
    void Reset(int _str, int _col)
    {
        memset(this, 0, sizeof(Square_Matrix<_size_m>));
        s_m = _str;
        s_n = _col;
        for(int i = 0; i < s_m; i++)
            for (int j = 0; j < s_n; j++)
                M[i][j] = 0;
    }
    Square_Matrix operator = (const Square_Matrix<_size_m>& matr)
    {
        if (s_m == 0 && s_n == 0) //Add TaniaP
        {
            s_m = matr.s_m;
            s_n = matr.s_n;
        }

        if (s_m == matr.s_m && s_n == matr.s_n)
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
    void transp(Square_Matrix<_size_m> *matr){
        if (matr->s_m == 0 && matr->s_n == 0) //Add TaniaP
        {
            matr->s_m = s_m;
            matr->s_n = s_n;
        }
        if (s_m == matr->s_m && s_n == matr->s_n)
        {
            for (int i = 0; i < s_m; i++)
            {
                for (int j = 0; j < s_n; j++)
                {
                    matr->M[i][j] = M[j][i];
                }
            }

        }
    }
    //MatrXMatr(const TMatrix<_size_m> &M1, const TMatrix<_size_m> &M2, TMatrix<_size_m> &A_out)
    bool MatrXMatr(Square_Matrix<_size_m> *M2,Square_Matrix<_size_m> *A_out){
        if (
                (s_n != M2->s_m)	||
                (s_m > _size_m)	||
                (s_n > _size_m)	||
                (M2->s_m > _size_m)	||
                (M2->s_n > _size_m))
            return false;
        int i,j,k;
        for(k = 0; k < M2->s_n; k++)
        {
            for(i = 0; i < s_m; i++)
            {
                for(j = 0; j < s_n; j++)
                {
                    A_out->M[i][k] += M[i][j] * M2->M[j][k];
                }
            }
        }
        A_out->s_m = s_m;
        A_out->s_n = M2->s_n;
        return true;

    }
    bool MatrXMatrXMatr(Square_Matrix<_size_m> *M2, Square_Matrix<_size_m> *M3, Square_Matrix<_size_m> *A_out)
    {
        Square_Matrix<_size_m> Mcurr;

        bool ItsOk;

        ItsOk = MatrXMatr(M2, &Mcurr); //Mcurr = M1 * M2
        if (!ItsOk)
            return false;

        ItsOk = Mcurr.MatrXMatr(M3, A_out); //A_out = M1 * M2 * M3
        return ItsOk;
    }

    double  M[_size_m][_size_m];		// Matrix of numbers
    int     s_m{_size_m};               // Number of rows
    int     s_n{_size_m};               // Number of columns
};

#endif
