/*!\file GLSector.cpp
\brief Implementation file for GLSector class.
*/

#include "conversion/Spherical_Topo.h"

#include "GLSector.h"
#include "Constants.h"

#include <cmath>

#undef _OSF_SOURCE
#ifndef WIN32
    #include <cstdlib>
#endif

/* ikemy 2010-11-09 */
/* BEGIN */

// ========================================================
// Quadric equation solution.
quint32 GLMath::QE( const double A,
                    const double B,
                    const double C,
                    double &Z1,
                    double &Z2 )
{
    double D = ( B * B ) - ( 4 * A * C );
    if ( D < 0 )
    {
        return 0;
    }
    if ( D == 0 )
    {
        Z1 = -B / ( 2.0 * A );
        return 1;
    }
    if ( D > 0 )
    {
        Z1 = ( -B + sqrt( D ) ) / ( 2.0 * A );
        Z2 = ( -B - sqrt( D ) ) / ( 2.0 * A );
        return 2;
    }
    return 0;
}

// ========================================================
// Get equation of straight from 2 points.
bool GLMath::SE_From_2_Pts( const GLPointDouble3D Pt1,
                            const GLPointDouble3D Pt2,
                            double &K,
                            double &B )
{
    if ( Pt1.z == Pt2.z &&
         Pt1.x == Pt2.x )
    {
        return false;
    }
    double X1 = Pt1.x;
    double Z1 = Pt1.z;
    double X2 = Pt2.x;
    double Z2 = Pt2.z;
    K = ( X1 - X2 ) / ( Z1 - Z2 );
    B = X1 - ( ( Z1 * ( X1 - X2 ) ) / ( Z1 - Z2 ) );
    return true;
}

// ========================================================
// Get length of segment from 2 points.
double GLMath::Segm_Length_2D( const GLPointDouble3D Pt1,
                                 const GLPointDouble3D Pt2 )
{
    double A =  pow( ( Pt2.z - Pt1.z ),
                       2.0 ) +
                  pow( ( Pt2.x - Pt1.x ),
                       2.0 );
    return ( sqrt( A ) );
}

// ========================================================
// Get length of segment from 2 points.
double GLMath::Segm_Length_3D( const GLPointDouble3D Pt1,
                                 const GLPointDouble3D Pt2 )
{
    double A =  pow( ( Pt2.x - Pt1.x ),
                       2.0 ) +
                  pow( ( Pt2.y - Pt1.y ),
                       2.0 ) +
                  pow( ( Pt2.z - Pt1.z ),
                       2.0 );
    return ( sqrt( A ) );
}

// ========================================================
// Straight and circle crossing verifying.
quint32 GLMath::Str_Cross_Circle( double K,
                                  double B,
                                  double R,
                                  GLPointDouble3D& Pt1,
                                  GLPointDouble3D& Pt2 )
{
    const double a = ( K * K ) + 1;
    const double b = 2.0 * K * B;
    const double c = ( B * B ) - ( R * R );
    quint32 Res = GLMath::QE( a,
                              b,
                              c,
                              Pt1.z,
                              Pt2.z );
    if ( Res == 0 ) { return 0; }
    if ( Res == 1 ) { Pt1.x = ( K * Pt1.z ) + B; return 1; }
    if ( Res == 2 ) { Pt1.x = ( K * Pt1.z ) + B; Pt2.x = ( K * Pt2.z ) + B; return 2; }
    return 0;
}

// ========================================================
bool GLMath::Points_Cmp_2D( const GLPointDouble3D& Pt1,
                            const GLPointDouble3D& Pt2 )
{
    double Delta = 0.1e-9;
    if ( fabs( Pt1.z - Pt2.z ) < Delta )
    {
        if ( fabs( Pt1.x - Pt2.x ) < Delta )
        {
            return true;
        }
    }
    return false;
}

// ========================================================
bool GLMath::Point_In_Segment( const GLPointDouble3D& SPt1,
                               const GLPointDouble3D& SPt2,
                               const GLPointDouble3D& TGT_Pt )
{
    double Min_z, Max_z, Min_x, Max_x;
    if ( SPt1.z >= SPt2.z ) { Min_z = SPt2.z; Max_z = SPt1.z; }
    else { Min_z = SPt1.z; Max_z = SPt2.z; }
    if ( SPt1.x >= SPt2.x ) { Min_x = SPt2.x; Max_x = SPt1.x; }
    else { Min_x = SPt1.x; Max_x = SPt2.x; }
    if ( TGT_Pt.z >= Min_z &&
         TGT_Pt.z <= Max_z )
    {
        if ( TGT_Pt.x >= Min_x &&
             TGT_Pt.x <= Max_x )
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    return false;
}

// ========================================================
// Get tangent points to circle from point outside of this circle
quint8 GLMath::Get_Tangent_Pts( const GLPointDouble3D& Pt0,
                                double R,
                                GLPointDouble3D& Pt1,
                                GLPointDouble3D& Pt2 )
{
    double Zt, Xt, Dst, Lt, L, N, A, B, Alpha, Beta, Gamma, D, D1;

    Zt = Pt0.z;
    Xt = Pt0.x;
    Dst = sqrt( pow( Zt, 2.0 ) + pow( Xt, 2.0 ) );
    Lt = pow( Dst, 2.0 ) - pow( R, 2.0 );
    if ( Lt < 0 )
    {
        return 0;
    }
    L = sqrt( Lt );
    N = pow( Zt, 2.0 ) + pow( Xt, 2.0 ) - pow( L, 2.0 );
    A = -Zt / Xt;
    B = ( pow( R, 2.0 ) + N ) / ( 2.0 * Xt );
    Alpha = 1 + pow( A, 2.0 );
    Beta = 2.0 * A * B;
    Gamma = pow( B, 2.0 ) - pow( R, 2.0 );
    D1 = pow( Beta, 2.0 ) - 4.0 * Alpha * Gamma;
    if ( D1 < 0 )
    {
        return 0;
    }
    D = sqrt( D1 );
    Pt1.z = ( -Beta + D ) / ( 2.0 * Alpha );
    Pt1.x = A * Pt1.z + B;
    Pt2.z = ( -Beta - D ) / ( 2.0 * Alpha );
    Pt2.x = A * Pt2.z + B;

    if ( GLMath::Points_Cmp_2D( Pt1,
                                Pt2 ) )
    {
        return 1;
    }

    return 2;
}

// ========================================================
// Get integer part (floor) of fractional digit.
double GLMath::Get_Int_Part( double D )
{
    return floor( D );
}

// ========================================================
// Get fractional part of fractional digit.
double GLMath::Get_Fract_Part( double D )
{
    return ( D - floor( D ) );
}

// ========================================================
double GLMath::Course_Angle( const GLPoint3D& TGT_Coord,
                               const GLVelocity3D& TGT_Vel,
                               const GLPoint3D& L_Coord )
{
    const double _PI = 3.1415926535897932384626433832795;
    double J = ( double )( TGT_Vel.vx * ( L_Coord.x - TGT_Coord.x ) + TGT_Vel.vz * ( L_Coord.z - TGT_Coord.z ) );
    double V = ( double )GLabsFlat( TGT_Vel );
    double L = ( double )( 1.0 * ( L_Coord.x - TGT_Coord.x ) * ( L_Coord.x - TGT_Coord.x ) + 1.0 * ( L_Coord.z - TGT_Coord.z ) * ( L_Coord.z - TGT_Coord.z ) );
    double I = V * sqrt( L );
    double D = ( I > 0.5 ) ? ( J / I ) : 0.0;

    if ( abs( D ) > 1.0 )
    {
        D = 1.0 * D / abs( D );
    }

    return ( 180.0 / _PI ) * acos( D );
}

// ========================================================
GLPointDouble3D GLMath::Extrapolate( const GLPointDouble3D& TGT_Coord,
                                     const GLVelocity3D& TGT_Vel,
                                     double TSec )
{
    GLPointDouble3D Ex_Coord;
    Ex_Coord.x = TGT_Coord.x + ( double )( TGT_Vel.vx ) * TSec;
    Ex_Coord.y = TGT_Coord.y + ( double )( TGT_Vel.vy ) * TSec;
    Ex_Coord.z = TGT_Coord.z + ( double )( TGT_Vel.vz ) * TSec;
    return Ex_Coord;
}

// ========================================================
quint32 GLMath::Get_Cross_Pts_Str_Circle( const GLPoint3D& Pt_AO,
                                          const GLVelocity3D& Vel_AO,
                                          double R,
                                          GLPointDouble3D& Pt1,
                                          GLPointDouble3D& Pt2 )
{
    GLPointDouble3D Curr_Coord;
    Curr_Coord.copy( Pt_AO );
    // СЌРєСЃС‚СЂР°РїРѕР»РёСЂСѓРµРј РґР»СЏ РїРѕР»СѓС‡РµРЅРёСЏ РІС‚РѕСЂРѕР№ С‚РѕС‡РєРё
    GLPointDouble3D Ex_Coord = GLMath::Extrapolate( Curr_Coord,
                                                    Vel_AO,
                                                    1.0 );
    double K, B;
    // РїРѕР»СѓС‡Р°РµРј СѓСЂР°РІРЅРµРЅРёРµ РїСЂСЏРјРѕР№ РїРѕ РґРІСѓРј С‚РѕС‡РєР°Рј
    if ( !GLMath::SE_From_2_Pts( Curr_Coord,
                                 Ex_Coord,
                                 K,
                                 B ) )
    {
        return 0;
    }

    // РёС‰РµРј С‚РѕС‡РєРё РїРµСЂРµСЃРµС‡РµРЅРёСЏ РїСЂСЏРјРѕР№ Рё РѕРєСЂСѓР¶РЅРѕСЃС‚Рё
    return GLMath::Str_Cross_Circle( K,
                                     B,
                                     R,
                                     Pt1,
                                     Pt2 );
}

// ========================================================
double GLMath::Round( double Dbl_Val,
                        quint32 Prec )
{
    double DD = Dbl_Val * pow( 10.0, ( double )Prec );
    double DD1 = floor( DD );

    double DD2 = DD - DD1;
    if ( DD2 > 0.5 )
    {
        DD1 += 1.0;
    }
    qint32 I = ( qint32 )DD1;
    DD = ( double )I / pow( 10.0, ( double )Prec );
    return DD;
}

// =======================================================
bool GLMath::In_Target( const GLPointFloat3D &TGT_Coord,
                        const GLVelocityFloat3D &TGT_Vel,
                        const GLPointFloat3D &Launcher_Coord )
{
    return ( ( ( TGT_Coord.x - Launcher_Coord.x) * TGT_Vel.vx + ( TGT_Coord.z - Launcher_Coord.z ) * TGT_Vel.vz ) < 0 ) ? true : false;
}

// ========================================================
// GLSegment constructor
GLSegment::GLSegment( GLPointDouble3D Pt1,
                      GLPointDouble3D Pt2 ):
    m_Pt1( Pt1 ),
    m_Pt2( Pt2 )
{
    GLMath::SE_From_2_Pts( m_Pt1,
                           m_Pt2,
                           m_K,
                           m_B );
    m_Length = GLMath::Segm_Length_2D( m_Pt1,
                                       m_Pt2 );
}

// ========================================================
// Straight and segment crossing verifying
bool GLSegment::Str_Cross_Segment( double K,
                                   double B,
                                   GLPointDouble3D& Cr_Pt )
{
    if ( K == m_K )
    {
        return false;
    }
    // Get crossing point
    Cr_Pt.z = ( m_B - B ) / ( K - m_K );
    Cr_Pt.x = K * Cr_Pt.z + B;

    double L1 = GLMath::Segm_Length_2D( m_Pt1,
                                          Cr_Pt );
    double L2 = GLMath::Segm_Length_2D( m_Pt2,
                                          Cr_Pt );
    double Res = abs( m_Length - ( L1 + L2 ) );
    if ( Res >= 0 &&
         Res < MSR_ERR )
    {
        return true;
    }
    return false;
}


// ========================================================
GLWrkPolygon::GLWrkPolygon( std::vector< GLSegment > &Segm_List )
{
    m_Segm_List = Segm_List;
}

// ========================================================
GLWrkPolygon::GLWrkPolygon( GLPolygon &PGN )
{
    m_Segm_List.clear();
    quint32 Sz = PGN.size();
    if ( Sz >= 3 )
    {
        for ( quint32 i = 1; i < Sz; i++ )
        {
            GLPointDouble3D Pt1( 0, 0, 0 ); GLPointDouble2D _P1 = PGN[ i - 1 ]; Pt1.z = _P1.x; Pt1.x = _P1.y;
            GLPointDouble3D Pt2( 0, 0, 0 ); GLPointDouble2D _P2 = PGN[ i ];     Pt2.z = _P2.x; Pt2.x = _P2.y;

            GLSegment SGM( Pt1, Pt2 ); m_Segm_List.push_back( SGM );
        }
        GLPointDouble3D Pt1( 0, 0, 0 ); GLPointDouble2D _P1 = PGN[ Sz - 1 ];    Pt1.z = _P1.x; Pt1.x = _P1.y;
        GLPointDouble3D Pt2( 0, 0, 0 ); GLPointDouble2D _P2 = PGN[ 0 ];         Pt2.z = _P2.x; Pt2.x = _P2.y;
        GLSegment SGM( Pt1, Pt2 ); m_Segm_List.push_back( SGM );
    }
}

// ========================================================
bool GLWrkPolygon::Str_Cross_Polygon( double K,
                                      double B )
{
    bool Res = false;
    for ( auto Val: m_Segm_List )
    {
        GLSegment &Segm = Val;
        GLPointDouble3D Cr_Pt;

        if ( Segm.Str_Cross_Segment( K,
                                     B,
                                     Cr_Pt ) )
        {
            Res = true;
            return Res;
        }
    }

    return Res;
}

// ========================================================
quint32 GLWrkPolygon::Str_Cross_Polygon( double K,
                                         double B,
                                         std::vector< GLPointDouble3D > &Cr_Pts_Vect )
{
    quint32 Cr_Pts = 0;
    Cr_Pts_Vect.clear();
    for ( auto Val: m_Segm_List )
    {
        GLSegment &Segm = Val;
        GLPointDouble3D Cr_Pt;

        if ( Segm.Str_Cross_Segment( K,
                                     B,
                                     Cr_Pt ) )
        {
            Cr_Pts++;
            Cr_Pts_Vect.push_back( Cr_Pt );
        }
    }
    return Cr_Pts;
}

// ========================================================
bool GLWrkPolygon::Is_Valid()
{
    quint32 Sz = m_Segm_List.size();
    return ( Sz >= 3 ) ? true : false;
}

// ========================================================
// GLWrkTrap constructor
GLWrkTrap::GLWrkTrap( GLPointDouble3D Pt_A,
                      GLPointDouble3D Pt_B,
                      GLPointDouble3D Pt_C,
                      GLPointDouble3D Pt_D ):
    m_Trap_Segm_AB( Pt_A,
                    Pt_B ),
    m_Trap_Segm_BC( Pt_B,
                    Pt_C ),
    m_Trap_Segm_CD( Pt_C,
                    Pt_D ),
    m_Trap_Segm_DA( Pt_D,
                    Pt_A )
{
    //
}

// ========================================================
// Check condition "straight and trapezium segments have crossing point(-s)"
bool GLWrkTrap::Str_Cross_Trap( double K,
                                double B )
{
    GLPointDouble3D Cr_Pt;
    if (
            m_Trap_Segm_AB.Str_Cross_Segment( K,
                                              B,
                                              Cr_Pt ) ||
            m_Trap_Segm_BC.Str_Cross_Segment( K,
                                              B,
                                              Cr_Pt ) ||
            m_Trap_Segm_CD.Str_Cross_Segment( K,
                                              B,
                                              Cr_Pt ) ||
            m_Trap_Segm_DA.Str_Cross_Segment( K,
                                              B,
                                              Cr_Pt )
        )
    {
        return true;
    }
    else
    {
        return false;
    }
}

// ========================================================
// Check condition "straight and trapezium segments have crossing point(-s)".
quint32 GLWrkTrap::Str_Cross_Trap( double K,
                                   double B,
                                   GLPointDouble3D& Cr_Pt1,
                                   GLPointDouble3D& Cr_Pt2 )
{
    GLPointDouble3D Cr_Pt;
    quint32 Cr_Pts = 0;
    if ( m_Trap_Segm_AB.Str_Cross_Segment( K,
                                           B,
                                           Cr_Pt ) )
    {
        Cr_Pts++;
        Cr_Pt1 = Cr_Pt;
    }
    if ( m_Trap_Segm_CD.Str_Cross_Segment( K,
                                           B,
                                           Cr_Pt ) )
    {
        if ( Cr_Pts == 0 )
        {
            Cr_Pts++;
            Cr_Pt1 = Cr_Pt;
        }
        else
        {
            if ( Cr_Pts == 1 )
            {
                Cr_Pts++;
                Cr_Pt2 = Cr_Pt;
            }
        }
    }
    return Cr_Pts;
}

// ========================================================
// GLSector2D constructor
GLSector2D::GLSector2D():
    m_Wrk_Trap( nullptr )
{
    //
}

// ========================================================
// GLSector2D constructor
GLSector2D::GLSector2D( double Azimuth, double Base_Angle, quint32 Near_B, quint32 Far_B, const GLPoint3D Pos )
{
    // Set base parameters
    setBaseAngle( Base_Angle );
//	printf( "\tBase Angle = %.3f\n", m_BaseAngle );
    setAzimuth( Azimuth );
//	printf( "\tAzimuth = %.3f\n", m_Azimuth );
    setBorders( Near_B,
                Far_B );
//	printf( "\tNear Border = %d\n\tFar Border = %d\n", m_nearBord, m_farBord );
    setPsn( Pos );
//	printf( "\tPosition: x = %d, y = %d\n\n", m_Position.x, m_Position.y );

    // Creation of working trapezium
    GLPointDouble3D Pt_A, Pt_B, Pt_C, Pt_D;
    Get_Wrk_Trap_Pts( Pt_A,
                      Pt_B,
                      Pt_C,
                      Pt_D );
    m_Wrk_Trap = new GLWrkTrap( Pt_A,
                                Pt_B,
                                Pt_C,
                                Pt_D );
}

// ========================================================
// GLSector2D constructor
GLSector2D::GLSector2D( quint32 Near_B,
                        quint32 Far_B,
                        double Beg_Az,
                        double End_Az,
                        GLPoint3D Pos )
{
//	printf( "\tBegin Azimuth = %.3f, End Azimuth = %.3f\n", begAz, endAz );
    double Azimuth = ( Beg_Az + End_Az ) / 2.0;
    double Base_Angle = End_Az - Beg_Az;

    // Set base parameters
    setBaseAngle( Base_Angle );
//	printf( "\tBase Angle = %.3f\n", m_BaseAngle );
    setAzimuth( Azimuth );
//	printf( "\tAzimuth = %.3f\n", m_Azimuth );
    setBorders( Near_B,
                Far_B );
//	printf( "\tNear Border = %d\n\tFar Border = %d\n", m_nearBord, m_farBord );
    setPsn( Pos );
//	printf( "\tPosition: x = %d, y = %d\n\n", m_Position.x, m_Position.y );

    // Creation of working trapezium
    GLPointDouble3D Pt_A, Pt_B, Pt_C, Pt_D;
    Get_Wrk_Trap_Pts( Pt_A,
                      Pt_B,
                      Pt_C,
                      Pt_D );
    m_Wrk_Trap = new GLWrkTrap( Pt_A,
                                Pt_B,
                                Pt_C,
                                Pt_D );
}

// ========================================================
GLSector2D::~GLSector2D()
{
    //
}

// ========================================================
GLSector2D::GLSector2D(const GLSector2D& sector2D) : GLSector(sector2D)
{

}

// ========================================================
GLSector2D& GLSector2D::operator= (const GLSector2D& sector2D)
{
  GLSector::operator= (sector2D);
  return *this;
}

// ========================================================
// Checks the condition of belonging (coord) into the sector.
bool GLSector2D::Point_In_Sector_2D( GLPointDouble3D& Coord ) const
{
    if ( Point_In_Base_Angle( Coord ) )
    {
        if ( Point_In_Border( Coord ) )
        {
            return true;
        }
    }
    return false;
}

// ========================================================
// Checks the condition of belonging (coord) into the base angle.
bool GLSector2D::Point_In_Base_Angle( const GLPointDouble3D& Coord ) const
{
    double Angle = 0;
    CTopocentric Coord_Topo( Coord.x,
                             Coord.y,
                             Coord.z );
    CSpherical Coord_Spherical;
    if ( !TOPO_SPHERICAL( &Coord_Topo,
                          &Coord_Spherical ) )
    {
        return false;
    }
    Angle = Coord_Spherical.m_dB;
    if ( Angle < 0 )
    {
        Angle = Angle + 2.0 * PI;
    }

    return Angle_In_Sector_2D( Angle );
}

// ========================================================
bool GLSector2D::Angle_In_Sector_2D( double Angle ) const
{
    if ( Angle < 0 )
    {
        Angle += 2 * PI;
    }
    double Half_Base_Angle = m_BaseAngle / 2.0;
    if ( ( m_Azimuth - Half_Base_Angle ) < 0 )
    {
        if ( Angle > PI )
        {
            if ( Angle >= 2.0 * PI + ( m_Azimuth - Half_Base_Angle ) )
            {
                return true;
            }
        }
        else
        {
            if ( Angle <= ( m_Azimuth + Half_Base_Angle ) )
            {
                return true;
            }
        }
    }
    else
    {
        if ( ( m_Azimuth + Half_Base_Angle ) >= ( 2.0 * PI ) )
        {
            if (
                    ( Angle >= ( m_Azimuth - Half_Base_Angle ) &&
                      Angle <= 2.0 * PI ) ||
                    ( Angle >= 0.0 &&
                      Angle <= ( m_Azimuth + Half_Base_Angle - 2.0 * PI ) )
                )
            {
                return true;
            }
        }
        else
        {
            if (
                    Angle >= ( m_Azimuth - Half_Base_Angle ) &&
                    Angle <= ( m_Azimuth + Half_Base_Angle )
                )
            {
                return true;
            }
        }
    }
    return false;
}

// ========================================================
// Checks the condition of belonging (coord) into the border of sector.
bool GLSector2D::Point_In_Border( const GLPointDouble3D& Coord ) const
{
    const GLPointDouble3D Pt_0;
    double Distance_To_Pt = GLMath::Segm_Length_2D( Coord,
                                                      Pt_0 );
    double Msr_Err = ( m_farBord - m_nearBord ) * 0.001;

    if ( Distance_To_Pt >= ( m_nearBord - Msr_Err ) &&
         Distance_To_Pt <= ( m_farBord + Msr_Err ) )
    {
        return true;
    }
    else
    {
        return false;
    }
}

// ========================================================
// Get 4 points for creation of working trapezium of sector.
void GLSector2D::Get_Wrk_Trap_Pts( GLPointDouble3D& Pt_A,
                                   GLPointDouble3D& Pt_B,
                                   GLPointDouble3D& Pt_C,
                                   GLPointDouble3D& Pt_D )
{
    // Point A
    Pt_A.x = ( double )m_nearBord * cos( m_Azimuth - m_BaseAngle / 2.0 );
    Pt_A.z = ( double )m_nearBord * sin( m_Azimuth - m_BaseAngle / 2.0 );

    // Point B
    Pt_B.x = ( double )m_farBord * cos( m_Azimuth - m_BaseAngle / 2.0 );
    Pt_B.z = ( double )m_farBord * sin( m_Azimuth - m_BaseAngle / 2.0 );

    // Point C
    Pt_C.x = ( double )m_farBord * cos( m_Azimuth + m_BaseAngle / 2.0 );
    Pt_C.z = ( double )m_farBord * sin( m_Azimuth + m_BaseAngle / 2.0 );

    // Point D
    Pt_D.x = ( double )m_nearBord * cos( m_Azimuth + m_BaseAngle / 2.0 );
    Pt_D.z = ( double )m_nearBord * sin( m_Azimuth + m_BaseAngle / 2.0 );
}

// ========================================================
// Checks the condition of straight ( x = K * z + B ) crossing the sector.
bool GLSector2D::Str_Cross_Sector( double K,
                                   double B )
{
    GLPointDouble3D pt1;
    GLPointDouble3D pt2;
    if ( m_Wrk_Trap->Str_Cross_Trap( K,
                                     B ) )
    {
        return true;
    }
    else
    {
        quint32 crPts = GLMath::Str_Cross_Circle( K,
                                                   B,
                                                   ( double )m_farBord,
                                                   pt1,
                                                   pt2 );
        if ( crPts == 0 )
        {
            return false;
        }
        if ( crPts == 1 )
        {
            if ( Point_In_Sector_2D( pt1 ) )
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        if ( crPts == 2 )
        {
            if ( Point_In_Sector_2D( pt1 ) || Point_In_Sector_2D( pt2 ) )
            {
                return true;
            }
            else
            {
                return false;
            }
        }
    }
    return false;
}

// ========================================================
// Checks the condition of straight ( x = K * z + B ) crossing the sector and return quantity of crossing points and its' coordinates.
quint32 GLSector2D::Str_Cross_Sector( double K,
                                      double B,
                                      GLPointDouble3D& Cr_Pt1,
                                      GLPointDouble3D& Cr_Pt2 )
{
    GLPointDouble3D Pt1, Pt2, Pt3, Pt4;
    quint32 Cir_Cr_Pts_Cnt = 0;
    quint32 Tr_Cr_Pts = m_Wrk_Trap->Str_Cross_Trap( K,
                                                  B,
                                                  Cr_Pt1,
                                                  Cr_Pt2 );
    quint32 Cr_Pts_Far = 0;
    quint32 Cr_Pts_Near = 0;
    switch ( Tr_Cr_Pts )
    {
        case 2: { return 2; }
        case 1:
        {
            Cr_Pts_Far = GLMath::Str_Cross_Circle( K,
                                                   B,
                                                   ( double )m_farBord,
                                                   Pt1,
                                                   Pt2 );
            switch ( Cr_Pts_Far )
            {
                case 2:
                {
                    if ( Point_In_Sector_2D( Pt1 ) &&
                         Point_In_Sector_2D( Pt2 ) )
                    {
                        if ( !GLMath::Points_Cmp_2D( Cr_Pt1,
                                                     Pt1 ) )
                        {
                            Cr_Pt2 = Pt1;
                            return 2;
                        }
                        else
                        {
                            if ( !GLMath::Points_Cmp_2D( Cr_Pt1,
                                                         Pt2 ) )
                            {
                                Cr_Pt2 = Pt2;
                                return 2;
                            }
                        }
                    }
                    else
                    {
                        if ( Point_In_Sector_2D( Pt1 ) &&
                             !Point_In_Sector_2D( Pt2 ) )
                        {
                            if ( !GLMath::Points_Cmp_2D( Cr_Pt1,
                                                         Pt1 ) )
                            {
                                Cr_Pt2 = Pt1;
                                return 2;
                            }
                        }
                        else
                        {
                            if ( !Point_In_Sector_2D( Pt1 ) &&
                                 Point_In_Sector_2D( Pt2 ) )
                            {
                                if ( !GLMath::Points_Cmp_2D( Cr_Pt1,
                                                             Pt2 ) )
                                {
                                    Cr_Pt2 = Pt2;
                                    return 2;
                                }
                            }
                        }
                    }
                    break;
                }
                case 1:
                {
                    if ( Point_In_Sector_2D( Pt1 ) )
                    {
                        if ( !GLMath::Points_Cmp_2D( Cr_Pt1,
                                                     Pt1 ) )
                        {
                            Cr_Pt2 = Pt1;
                        }
                    }
                    break;
                }
                case 0:
                default:
                {
                    break;
                }
            }
            Cr_Pts_Near = GLMath::Str_Cross_Circle( K,
                                                    B,
                                                    ( double )m_nearBord,
                                                    Pt3,
                                                    Pt4 );
            switch ( Cr_Pts_Near )
            {
                case 2:
                {
                    if ( Point_In_Sector_2D( Pt3 ) &&
                         Point_In_Sector_2D( Pt4 ) )
                    {
                        if ( !GLMath::Points_Cmp_2D( Cr_Pt1,
                                                     Pt3 ) )
                        {
                            Cr_Pt2 = Pt3;
                            return 2;
                        }
                        else
                        {
                            if ( !GLMath::Points_Cmp_2D( Cr_Pt1,
                                                         Pt4 ) )
                            {
                                Cr_Pt2 = Pt4;
                                return 2;
                            }
                        }
                    }
                    else
                    {
                        if ( Point_In_Sector_2D( Pt3 ) &&
                             !Point_In_Sector_2D( Pt4 ) )
                        {
                            if ( !GLMath::Points_Cmp_2D( Cr_Pt1,
                                                         Pt3 ) )
                            {
                                Cr_Pt2 = Pt3;
                                return 2;
                            }
                        }
                        else
                        {
                            if ( !Point_In_Sector_2D( Pt3 ) &&
                                 Point_In_Sector_2D( Pt4 ) )
                            {
                                if ( !GLMath::Points_Cmp_2D( Cr_Pt1,
                                                             Pt4 ) )
                                {
                                    Cr_Pt2 = Pt4;
                                    return 2;
                                }
                            }
                        }
                    }
                    break;
                }
                case 1:
                {
                    if ( Point_In_Sector_2D( Pt3 ) )
                    {
                        if ( !GLMath::Points_Cmp_2D( Cr_Pt1,
                                                     Pt3 ) )
                        {
                            Cr_Pt2 = Pt3;
                        }
                    }
                    break;
                }
                case 0:
                default:
                {
                    break;
                }
            }
            break;
        }
        case 0:
        {
            Cr_Pts_Far = GLMath::Str_Cross_Circle( K,
                                                   B,
                                                   ( double )m_farBord,
                                                   Pt1,
                                                   Pt2 );
            switch ( Cr_Pts_Far )
            {
                case 2:
                    if ( Point_In_Sector_2D( Pt1 ) &&
                         Point_In_Sector_2D( Pt2 ) )
                    {
                        Cr_Pt1 = Pt1;
                        Cr_Pt2 = Pt2;
                        return 2;
                    }
                    else
                    {
                        if ( Point_In_Sector_2D( Pt1 ) &&
                             !Point_In_Sector_2D( Pt2 ) )
                        {
                            Cr_Pt1 = Pt1;
                            Cir_Cr_Pts_Cnt++;
                        }
                        else
                        {
                            if ( !Point_In_Sector_2D( Pt1 ) &&
                                 Point_In_Sector_2D( Pt2 ) )
                            {
                                Cr_Pt1 = Pt2;
                                Cir_Cr_Pts_Cnt++;
                            }
                        }
                    }
                    break;
                case 1:
                    if ( Point_In_Sector_2D( Pt1 ) )
                    {
                        Cr_Pt1 = Pt1;
                        Cir_Cr_Pts_Cnt++;
                    }
                    break;
                case 0:
                    break;
                default:
                    break;
            }
            Cr_Pts_Near = GLMath::Str_Cross_Circle( K,
                                                    B,
                                                    ( double )m_nearBord,
                                                    Pt3,
                                                    Pt4 );
            switch ( Cr_Pts_Near )
            {
                case 2:
                {
                    if ( Point_In_Sector_2D( Pt3 ) &&
                         Point_In_Sector_2D( Pt4 ) )
                    {
                        Cr_Pt1 = Pt3;
                        Cr_Pt2 = Pt4;
                        return 2;
                    }
                    else
                    {
                        if ( Point_In_Sector_2D( Pt3 ) &&
                             !Point_In_Sector_2D( Pt4 ) )
                        {
                            if ( Cir_Cr_Pts_Cnt == 0 )
                            {
                                Cr_Pt1 = Pt3;
                                return 1;
                            }
                            else
                            {
                                if ( Cir_Cr_Pts_Cnt == 1 )
                                {
                                    Cr_Pt2 = Pt3;
                                    return 2;
                                }
                            }
                        }
                        else
                        {
                            if ( !Point_In_Sector_2D( Pt3 ) &&
                                 Point_In_Sector_2D( Pt4 ) )
                            {
                                if ( Cir_Cr_Pts_Cnt == 0 )
                                {
                                    Cr_Pt1 = Pt4;
                                    return 1;
                                }
                                else
                                {
                                    if ( Cir_Cr_Pts_Cnt == 1 )
                                    {
                                        Cr_Pt2 = Pt4;
                                        return 2;
                                    }
                                }
                            }
                            else
                            {
                                if ( !Point_In_Sector_2D( Pt3 ) &&
                                     !Point_In_Sector_2D( Pt4 ) )
                                {
                                    return Cir_Cr_Pts_Cnt;
                                }
                            }
                        }
                    }
                    break;
                }
                case 1:
                {
                    if ( Point_In_Sector_2D( Pt3 ) )
                    {
                        if ( Cir_Cr_Pts_Cnt == 0 )
                        {
                            Cr_Pt1 = Pt3;
                            return 1;
                        }
                        else
                        {
                            if ( Cir_Cr_Pts_Cnt == 1 )
                            {
                                Cr_Pt2 = Pt3;
                                return 2;
                            }
                        }
                    }
                    else
                    {
                        return Cir_Cr_Pts_Cnt;
                    }
                    break;
                }
                case 0:
                {
                    return Cir_Cr_Pts_Cnt;
                }
                default:
                {
                    break;
                }
            }
            break;
        }
        default:
        {
            break;
        }
    }
    return 0;
}

// ========================================================
bool GLSector2D::Segment_Cross_Sector( double K,
                                       double B,
                                       const GLPointDouble3D& Curr_Pt,
                                       const GLPointDouble3D& Fall_Pt )
{
    GLPointDouble3D Cr_Pt1, Cr_Pt2;
    quint32 Cr_Pts = Str_Cross_Sector( K,
                                       B,
                                       Cr_Pt1,
                                       Cr_Pt2 );
    switch ( Cr_Pts )
    {
        case 2:
        {
            if ( GLMath::Point_In_Segment( Curr_Pt,
                                           Fall_Pt,
                                           Cr_Pt1 ) )
            {
                return true;
            }
            else
            {
                if ( GLMath::Point_In_Segment( Curr_Pt,
                                               Fall_Pt,
                                               Cr_Pt2 ) )
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }
            break;
        }
        case 1:
        {
            if ( GLMath::Point_In_Segment( Curr_Pt,
                                           Fall_Pt,
                                           Cr_Pt1 ) )
            {
                return true;
            }
            else
            {
                return false;
            }
            break;
        }
        case 0:
        default:
        {
            return false;
        }
    }
    return false;
}

// ========================================================
GLPoint3D GLSector2D::Get_Pos()
{
    return m_Position;
}

/* END */

GLSector::GLSector(void):
 m_BaseAngle(0),
 m_Azimuth(-1),
 m_begSect(0),
 m_endSect(0),
 m_LoAngleElevtn(0),
 m_HiAngleElevtn(0),
 m_nearBord(0),
 m_farBord(0)
{

}

GLSector::GLSector(const GLSector& sector):
 m_BaseAngle(sector.m_BaseAngle),
 m_Azimuth(sector.m_Azimuth),
 m_begSect(sector.m_begSect),
 m_endSect(sector.m_endSect),
 m_LoAngleElevtn(sector.m_LoAngleElevtn),
 m_HiAngleElevtn(sector.m_HiAngleElevtn),
 m_nearBord(sector.m_nearBord),
 m_farBord(sector.m_farBord)
{
}

GLSector& GLSector::operator=(const GLSector &sector)
{
    m_BaseAngle     = sector.m_BaseAngle;
    m_Azimuth       = sector.m_Azimuth;
    m_begSect       = sector.m_begSect;
    m_endSect       = sector.m_endSect;
    m_LoAngleElevtn = sector.m_LoAngleElevtn;
    m_HiAngleElevtn = sector.m_HiAngleElevtn;
    m_nearBord      = sector.m_nearBord;
    m_farBord       = sector.m_farBord;

    return *this;
}

 GLSector::GLSector(GLBearing b1, GLBearing b2):
 m_BaseAngle(0),
 m_Azimuth(-1),
 m_LoAngleElevtn(0),
 m_HiAngleElevtn(0),
 m_nearBord(0),
 m_farBord(0)
{
 setBaseAngle(b1.b, b2.b);
}

bool GLSector::ptInSector( GLPoint3D& coord ) const
{
  if(ptInBaseAngle(coord) == true)
    if(ptInAngleElevat(coord) == true)
      if(ptInBorder(coord) == true) return true;
  return false;
}

/*bool GLSector::anglInSector( double angle ) const
{
  if(angle < 0)
  {
    angle += 2 * PI;
  }
  double halfBaseAngl = m_BaseAngle / 2;
  if( (m_Azimuth - halfBaseAngl) < 0 )
  {
      if(angle > PI)
      {
          if(angle >= 2.0*PI + (m_Azimuth - halfBaseAngl))
          {
            return true;
          }
      }
      else
      {
          if( angle <= (m_Azimuth + halfBaseAngl) )
          {
            return true;
          }
      }
  }
  else
  {
      if( angle >= (m_Azimuth - halfBaseAngl) &&
        angle <= (m_Azimuth + halfBaseAngl) )
    {
      return true;
    }
  }
  return false;
}*/

bool GLSector::anglInSector( double angle ) const
{
    if( m_BaseAngle == 0. )
        return false;

    if( angle < 0. )
    {
        angle += 2.*PI;
    }

    if( angle > 2.*PI )
    {
        angle -= 2.*PI;
    }

    double halfBaseAngl = m_BaseAngle / 2.;

    double angle_min = m_Azimuth - halfBaseAngl;
    if( angle_min < 0. )
        angle_min += 2.*PI;

    double angle_max = m_Azimuth + halfBaseAngl;
    if( angle_max > 2.*PI )
        angle_max -= 2.*PI;

    if( angle_min < angle_max )
    {
        if( (angle >= angle_min) && (angle <= angle_max) )
            return true;
    }
    else
    {
        if( (angle >= angle_min) && (angle <= 2.*PI) )
            return true;
        if( (angle <= angle_max) && (angle >= 0.) )
            return true;
    }
    return false;
}

bool GLSector::ptInBaseAngle( const GLPoint3D& coord ) const
{
  double angle = 0;
  angle = atan2( (double)coord.y - m_Position.y, (double)coord.x - m_Position.x );
  if( angle < 0 ) angle = angle + 2.0*PI;
  return anglInSector(angle);
}

bool GLSector::ptInBorder( const GLPoint3D& coord ) const
{
  quint32 distncToPt =
           GLdistance( GLPoint2D(coord.x,coord.y),
                       GLPoint2D(m_Position.x,m_Position.y) );
  if( distncToPt >= m_nearBord &&
      distncToPt <= m_farBord ) return true;
  else return false;
}

bool GLSector::ptInAngleElevat( const GLPoint3D& coord ) const
{
  quint32 distncToPt =
           GLdistance( GLPoint2D(coord.x,coord.y),
                       GLPoint2D(m_Position.x,m_Position.y) );
  double ptAnglElvtn = atan2( (double)coord.z/* - m_Position.h*/ ,(double)distncToPt );
  if( ptAnglElvtn < m_HiAngleElevtn &&
      ptAnglElvtn > m_LoAngleElevtn) return true;
  else return false;
}

void GLSector::setAzimuthExpl(double angl)
{
  m_Azimuth = angl;
}

void GLSector::setAzimuth(double angl)
{
  //assert(angl <= (2.0*PI + 0.01));
  double limit_angle_value(2.0*PI);

  while(angl > limit_angle_value)
        angl-=limit_angle_value;

  if(angl < 0)
  {
    angl+=2.0*PI;
  }
  //assert( (angl >= 0 && angl <= (2.0*PI + 0.01)) );

  m_Azimuth = angl;
  double halfSect = m_BaseAngle / 2;
  m_begSect = angl - halfSect;
  if(m_begSect < 0)
    m_begSect += 2.0*PI;
  m_endSect = angl + halfSect;
  if(m_endSect > 2.0*PI)
    m_endSect -= 2.0*PI;
}

bool GLSector::setBaseAngle(double begSect,double endSect)
{
 // assert(begSect <= (2.0*PI + 0.01));
 // assert(endSect <= (2.0*PI + 0.01));
    double limit_angle_value(2.0*PI);

    while(begSect > limit_angle_value)
        begSect-=limit_angle_value;

    while(endSect > limit_angle_value)
        endSect-=limit_angle_value;

  m_BaseAngle = endSect - begSect;

    if (PI < fabs(m_BaseAngle))
    {
        m_BaseAngle = (m_BaseAngle > 0) ?
                                        m_BaseAngle - 2*PI :
                                        m_BaseAngle + 2*PI;
    }

    m_BaseAngle = fabs(m_BaseAngle);

    double halfSect = m_BaseAngle / 2;

    if ((endSect - begSect) == m_BaseAngle)
    {
        m_Azimuth = begSect + halfSect;
    }
    else if ((begSect - endSect) == m_BaseAngle)
    {
        m_Azimuth = endSect + halfSect;
    }
    else if ((endSect - begSect) == (2*PI - m_BaseAngle))
    {
        m_Azimuth = endSect + halfSect;
    }
    else if ((begSect - endSect) == (2*PI - m_BaseAngle))
    {
        m_Azimuth = begSect + halfSect;
    }
    else
    {
        //assert(false);
        return false;
    }

    if(m_Azimuth > 2*PI)  m_Azimuth -= 2.0*PI;

  m_begSect = begSect;
  m_endSect = endSect;
  return true;
}

double GLSector::getAzimuth(void) const
{
  return m_Azimuth;
}

double GLSector::getBaseAngle(void) const
{
  return m_BaseAngle;
}

quint32 GLSector::getFarBordr(void) const
{
  return m_farBord;
}

quint32 GLSector::getNearBordr(void) const
{
  return m_nearBord;
}

double GLSector::getLoAngleElevtn(void) const
{
  return m_LoAngleElevtn;
}

double GLSector::getHiAngleElevtn(void) const
{
  return m_HiAngleElevtn;
}

double GLSector::getBegSect(void) const
{
  return m_begSect;
}

double GLSector::getEndSect(void) const
{
  return m_endSect;
}

void GLSector::setBaseAngle(double angl)
{
 // assert( (angl >= 0 && angl <= (2.0 * PI + 0.001)) );
  double limit_angle_value(2.0*PI);

  if(angl > limit_angle_value)
      angl = 0.5*PI;
  m_BaseAngle = angl;
}

bool GLSector::setBorders(quint32 nearB,quint32 farB)
{
  if(farB > 0 && farB > nearB)
  {
    m_nearBord = nearB;
    m_farBord = farB;
    return true;
  }
  return false;
}

void GLSector::setPsn(const GLPoint3D& psn)
{
  m_Position = psn;
}


bool GLSector::setAnglsElevtn(double loB,double hiB)
{
  if((loB < hiB && hiB <= (PI / 2.0 + 0.001)))
  {
    m_LoAngleElevtn = loB;
    m_HiAngleElevtn = hiB;
    return true;
  }
  return false;
}

double GLmin2rad( short minutes )
{
  return ( (double)minutes * PI / 10800.0 );
}

double GLmin2rad( double minutes )
{
  return ( minutes * PI / 10800.0 );
}

double GLrad2min( double radians )
{
  return ( radians * 10800.0 / PI );
}

double GLsec2rad(short sec )
{
  return ( (double)sec * PI / 648000.0 );
}

double GLsec2rad( double sec )
{
  return ( sec * PI / 648000.0 );
}

double GLrad2sec( double radians )
{
  return ( radians * 648000.0 / PI );
}

double GLgrad2rad( short degree )
{
  return ( (double)degree * PI / 180.0 );
}

short GLrad2grad( double radians )
{
  return (short)( radians * 180.0 / PI );
}

double GLdegr2rad( double degree )
{
  return degree * PI / 180.0;
}

double GLrad2degr( double radians )
{
  return radians * 180.0 / PI;
}

double GLdegr2min( double degrees )
{
    return ( degrees * 60.0 );
}

double GLmin2degr( double mins )
{
    return ( mins / 60.0 );
}

// ========================================================
void GL_Degrees_2_DMS( double Degrees,
                       double &Deg,
                       double &Min,
                       double &Sec )
{
    double Tmp;
    // 1, Degrees
    Deg = ( double )( ( quint64 )( Degrees ) );
    // 2, Minutes
    Tmp = ( Degrees - Deg ) * 60.0;
    Min = ( double )( ( quint64 )( Tmp ) );
    // 3, Seconds
    Sec = ( Tmp - Min ) * 60.0;
}

// ========================================================
void GL_DMS_2_Degrees( double Deg,
                       double Min,
                       double Sec,
                       double &Degrees )
{
    Degrees = ( ( Deg * 3600.0 + Min * 60.0 + Sec ) / 3600.0 );
}
