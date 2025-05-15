/*! \file GLSector.h
\brief Header file for GLSector class.
*/

// _OS_INDEPENDENT_

#ifndef _GLSector_h_
#define _GLSector_h_

#include "GLGeometry.h"
#include "GLPolygon.h"
#include <cassert>

/* ikemy 2010-11-09 */
/* BEGIN */
#define MSR_ERR 0.1

// Warning! All accounts in topocentric coordinate system.
// x - to north, z - to east.
class GLMath
{
public:
    GLMath();

    // PACKAGE		:  GL.
    // FUNCTION		:  QE( const double A, const double B, const double C, double& Z1, double& Z2 ).
    // DESCRIPTION	:  Quadric equations solution.
    // INPUTS		:  A, B, C - coefficients of quadric equation; Z1, Z2 - buffers for probable roots.
    // RETURNS		:  Number of real roots of quadric equation.
    static quint32 QE( const double A,
                       const double B,
                       const double C,
                       double& Z1,
                       double& Z2 );
    // Р РµС€РµРЅРёРµ РєРІР°РґСЂР°С‚РЅС‹С… СѓСЂР°РІРЅРµРЅРёР№.

    // PACKAGE		:  GL.
    // FUNCTION		:  SE_From_2_Pts( const GLPointDouble3D Pt1, const GLPointDouble3D Pt2, double& K, double& B ).
    // DESCRIPTION	:  Get equation of straight from 2 points.
    // INPUTS		:  Pt1, Pt2 - 2 points; K, B - buffers for straight coefficients.
    // RETURNS		:  True - success, false - fail.
    static bool SE_From_2_Pts( const GLPointDouble3D Pt1,
                               const GLPointDouble3D Pt2,
                               double& K,
                               double& B );
    // РџРѕР»СѓС‡РёС‚СЊ СѓСЂР°РІРЅРµРЅРёРµ РїСЂСЏРјРѕР№ РїРѕ РґРІСѓРј С‚РѕС‡РєР°Рј.

    // PACKAGE		:  GL.
    // FUNCTION		:  Segm_Length_2D( const GLPointDouble3D Pt1, const GLPointDouble3D Pt2 ).
    // DESCRIPTION	:  Get length of segment from 2 points.
    // INPUTS		:  Pt1, Pt2 - 2 points.
    // RETURNS		:  Length of segment.
    static double Segm_Length_2D( const GLPointDouble3D Pt1,
                                    const GLPointDouble3D Pt2 );
    // РџРѕР»СѓС‡РёС‚СЊ РґР»РёРЅСѓ РѕС‚СЂРµР·РєР° РїРѕ РґРІСѓРј С‚РѕС‡РєР°Рј.

    // PACKAGE		:  GL.
    // FUNCTION		:  Segm_Length_3D( const GLPointDouble3D Pt1, const GLPointDouble3D Pt2 ).
    // DESCRIPTION	:  Get length of segment from 2 points.
    // INPUTS		:  Pt1, Pt2 - 2 points.
    // RETURNS		:  Length of segment.
    static double Segm_Length_3D( const GLPointDouble3D Pt1,
                                    const GLPointDouble3D Pt2 );
    // РџРѕР»СѓС‡РёС‚СЊ РґР»РёРЅСѓ РѕС‚СЂРµР·РєР° РїРѕ РґРІСѓРј С‚РѕС‡РєР°Рј.

    // PACKAGE		:  GL.
    // FUNCTION		:  Str_Cross_Circle( double K, double B, double R, GLPointDouble3D& Pt1, GLPointDouble3D& Pt2 ).
    // DESCRIPTION	:  Straight and circle crossing verifying.
    // INPUTS		:  K, B - coefficients of straight; R - radius of circle; Pt1, Pt2 - probable crossing points.
    // RETURNS		:  Count of crossing points.
    static quint32 Str_Cross_Circle( double K, double B, double R, GLPointDouble3D& Pt1, GLPointDouble3D& Pt2 );
    // РќР°Р№С‚Рё С‚РѕС‡РєРё РїРµСЂРµСЃРµС‡РµРЅРёСЏ РїСЂСЏРјРѕР№ Рё РѕРєСЂСѓР¶РЅРѕСЃС‚Рё.

    static bool Points_Cmp_2D( const GLPointDouble3D& Pt1,
                               const GLPointDouble3D& Pt2 );
    // РЎСЂР°РІРЅРёС‚СЊ РґРІРµ С‚РѕС‡РєРё.

    static bool Point_In_Segment( const GLPointDouble3D& SPt1,
                                  const GLPointDouble3D& SPt2,
                                  const GLPointDouble3D& TGT_Pt );
    // РџСЂРѕРІРµСЂРёС‚СЊ, РїСЂРёРЅР°РґР»РµР¶РёС‚ Р»Рё С‚РѕС‡РєР° trgtPt РѕС‚СЂРµР·РєСѓ ( SPt1, SPt2 ).
    // РџСЂРµРґРїРѕР»Р°РіР°РµС‚СЃСЏ, С‡С‚Рѕ СѓР¶Рµ РґРѕРїРѕРґР»РёРЅРЅРѕ РёР·РІРµСЃС‚РЅРѕ,
    // С‡С‚Рѕ РІСЃРµ С‚СЂРё С‚РѕС‡РєРё Р»РµР¶Р°С‚ РЅР° РѕРґРЅРѕР№ РїСЂСЏРјРѕР№.

    // PACKAGE		:  GL.
    // FUNCTION		:  Get_Tangent_Pts( const GLPointDouble3D Pt0, double R, GLPointDouble3D& Pt1, GLPointDouble3D& Pt2 ).
    // DESCRIPTION	:  Get tangent points to circle from point outside of this circle.
    // INPUTS		:  Pt0 - point outside circle; R - radius of circle; Pt1, Pt2 - tangent points.
    // RETURNS		:  Number of tangent points.
    static quint8 Get_Tangent_Pts( const GLPointDouble3D& Pt0,
                                   double R,
                                   GLPointDouble3D& Pt1,
                                   GLPointDouble3D& Pt2 );
    // РџРѕР»СѓС‡РёС‚СЊ РєРѕРѕСЂРґРёРЅР°С‚С‹ С‚РѕС‡РµРє РєР°СЃР°РЅРёСЏ Рє РѕРєСЂСѓР¶РЅРѕСЃС‚Рё РёР· С‚РѕС‡РєРё, Р»РµР¶Р°С‰РµР№ РІРЅРµ СЌС‚РѕР№ РѕРєСЂСѓР¶РЅРѕСЃС‚Рё.

    // PACKAGE		:  GL.
    // FUNCTION		:  Get_Int_Part( double D ).
    // DESCRIPTION	:  Get integer part (floor) of fractional digit.
    // INPUTS		:  D - fractional digit.
    // RETURNS		:  Integer part (floor) of fractional digit.
    static double Get_Int_Part( double D );
    // РџРѕР»СѓС‡РёС‚СЊ С†РµР»СѓСЋ С‡Р°СЃС‚СЊ (РїРѕР») РґСЂРѕР±РЅРѕРіРѕ С‡РёСЃР»Р°.

    // PACKAGE		:  GL.
    // FUNCTION		:  Get_Fract_Part( double D ).
    // DESCRIPTION	:  Get fractional part of fractional digit.
    // INPUTS		:  D - fractional digit.
    // RETURNS		:  Fractional part of fractional digit.
    static double Get_Fract_Part( double D );
    // РџРѕР»СѓС‡РёС‚СЊ РґСЂРѕР±РЅСѓСЋ С‡Р°СЃС‚СЊ РґСЂРѕР±РЅРѕРіРѕ С‡РёСЃР»Р°.

    // PACKAGE		: GL.
    // FUNCTION		: Course_Angle( const GLPoint3D& TGT_Coord, const GLVelocity3D& TGT_Vel, const GLPoint3D& L_Coord ).
    // DESCRIPTION	: Calculate course angle.
    // INPUT		: TGT_Coord - target coordinates,
    // TGT_Vel - target velocity,
    // L_Coord - launcher coordinates.
    // OUTPUT		: Value of course angle.
    static double Course_Angle( const GLPoint3D& TGT_Coord,
                                   const GLVelocity3D& TGT_Vel,
                                   const GLPoint3D& L_Coord );
    // Р Р°СЃС‡РёС‚Р°С‚СЊ РєСѓСЂСЃ Р’Рћ.

    static GLPointDouble3D Extrapolate( const GLPointDouble3D& TGT_Coord,
                                        const GLVelocity3D& TGT_Vel,
                                        double TSec );
    // Р­РєСЃС‚СЂР°РїРѕР»РёСЂРѕРІР°С‚СЊ РєРѕРѕСЂРґРёРЅР°С‚С‹.

    static quint32 Get_Cross_Pts_Str_Circle( const GLPoint3D& Pt_AO,
                                             const GLVelocity3D& Vel_AO,
                                             double R,
                                             GLPointDouble3D& Pt1,
                                             GLPointDouble3D& Pt2 );
    // РџРѕР»СѓС‡РёС‚СЊ С‚РѕС‡РєРё РїРµСЂРµСЃРµС‡РµРЅРёСЏ РїСЂСЏРјРѕР№ СЃ РѕРєСЂСѓР¶РЅРѕСЃС‚СЊСЋ.

    static double Round( double Dbl_Val,
                           quint32 Prec );
    // РћРєСЂСѓРіР»РёС‚СЊ СЃ Р·Р°РґР°РЅРЅРѕР№ С‚РѕС‡РЅРѕСЃС‚СЊСЋ.

    static bool In_Target( const GLPointFloat3D &TGT_Coord,
                           const GLVelocityFloat3D &TGT_Vel,
                           const GLPointFloat3D &Launcher_Coord );
    // Р’С…РѕРґСЏС‰Р°СЏ С†РµР»СЊ?
};

// РѕС‚СЂРµР·РѕРє РЅР° РїР»РѕСЃРєРѕСЃС‚Рё
class GLSegment
{
public:
    // constructor
    GLSegment( GLPointDouble3D Pt1,
               GLPointDouble3D Pt2 );

    // straight and segment crossing verifying
    bool Str_Cross_Segment( double K,
                            double B,
                            GLPointDouble3D& Cr_Pt );

private:
    // Begin point
    GLPointDouble3D m_Pt1;
    // End point
    GLPointDouble3D m_Pt2;
    // 'K' coefficient of straight equation
    double m_K;
    // 'B' coefficient of straight equation
    double m_B;
    // Length
    double m_Length;
};

class GLWrkPolygon
{
public:
    GLWrkPolygon( std::vector< GLSegment > &Segm_List );
    GLWrkPolygon( GLPolygon &PGN );

    // PACKAGE		:  GL.
    // FUNCTION		:  Str_Cross_Polygon( double K, double B ).
    // DESCRIPTION	:  Straight and polygon segments crossing verifying.
    // INPUTS		:  K, B - coefficients of straight.
    // RETURNS		:  True, if crossing exists, another - false.
    bool Str_Cross_Polygon( double K,
                            double B );
    // РїСЂРѕРІРµСЂРєР°, РїРµСЂРµСЃРµРєР°РµС‚ Р»Рё РїСЂСЏРјР°СЏ РјРЅРѕРіРѕСѓРіРѕР»СЊРЅРёРє

    // PACKAGE		:  GL.
    // FUNCTION		:  Str_Cross_Polygon( double K, double B, std::vector< GLPointDouble3D > &Cr_Pts_Vect ).
    // DESCRIPTION	:  Straight and polygon segments crossing verifying.
    // INPUTS		:  K, B - coefficients of straight; reference to list of probable crossing points.
    // RETURNS		:  Quantity of crossing points.
    quint32 Str_Cross_Polygon( double K,
                               double B,
                               std::vector< GLPointDouble3D > &Cr_Pts_Vect );
    // РїСЂРѕРІРµСЂРєР°, РїРµСЂРµСЃРµРєР°РµС‚ Р»Рё РїСЂСЏРјР°СЏ РјРЅРѕРіРѕСѓРіРѕР»СЊРЅРёРє

    bool Is_Valid();

    std::vector< GLSegment > m_Segm_List;
};

class GLWrkTrap
{
public:
    GLWrkTrap( GLPointDouble3D Pt_A,
               GLPointDouble3D Pt_B,
               GLPointDouble3D Pt_C,
               GLPointDouble3D Pt_D );

    // PACKAGE		:  GL.
    // FUNCTION		:  Str_Cross_Trap( double K, double B ).
    // DESCRIPTION	:  Straight and trapezium segments crossing verifying.
    // INPUTS		:  K, B - coefficients of straight.
    // RETURNS		:  True, if crossing exists, another - false.
    bool Str_Cross_Trap( double K,
                         double B );
    // РїСЂРѕРІРµСЂРєР°, РїРµСЂРµСЃРµРєР°РµС‚ Р»Рё РїСЂСЏРјР°СЏ С‚СЂР°РїРµС†РёСЋ

    // PACKAGE		:  GL.
    // FUNCTION		:  Str_Cross_Trap( double K, double B, GLPointDouble3D& Cr_Pt1, GLPointDouble3D& Cr_Pt2 ).
    // DESCRIPTION	:  Straight and trapezium segments crossing verifying.
    // INPUTS		:  K, B - coefficients of straight; crPt1, crPt2 - references to probable crossing points.
    // RETURNS		:  Quantity of crossing points.
    quint32 Str_Cross_Trap( double K,
                            double B,
                            GLPointDouble3D& Cr_Pt1,
                            GLPointDouble3D& Cr_Pt2 );
    // РїСЂРѕРІРµСЂРєР°, РїРµСЂРµСЃРµРєР°РµС‚ Р»Рё РїСЂСЏРјР°СЏ С‚СЂР°РїРµС†РёСЋ

private:
    // 4 segments of trapezium
    GLSegment m_Trap_Segm_AB;
    GLSegment m_Trap_Segm_BC;
    GLSegment m_Trap_Segm_CD;
    GLSegment m_Trap_Segm_DA;
};
/* END */

//PACKAGE		:  GL
//CLASS			:  GLSector
//DESCRIPTION	:  Sector.
class GLSector
{
public:
    //PACKAGE		:  GL
    //FUNCTION		:  CGLDataQueue::CGLDataQueue( void )
    //DESCRIPTION	:  Class constructor.
    //INPUTS		:  size - Size of queue.
    //RETURNS		:  NONE
    GLSector( void );

    GLSector(const GLSector& sector);

    GLSector& operator = (const GLSector& sector);

    //PACKAGE		:  GL
    //FUNCTION		:  GLSector::GLSector( GLBearing b1, GLBearing b2 );
    //DESCRIPTION	:  Class constructor.
    //INPUTS		:  b1 - Left border, b2 - Right border.
    //RETURNS		:  NONE
    GLSector( GLBearing b1,	//Left border.
        GLBearing b2	//Right border.
        );

    //PACKAGE		:  GL
    //FUNCTION		:  double GLSector::getAzimuth(void) const;
    //DESCRIPTION	:  Retrieves sector azimuth.
    //INPUTS		:  None.
    //RETURNS		:  sector azimuth.
    double getAzimuth(void) const;

    //PACKAGE		:  GL
    //FUNCTION		:  double GLSector::getBaseAngle(void) const;
    //DESCRIPTION	:  Retrieves base angle.
    //INPUTS		:  None.
    //RETURNS		:  base angle.
    double getBaseAngle(void) const;

    //PACKAGE		:  GL
    //FUNCTION		:  double GLSector::getBegSect(void) const;
    //DESCRIPTION	:  Retrieves preamble azimuth of sector.
    //INPUTS		:  None.
    //RETURNS		:  preamble azimuth of sector.
    double getBegSect(void) const;

    //PACKAGE		:  GL
    //FUNCTION		:  double GLSector::getEndSect(void) const;
    //DESCRIPTION	:  Retrieves final azimuth of sector.
    //INPUTS		:  None.
    //RETURNS		:  final azimuth of sector.
    double getEndSect(void) const;

    //PACKAGE		:  GL
    //FUNCTION		:  quint32 GLSector::getFarBordr(void) const;
    //DESCRIPTION	:  Retrieves far border of sector.
    //INPUTS		:  None.
    //RETURNS		:  far border of sector.
    quint32 getFarBordr(void) const;

    //PACKAGE		:  GL
    //FUNCTION		:  quint32 GLSector::getNearBordr(void) const;
    //DESCRIPTION	:  Retrieves near border of sector.
    //INPUTS		:  None.
    //RETURNS		:  near border of sector.
    quint32 getNearBordr(void) const;

    //PACKAGE		:  GL
    //FUNCTION		:  double GLSector::getLoAngleElevtn(void) const;
    //DESCRIPTION	:  Retrieves lower border of sector.
    //INPUTS		:  None.
    //RETURNS		:  lower border of sector.
    double getLoAngleElevtn(void) const;

    //PACKAGE		:  GL
    //FUNCTION		:  double GLSector::getHiAngleElevtn(void) const;
    //DESCRIPTION	:  Retrieves higher border of sector.
    //INPUTS		:  None.
    //RETURNS		:  higher border of sector.
    double getHiAngleElevtn(void) const;

    //PACKAGE		:  GL
    //FUNCTION		:  void GLSector::setAzimuth( double angl );
    //DESCRIPTION	:  Sets azimuth.
    //INPUTS		:  angl - azimuth. Range (0,2*pi)
    //RETURNS		:  None.
    void setAzimuth( double angl	//azimuth.
        );

    //PACKAGE		:  GL
    //FUNCTION		:  void GLSector::setAzimuthExpl( double angl );
    //DESCRIPTION	:  Sets azimuth explicit.
    //INPUTS		:  angl - azimuth. Range (0,2*pi)
    //RETURNS		:  None.
    void setAzimuthExpl( double angl	//azimuth.
        );

    //PACKAGE		:  GL
    //FUNCTION		:  void GLSector::setBaseAngle(double angl);
    //DESCRIPTION	:  Sets size of base angle.
    //INPUTS		:  angl - base angle. Range (0,pi)
    //RETURNS		:  None.
    void setBaseAngle( double angl	//size of base angle.
        );

    //PACKAGE		:  GL
    //FUNCTION		:  void GLSector::setBaseAngle(double begSect,double endSect);
    //DESCRIPTION	:  Sets size of base angle & azimuth.
    //INPUTS		:  begSect - preamble azimuth of sector. endSect - final azimuth of sector. Range (0,2*pi)
    //RETURNS		:  None.
    bool setBaseAngle( double begSect,	//preamble azimuth of sector.
        double endSect	//final azimuth of sector..
        );

    //PACKAGE		:  GL
    //FUNCTION		:  void GLSector::setBorders(quint32 nearB,quint32 farB);
    //DESCRIPTION	:  Sets borders of sector.
    //INPUTS		:  nearB - Near border of sector, farB - Far border of sector..
    //RETURNS		:  None.
    bool setBorders( quint32 nearB,	//Near border of sector.
        quint32 farB	//Far border of sector.
        );

    //PACKAGE		:  GL
    //FUNCTION		:  void GLSector::setPsn(const GLPoint3D& psn);
    //DESCRIPTION	:  Sets position of sector in space.
    //INPUTS		:  psn - Position of sector in space.
    //RETURNS		:  None.
    void setPsn( const GLPoint3D& psn	//Position of sector in space.
        );

    //PACKAGE		:  GL
    //FUNCTION		:  void GLSector::setAnglsElevtn(double loB,double hiB);
    //DESCRIPTION	:  Sets angles of elevation.
    //INPUTS		:  loB Low border of angle of elevation, hiB - High border of angle of elevation.
    //RETURNS		:  None.
    bool setAnglsElevtn( double loB,	//Low border of angle of elevation.
        double hiB	//High border of angle of elevation.
        );

    //PACKAGE		:   GL.
    //FUNCTION		:   bool GLSector::ptInSector( GLPoint3D& coord ) const;
    //DESCRIPTION	:	checks the condition of belonging (coord) into the sector.
    //INPUTS		:   coord - coordinates of point.
    //RETURNS		:   true if point belonging into sector another false.
    virtual bool ptInSector( GLPoint3D& coord	//
        ) const;

    //PACKAGE		:  GL
    //FUNCTION		:  bool GLSector::anglInSector( double angle ) const;
    //DESCRIPTION	: Checks the condition of belonging (angle) into the sector.
    //INPUTS		:  angle - Checking angle.
    //RETURNS		:  true if angle belonging into sector another false.
    bool anglInSector( double angle	//Checking angle.
        ) const;

    //PACKAGE		:  GL
    //FUNCTION		:  bool GLSector::ptInBaseAngle( const GLPoint3D& coord ) const;
    //DESCRIPTION	: Checks the condition of belonging (coord) into the base angle.
    //INPUTS		:  coord - Checking point.
    //RETURNS		:  true if point belonging into base angle of sector another false.
    bool ptInBaseAngle( const GLPoint3D& coord	//Checking point.
        ) const;

    //PACKAGE		:  GL
    //FUNCTION		:  bool GLSector::ptInBorder( const GLPoint3D& coord ) const;
    //DESCRIPTION	: Checks the condition of belonging (coord) into the border of sector.
    //INPUTS		:  coord - Checking point.
    //RETURNS		:  true if point belonging into the border of sector of sector another false.
    bool ptInBorder( const GLPoint3D& coord		//Checking point.
        ) const;

    //PACKAGE		:  GL
    //FUNCTION		:  bool GLSector::ptInBorder( const GLPoint3D& coord ) const;
    //DESCRIPTION	: Checks the condition of belonging (coord) into the angle of elevation.
    //INPUTS		:  coord - Checking point.
    //RETURNS		:  true if point belonging into the angle of elevation of sector another false.
    bool ptInAngleElevat( const GLPoint3D& coord	//Checking point.
        ) const;

protected:

    double m_BaseAngle;
    //base angle.

    double m_Azimuth;
    //azimuth.

    double m_begSect;
    //preamble azimuth of sector.

    double m_endSect;
    //final azimuth of sector.

    double m_LoAngleElevtn;
    //Low border of angle of elevation.

    double m_HiAngleElevtn;
    //High border of angle of elevation.

    quint32 m_nearBord;
    //Near border.

    quint32 m_farBord;
    //Far border.

    GLPoint3D m_Position;
    //Position of sector in space.
};

/* ikemy 2010-11-10 */
/* BEGIN */
// Warning! All accounts in topocentric coordinate system.
class GLSector2D : public GLSector
{
public:
    GLSector2D();
    GLSector2D( double Azimuth, double Base_Angle, quint32 Near_B, quint32 Far_B, GLPoint3D Pos );
    GLSector2D( quint32 Near_B, quint32 Far_B, double Beg_Az, double End_Az, GLPoint3D Pos );
    virtual ~GLSector2D();

    GLSector2D(const GLSector2D& sector2D);
    GLSector2D& operator= (const GLSector2D& sector2D);


    // PACKAGE		:	GL.
    // FUNCTION		:	Point_In_Sector_2D( GLPointDouble3D& Coord ) const.
    // DESCRIPTION	:	Checks the condition of belonging (coord) into the sector.
    // INPUTS		:	Coord - coordinates of point.
    // RETURNS		:	True, if point belonging into sector, another - false.
    bool Point_In_Sector_2D( GLPointDouble3D& Coord ) const;

    // PACKAGE		:	GL.
    // FUNCTION		:	Point_In_Base_Angle( const GLPointDouble3D& Coord ) const.
    // DESCRIPTION	:	Checks the condition of belonging (coord) into the base angle.
    // INPUTS		:	Coord - checking point.
    // RETURNS		:	True, if point belonging into base angle of sector, another - false.
    bool Point_In_Base_Angle( const GLPointDouble3D& Coord ) const;

    //PACKAGE		:  GL.
    //FUNCTION		:  Angle_In_Sector_2D( double Angle ) const;
    //DESCRIPTION	:  Checks the condition of belonging (angle) into the sector.
    //INPUTS		:  Angle - Checking angle.
    //RETURNS		:  True if angle belonging into sector another false.
    bool Angle_In_Sector_2D( double Angle	//Checking angle.
        ) const;

    // PACKAGE		:	GL.
    // FUNCTION		:	Point_In_Border( const GLPointDouble3D& Coord ) const.
    // DESCRIPTION	:	Checks the condition of belonging (coord) into the border of sector.
    // INPUTS		:	Coord - Checking point.
    // RETURNS		:	True, if point belonging into the border of sector, another - false.
    bool Point_In_Border( const GLPointDouble3D& Coord ) const;

    // PACKAGE		:	GL.
    // FUNCTION		:	Str_Cross_Sector( double K, double B ).
    // DESCRIPTION	:	Checks the condition of straight ( y = K * x + B ) crossing the sector.
    // INPUTS		:	K, B - coefficients of straight.
    // RETURNS		:	True, if straight cross the sector, another - false.
    bool Str_Cross_Sector( double K,
                           double B );

    // PACKAGE		:	GL.
    // FUNCTION		:	Str_Cross_Sector( double K, double B, GLPointDouble3D& Cr_Pt1, GLPointDouble3D& Cr_Pt2 ).
    // DESCRIPTION	:	Checks the condition of straight ( y = K * x + B ) crossing the sector.
    // INPUTS		:	K, B - coefficients of straight; Cr_Pt1, Cr_Pt2 - references to probable crossing points.
    // RETURNS		:	Quantity of crossing points.
    quint32 Str_Cross_Sector( double K,
                              double B,
                              GLPointDouble3D& Cr_Pt1,
                              GLPointDouble3D& Cr_Pt2 );

    // PACKAGE		:	GL.
    // FUNCTION		:	Segment_Cross_Sector( double K, double B, const GLPointDouble3D& Curr_Pt, const GLPointDouble3D& Fall_Pt ).
    // DESCRIPTION	:	Checks the condition of straight ( y = K * x + B ) crossing the sector.
    // INPUTS		:	K, B - coefficients of straight; ?; Fall_Pt - reference to fall point of BT.
    // RETURNS		:	Quantity of crossing points.
    bool Segment_Cross_Sector( double K,
                               double B,
                               const GLPointDouble3D& Curr_Pt,
                               const GLPointDouble3D& Fall_Pt );

    GLPoint3D Get_Pos();

private:
    GLWrkTrap* m_Wrk_Trap;
    // Work trapezium of sector

    // PACKAGE		:	GL.
    // FUNCTION		:	Get_Wrk_Trap_Pts( GLPointDouble3D& Pt_A, GLPointDouble3D& Pt_B, GLPointDouble3D& Pt_C, GLPointDouble3D& Pt_D ).
    // DESCRIPTION	:	Get 4 points for creation of working trapezium of sector.
    // INPUTS		:	Pt1, pt2, pt3, pt4 - buffers for points' coordinates writing.
    // RETURNS		:	None.
    void Get_Wrk_Trap_Pts( GLPointDouble3D& Pt_A,
                           GLPointDouble3D& Pt_B,
                           GLPointDouble3D& Pt_C,
                           GLPointDouble3D& Pt_D );
};
/* END */

//PACKAGE		:  GL
//FUNCTION		:  double GLmin2rad( short minutes );
//DESCRIPTION	:  Converts minutes to radians.
//INPUTS		:  minutes - Value in minutes.
//RETURNS		:  Converted value in radians.
double GLmin2rad( short minutes	//Value in minutes.
    );

//PACKAGE		:  GL
//FUNCTION		:  double GLmin2rad( double minutes );
//DESCRIPTION	:  Converts minutes to radians.
//INPUTS		:  minutes - Value in minutes.
//RETURNS		:  Converted value in radians.
double GLmin2rad( double minutes	//Value in minutes.
    );

//PACKAGE		:  GL
//FUNCTION		:  double GLrad2min( double radians );
//DESCRIPTION	:  Converts radians to minutes.
//INPUTS		:  radians - Value in radians.
//RETURNS		:  Converted value in minutes.
double GLrad2min( double radians
    );

//PACKAGE		:  GL
//FUNCTION		:  short GLrad2grad( double radians );
//DESCRIPTION	:  Converts radians to grades.
//INPUTS		:  radians - Value in radians.
//RETURNS		:  Converted value in grades.
short GLrad2grad( double radians );

//PACKAGE		:  GL
//FUNCTION		:  double GLgrad2rad( short degree );
//DESCRIPTION	:  Converts degree to radians.
//INPUTS		:  degree - Value in degree.
//RETURNS		:  Converted value in radians.
double GLgrad2rad( short degree );

//PACKAGE		:  GL
//FUNCTION		:  double GLrad2degr( double radians );
//DESCRIPTION	:  Converts radians to degrees.
//INPUTS		:  radians - value in radians.
//RETURNS		:  Converted value in degrees.
double GLrad2degr( double radians );

//PACKAGE		:  GL
//FUNCTION		:  double GLdegr2rad( double degree );
//DESCRIPTION	:  Converts degrees to radians.
//INPUTS		:  degree - value in degrees.
//RETURNS		:  Converted value in radians.
double GLdegr2rad( double degree );

//PACKAGE		:  GL
//FUNCTION		:  double GLsec2rad( short sec );
//DESCRIPTION	:  Converts seconds to radians.
//INPUTS		:  sec - Value in seconds.
//RETURNS		:  Converted value in radians.
double GLsec2rad( short sec );

//PACKAGE		:  GL
//FUNCTION		:  double GLsec2rad( double sec );
//DESCRIPTION	:  Converts seconds to radians.
//INPUTS		:  sec - Value in seconds.
//RETURNS		:  Converted value in radians.
double GLsec2rad( double sec );

//PACKAGE		:  GL
//FUNCTION		:  int GLrad2sec( double radians );
//DESCRIPTION	:  Converts radians to seconds.
//INPUTS		:  radians - Value in radians.
//RETURNS		:  Converted value in seconds.
double GLrad2sec( double radians );

//PACKAGE		:  GL
//FUNCTION		:  double GLdegr2min( double degrees );
//DESCRIPTION	:  Converts degrees to minutes.
//INPUTS		:  degrees - Value in degrees.
//RETURNS		:  Converted value in minutes.
double GLdegr2min( double degrees );

//PACKAGE		:  GL
//FUNCTION		:  double GLmin2degr( double mins );
//DESCRIPTION	:  Converts minutes to degrees.
//INPUTS		:  mins - Value in minutes.
//RETURNS		:  Converted value in degrees.
double GLmin2degr( double mins );

void GL_Degrees_2_DMS( double Degrees,
                       double &Deg,
                       double &Min,
                       double &Sec );

void GL_DMS_2_Degrees( double Deg,
                       double Min,
                       double Sec,
                       double &Degrees );

//#include "GLSector.inl"

#endif //_GLSector_h_
