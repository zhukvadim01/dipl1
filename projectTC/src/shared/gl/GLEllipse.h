#ifndef GLELLIPSE_H
#define GLELLIPSE_H

#include "GLGeometry.h"
#include <QDateTime>

// PACKAGE		:	GL
// STRUCTURE	:	GLEllipse
// DESCRIPTION	:

namespace GL_ELL
{
    const qreal MIN_SEMIAXES_RATIO_DEFAULT = 0.1; //minimum ratio "small semiaxis / large semiaxis"
}

class GLEllipse
{
    friend QDataStream& operator<<(QDataStream& stream, const GLEllipse& data);
    friend QDataStream& operator>>(QDataStream& stream, GLEllipse& data);

public:
    // PACKAGE		:	GL
    // FUNCTION		:	GLEllipse::GLEllipse
    // DESCRIPTION	:	Constructor.
    // INPUTS 		:	void

    GLEllipse( void );

    // PACKAGE		:	GL
    // FUNCTION		:	GLEllipse::clear()
    // DESCRIPTION	:	Clear data.
    // INPUTS 		:	void

    void clear( void );		//Clear all elements .

    // PACKAGE		:	GL
    // FUNCTION		:	GLEllipse::operator =
    // DESCRIPTION	:	assign data.
    // INPUTS 		:	void

    GLEllipse& operator =( const GLEllipse& data );	// Data for assignment.

    // PACKAGE		:	GL
    // FUNCTION		:	GLEllipse::CheckEllipsesIntersection
    // DESCRIPTION	:	Checks whether current ellipse intersects with the another given ellipse
    // INPUTS 		:	Reference to the second ellipse
    // RETURNS      :   True if the current ellipse intersects with the second ellipse
    bool CheckEllipsesIntersection(GLEllipse &Ell2);

    // PACKAGE		:	GL
    // FUNCTION		:	GLEllipse::FormPlaneEllipse
    // DESCRIPTION	:	Forms ellipse on the plane using point and covariance matrix in ECEF coordinate system
    // INPUTS 		:	Point in ECEF coordinate system; covariance matrix in ECEF coordinate system;
    //              :   multiplicator for semiaxes
    // RETURNS      :   True if result is OK
    bool FormPlaneEllipse(GLPointDouble3D &P_ECEF, TMatrix<3> &Cov_ECEF, qreal Multiplicator);
    bool FormPlaneEllipse(GLPointDouble3D &P_ECEF, GLMatrix &Cov_ECEF, qreal Multiplicator);

    // PACKAGE		:	GL
    // FUNCTION		:	GLEllipse::CalcEllParamForCovMatr2x2
    // DESCRIPTION	:	Calculates ellipse parameters for given parameters of covariance matrix 2x2
    // INPUTS 		:	Dispersion on X coordinate; dispersion on Z coordinate; covariance moment XZ;
    //              :   reference to the resulting value of large semiaxis;
    //              :   reference to the resulting value of small semiaxis;
    //              :   reference to the resulting value of large semiaxis azimuth (radians)
    // RETURNS      :   True if result is OK
    bool CalcEllParamForCovMatr2x2(qreal Dx, qreal Dz, qreal Kxz,
                                   qreal &Res_a, qreal &Res_b, qreal &Res_angle);

    // PACKAGE		:	GL
    // FUNCTION		:	GLEllipse::RestrictionSemiaxes
    // DESCRIPTION	:	Restriction of semiaxes by given maximum admissible value
    // INPUTS 		:	Maximum admissible value of semiaxis
    // RETURNS      :   None
    void RestrictionSemiaxes(qreal MaxSemiaxis);

    // PACKAGE		:	GL
    // FUNCTION		:	GLEllipse::SetMinSemiaxesRatio
    // DESCRIPTION	:	Sets new value of minimum ratio of semiaxes
    // INPUTS 		:	New value of minimum ratio of semiaxes
    // RETURNS      :   None
    void SetMinSemiaxesRatio(qreal _MinSemiaxesRatio);

    // PACKAGE		:	GL
    // FUNCTION		:	GLEllipse::CalcEllipse
    // DESCRIPTION	:	Calculates dispersion ellipse using oblique projection along velocity vector
    // INPUTS 		:	Covariance matrix in the impact (or launch) point in the geocentric (ECEF) coordinate system;
    //              :   coordinates of impact (or launch) point in ECEF;
    //              :   velocity vector in the impact (or launch) point in ECEF;
    //              :   reference to the resulting value of large semiaxis (without multiplicator 3);
    //              :   reference to the resulting value of small semiaxis (without multiplicator 3);
    //              :   reference to the resulting value of large semiaxis azimuth (radians);
    //              :   sign of correction of semiaxes ratio
    // RETURNS      :   True if result is OK
    bool CalcEllipse(TMatrix<3> &CovMatr, const GLPointDouble3D &Point, const GLPointDouble3D &Vel,
                     qreal &aI, qreal &bI, qreal &betaI, bool bCorrRatio=false);
    bool CalcEllipse(TMatrix<6> &CovMatr, const GLPointDouble3D &Point, const GLPointDouble3D &Vel,
                     qreal &aI, qreal &bI, qreal &betaI, bool bCorrRatio=false);
    bool CalcEllipse(GLMatrix &CovMatr, const GLPointDouble3D &Point, const GLPointDouble3D &Vel,
                     qreal &aI, qreal &bI, qreal &betaI, bool bCorrRatio=false);

    GLPointDouble3D	Center;                 // Center
    qint64          time;                   // Time (msec)
    qreal           a;                      // Large semiaxis, m
    qreal           b;                      // Small semiaxis, m
    qreal           angle;                  // Azimuth of large semiaxis, radians
    qreal           min_semiaxes_ratio;     //minimum value of ratio "small semiaxis divided by large semiaxis"
};

#endif // GLELLIPSE_H

