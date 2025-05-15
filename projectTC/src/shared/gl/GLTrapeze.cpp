#include "GLTrapeze.h"

#include "conversion/Geocentric_Geo.h"
#include "conversion/Geocentric_Topo.h"
#include "conversion/Spherical_Topo.h"

using namespace GLFileLog;

GLTrapeze::GLTrapeze()
{
//    Reset();
}


GLTrapeze::GLTrapeze(const GLTrapeze &Q)
{
    this->P_base        = Q.P_base;
    this->AzFly         = Q.AzFly;
    this->SigmaAzFly    = Q.SigmaAzFly;
    this->SStrAzFly     = Q.SStrAzFly;
    this->MinRange      = Q.MinRange;
    this->MaxRange      = Q.MaxRange;
}


void GLTrapeze::Reset()
{
    P_base.clear();
    AzFly = 0;
    SigmaAzFly = 0;
    SStrAzFly = 0;
    MinRange = 0;
    MaxRange = 0;
}


GLTrapeze& GLTrapeze::operator =(const GLTrapeze &Q)
{
    this->P_base        = Q.P_base;
    this->AzFly         = Q.AzFly;
    this->SigmaAzFly    = Q.SigmaAzFly;
    this->SStrAzFly     = Q.SStrAzFly;
    this->MinRange      = Q.MinRange;
    this->MaxRange      = Q.MaxRange;
    return *this;
}


bool GLTrapeze::IntersectsWithCircle(GLPointDouble3D &Center, qreal R)
{
    bool bIntersection = false; //true if trapeze intersects with the circle
    bool bOK1 = true, bOK2 = true;

    //recalculation of base point to the geodesic coordinate system
    CGeocentric Base_gc(P_base.x, P_base.y, P_base.z); //base point in ECEF
    CGeodesic Base_gd; //base point in the Geodesic coordinate system
    GEOCENTRIC_GEO(&Base_gc, &Base_gd); //recalculation to the Geodesic CS

    //recalculation of Center to the topocentric coordinate system with the center in Base_gd
    CGeocentric Center_gc(Center.x, Center.y, Center.z); //Center in ECEF
    CTopocentric Center_tp; //Center in Topocentric CS
    GEOCENTRIC_TOPO(&Base_gd, &Center_gc, &Center_tp); //recalculation to the Topocentric CS

    CSpherical Center_sph; //Center in Spherical CS
    TOPO_SPHERICAL(&Center_tp, &Center_sph); //recalculation to the Spherical CS

    qreal d_b_c = Center_sph.m_dR * cos(Center_sph.m_dE); //distance from the base point to the center

    qreal AzRay1 = SB_2PI(AzFly * con_pi_div_180, SStrAzFly * con_pi_div_180); //azimuth of 1st ray of trapeze
    qreal AzRay2 = AB_2PI(AzFly * con_pi_div_180, SStrAzFly * con_pi_div_180); //azimuth of 2nd ray of trapeze

    if (d_b_c < R) //base point is within the circle
    {
        if (MinRange < R+R)
        {       //check for intersections where base point is within the circle
                //and minimum range is less than diameter
            qreal MinDist1 = -1, MaxDist1 = -1, MinDist2 = -1, MaxDist2 = -1;
            bOK1 = CalcDistanceToCircleInDir(AzRay1, d_b_c, Center_sph.m_dB, R, MinDist1, MaxDist1);
            bOK2 = CalcDistanceToCircleInDir(AzRay2, d_b_c, Center_sph.m_dB, R, MinDist2, MaxDist2);
            if (bOK1 && bOK2)
            {
                if (MinDist1 > MinRange || MinDist2 > MinRange)
                {
                    bIntersection = true;
                }
            }
        }
    }
    else //base point is outside the circle
    {
        qreal Alpha = asin(R / d_b_c); //angle between azimuth of center and tangent line to the circle drawn from (0;0)

        qreal AzTangent1 = SB_2PI(Center_sph.m_dB, Alpha); //azimuth of 1st tangent
        qreal AzTangent2 = AB_2PI(Center_sph.m_dB, Alpha); //azimuth of 2nd tangent

            //sings "Tangent i is located in sector between ray1 and ray2"
        bool bTangent1_in_sector = (SB_2PI(AzTangent1, AzRay1) >= 0 && (SB_2PI(AzRay2, AzTangent1) >= 0) ? true : false);
        bool bTangent2_in_sector = (SB_2PI(AzTangent2, AzRay1) >= 0 && (SB_2PI(AzRay2, AzTangent2) >= 0) ? true : false);

        if ( bTangent1_in_sector  //if 1st tangent is located between ray1 and ray2
             || bTangent2_in_sector ) //or if 2nd tangent is located between ray1 and ray2
        {
            if (bTangent1_in_sector && !bTangent2_in_sector)
            {
                //tangent1 is within the sector, tangent2 is without the sector, i.e. it is needed check for ray2
                qreal MinDist2 = -1, MaxDist2 = -1;
                bOK2 = CalcDistanceToCircleInDir(AzRay2, d_b_c, Center_sph.m_dB, R, MinDist2, MaxDist2);
                if (bOK2 && MaxDist2 > MinRange && MinDist2 < MaxRange)
                {
                    bIntersection = true;
                }
            }
            else if (!bTangent1_in_sector && bTangent2_in_sector)
            {
                //tangent2 is within the sector, tangent1 is without the sector, i.e. it is needed check for ray1
                qreal MinDist1 = -1, MaxDist1 = -1;
                bOK1 = CalcDistanceToCircleInDir(AzRay1, d_b_c, Center_sph.m_dB, R, MinDist1, MaxDist1);
                if (bOK1 && MaxDist1 > MinRange && MinDist1 < MaxRange)
                {
                    bIntersection = true;
                }
            }
            else //tangent1 is within the sector and tangent2 is within the sector
            {
                if (Center_sph.m_dR + R >= MinRange && Center_sph.m_dR - R <= MaxRange) //comparison with minimum and maximum ranges
                {
                    bIntersection = true;
                }
            }
        }
    }

    return bIntersection;
}


bool GLTrapeze::IsEmpty()
{
    bool bEmpty = false; //true if data is empty

    if (P_base.IsZero() || SStrAzFly < con_par_eps || MaxRange < con_par_eps )
    {
        bEmpty = true;
    }

    return bEmpty;
}


void GLTrapeze::LogData(FILE *_pLogFile, const char *_Name)
{
    if (_pLogFile != NULL)
    {
        tLog(_pLogFile, "%s %f %f %f "
             "%f %f %f %f %f ",
             _Name, P_base.x, P_base.y, P_base.z,
             AzFly, SigmaAzFly, SStrAzFly, MinRange, MaxRange);
    }
}


bool GLTrapeze::CalcDistanceToCircleInDir(qreal Az, qreal Dc, qreal Az_c, qreal R, qreal &MinDist, qreal &MaxDist)
{
    bool bRes = true;
    MinDist = -1;
    MaxDist = -1;

    qreal Alpha = SMODB_2PI(Az, Az_c); //angle between given direction and direction to the center of circle, from (0;0)
    qreal Discr = sqr(R) - sqr(Dc * sin(Alpha)); //discriminant
    if (Discr >= 0)
    {
        qreal sqrtDiscr = sqrt(Discr); //square root of discriminant
        qreal L1 = Dc * cos(Alpha) - sqrtDiscr; //solution of equation L^2-2*Dc*L*cos(Alpha)+Dc^2-R^2=0
        qreal L2 = Dc * cos(Alpha) + sqrt(Discr);

        if (Dc < R) //(0;0) is within the circle
        {
            qreal xC  = Dc*cos(Az_c);
            qreal zC  = Dc*sin(Az_c); //coordinates of center of the circle

            qreal xL1 = L1*cos(Az);
            qreal zL1 = L1*sin(Az); //vector with magnitude L1 in direction determined by Az from (0;0)

            qreal DistCL1 = sqrt(sqr(xL1-xC) + sqr(zL1-zC)); //distance from the center of circle to the point (zL1;xL1)

            if (fabs(DistCL1 - R) < con_par_eps) //point (zL1;xL1) belong to the circle
            {
                MinDist = L1;
                MaxDist = L1;
            }
            else
            {
                qreal xL2 = L2*cos(Az);
                qreal zL2 = L2*sin(Az); //vector with magnitude L1 in direction determined by Az from (0;0)

                qreal DistCL2 = sqrt(sqr(xL2-xC) + sqr(zL2-zC)); //distance from the center of circle to the point (zL2;xL2)

                if (fabs(DistCL2 - R) < con_par_eps) //point (zL2;xL2) belong to the circle
                {
                    MinDist = L2;
                    MaxDist = L2;
                }
                else
                {
                    bRes = false;
                }
            }
        }
        else //(0;0) is outside the circle
        {
            MinDist = L1;
            MaxDist = L2;
        }
    }
    else
    {
        bRes = false;
    }

    return bRes;
}
