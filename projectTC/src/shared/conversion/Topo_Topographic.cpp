// PACKAGE		: Conversion
// FILE         :   Topo_Topographic.h
//
// AUTHOR	: Lapushynski Aliaksandr, 2015
// DESCRIPTION :Implementation file for function of recalculation topographic coordinates system and topocentric
//                   coordinates system

#include <cmath>
#include "Topo_Topographic.h"
#include "ConstRecalc.h"

// PACKAGE		: Conversion
// FUNCTION        : RecountTOPOCENTRICtoTOPOGRAPHIC_coord
//
// DESCRIPTION     : Function of recalculation from topocentric to topographic
//
// INPUTS          : TPC - topocentric coordinates;
//
// RETURNS	   : TPG - topographic coordinates;
//
//
bool RecountTOPOCENTRICtoTOPOGRAPHIC_coord(const CTopocentric *TPC, CTopographic* TPG)
{
    bool F = false;

    {
        double Epsilon, r, rXZ, H, D, Alpha;

        rXZ = sqrt(TPC->m_dXt*TPC->m_dXt + TPC->m_dZt*TPC->m_dZt);
        if( rXZ == 0)
        {
            return false;
        }
        Epsilon = atan( TPC->m_dYt / rXZ );

        r = sqrt(pow(TPC->m_dXt, 2) + pow(TPC->m_dYt, 2) + pow(TPC->m_dZt, 2));

        H = sqrt(pow(cdMainRadius, 2) + pow(r, 2) + 2 * cdMainRadius * r * sin(Epsilon)) - cdMainRadius;

        D = cdMainRadius * asin( r * cos(Epsilon) /(cdMainRadius + H));

        Alpha = acos( TPC->m_dXt / rXZ );

        if (TPC->m_dZt < 0)
        {
            Alpha = 2*cdPi - Alpha;
        }

        TPG->m_dXt = D*cos(Alpha);
        TPG->m_dZt = D*sin(Alpha);
        TPG->m_dHt = H;

        F = true;
    }

    return F;
}

// PACKAGE		: Conversion
// FUNCTION        : RecountTOPOCENTRICtoTOPOGRAPHIC_coord
//
// DESCRIPTION     : Function of recalculation from topocentric to topographic
//
// INPUTS          : TPC - topocentric coordinates, TPC_vel TPC - topocentric velocity;
//
// RETURNS	   : TPG - topographic coordinates, TPG - topographic velocity;
//
//

bool RecountTOPOCENTRICtoTOPOGRAPHIC_coord_vel(const CTopocentric *TPC, const CTopocentric *TPC_vel,
                                               CTopographic* TPG, CTopographic* TPG_vel)
{
    if((!TPC) ||
            (!TPC_vel) ||
            (!TPG) || (!TPG_vel) ||
            (TPC->m_dXt < cdTOPO_X_Y_Z_MIN) ||
            (TPC->m_dXt > cdTOPO_X_Y_Z_MAX) ||
            (TPC->m_dYt < cdTOPO_X_Y_Z_MIN) ||
            (TPC->m_dYt > cdTOPO_X_Y_Z_MAX) ||
            (TPC->m_dZt < cdTOPO_X_Y_Z_MIN) ||
            (TPC->m_dZt > cdTOPO_X_Y_Z_MAX))
    {
        return false;
    }
    double R=cdMainRadius;
    double x=TPC->m_dXt;
    double y=TPC->m_dYt;
    double z=TPC->m_dZt;
    double vx=TPC_vel->m_dXt;
    double vy=TPC_vel->m_dYt;
    double vz=TPC_vel->m_dZt;

    if(sqrt(pow(x,2)+pow(z,2))==0)
    {
        return false;
    }
    if(sqrt(pow(x,2)+pow(y,2)+pow(z,2))==0)
    {
        return false;
    }

    double sqrtXZ=sqrt(pow(x,2)+pow(z,2));
    double sqrtXYZ=sqrt(pow(x,2)+pow(y,2)+pow(z,2));
    double scal_coord_v=x*vx+y*vy+z*vz;
    double e=atan(y/sqrtXZ);
    double de=(vy*pow(sqrtXZ,2)-y*(x*vx+z*vz))/(sqrtXZ*pow(sqrtXYZ,2));
    double H=sqrt(pow(R,2)+pow(sqrtXYZ,2)+2*R*sqrtXYZ*sin(e))-R;
    double Vh=(scal_coord_v*(1+R*sin(e)/sqrtXYZ)+R*sqrtXYZ*cos(e)*de)/(R+H);
    double D;

    if ( y >= -R)
    {
        D=R*asin(sqrtXYZ*cos(e)/(R+H));
    }
    else
    {
        D=R*(cdPi-asin(sqrtXYZ*cos(e)/(R+H)));
    }

    double dD=R/sqrt(pow((R+H),2)-pow(sqrtXYZ,2)*pow(cos(e),2))*(scal_coord_v*cos(e)/sqrtXYZ-
                                                                 sqrtXYZ*(sin(e)*de+cos(e)*Vh/(R+H)));
    double a=acos(x/sqrtXZ);

    if( z < 0)
    {
        a=2*cdPi-a;
    }

    double dcosa=z*(z*vx-x*vz)/pow(sqrtXZ,3);
    double dsina=x*(x*vz-z*vx)/pow(sqrtXZ,3);
    TPG->m_dXt=D*cos(a);
    TPG->m_dZt=D*sin(a);
    TPG->m_dHt=H;
    TPG_vel->m_dXt=dD*cos(a)+D*dcosa;
    TPG_vel->m_dZt=dD*sin(a)+D*dsina;
    TPG_vel->m_dHt=Vh;
    return true;
}

// PACKAGE		: Conversion
// FUNCTION        : RecountTOPOCENTRICtoTOPOGRAPHIC_coord
//
// DESCRIPTION     : Function of recalculation from topographic to topocentric
//
// INPUTS          : TPG - topographic coordinates;
//
// RETURNS	   : TPC - topocentric coordinates;
//
//
//
bool RecountTOPOGRAPHICtoTOPOCENTRIC_coord(const CTopographic* TPG, CTopocentric* TPC)
{
    if((!TPG) ||
            (!TPC) ||
            (TPG->m_dXt < cdTOPO_X_Y_Z_MIN) ||
            (TPG->m_dXt > cdTOPO_X_Y_Z_MAX) ||
            (TPG->m_dHt < cdTOPO_X_Y_Z_MIN) ||
            (TPG->m_dHt > cdTOPO_X_Y_Z_MAX) ||
            (TPG->m_dZt < cdTOPO_X_Y_Z_MIN) ||
            (TPG->m_dZt > cdTOPO_X_Y_Z_MAX))
    {
        return false;
    }
    bool F = false;

    {
        double Betha, D, Alpha, Y, rXZ;

        rXZ = sqrt(pow(TPG->m_dXt, 2) + pow(TPG->m_dZt, 2));

        if( rXZ == 0)
        {
            return false;
        }

        Betha = rXZ / cdMainRadius;
        if(cos(Betha)==0)
        {
            return false;
        }

        D = (cdMainRadius + TPG->m_dHt) * sin(Betha);

        if(Betha<cdPi/2)
        {
            Y=(cdMainRadius+TPG->m_dHt)*cos(Betha)-cdMainRadius    ;
        }
        else
        {
            Y=-2*cdMainRadius+(cdMainRadius-(cdMainRadius+TPG->m_dHt)*cos(cdPi-Betha));
        }
        //Y = sqrt( pow(cdMainRadius, 2) + pow((cdMainRadius + TPG->m_dHt)*cos(Betha), 2) - 2*cdMainRadius*(cdMainRadius + TPG->m_dHt)*cos(Betha));

        //        if ( TPG->m_dHt <= cdMainRadius * (1/cos(Betha) - 1) )
        //            Y = -Y;

        Alpha = acos( TPG->m_dXt / rXZ);

        if ( TPG->m_dZt < 0 )
        {
            Alpha = 2*cdPi - Alpha;
        }

        TPC->m_dXt = D*cos(Alpha);
        TPC->m_dYt = Y;
        TPC->m_dZt = D*sin(Alpha);

        F = true;
    }

    return F;
}


// PACKAGE		: Conversion
// FUNCTION        : RecountTOPOCENTRICtoTOPOGRAPHIC_coord
//
// DESCRIPTION     : Function of recalculation from topographic to topocentric
//
// INPUTS          : TPG - topographic coordinates,TPG_vel - topographic velocity;
//
// RETURNS	   : TPC - topocentric coordinates,TPC_vel - topocentric velocity;
//
//
//
bool RecountTOPOGRAPHICtoTOPOCENTRIC_coord_vel(const CTopographic *TPG, const CTopographic *TPG_vel, CTopocentric* TPC, CTopocentric* TPC_vel)
{
    if((!TPG) ||
            (!TPC) || (!TPG_vel) ||
            (!TPC_vel) ||
            (TPG->m_dXt < cdTOPO_X_Y_Z_MIN) ||
            (TPG->m_dXt > cdTOPO_X_Y_Z_MAX) ||
            (TPG->m_dHt < cdTOPO_X_Y_Z_MIN) ||
            (TPG->m_dHt > cdTOPO_X_Y_Z_MAX) ||
            (TPG->m_dZt < cdTOPO_X_Y_Z_MIN) ||
            (TPG->m_dZt > cdTOPO_X_Y_Z_MAX))
    {
        return false;
    }
    double R=cdMainRadius;
    double x=TPG->m_dXt;
    double z=TPG->m_dZt;
    double H=TPG->m_dHt;
    double vx=TPG_vel->m_dXt;
    double vz=TPG_vel->m_dZt;
    double vh=TPG_vel->m_dHt;

    if(sqrt(pow(x,2)+pow(z,2))==0)
    {
        return false;
    }

    double b=sqrt(pow(x,2)+pow(z,2))/R;
    double db=(vx*x+vz*z)/(R*sqrt(pow(x,2)+pow(z,2)));
    double D=(R+H)*sin(b);
    double dD=(R+H)*cos(b)*db+vh*sin(b);
    double Ytpsc;
    if(b<cdPi/2)
    {
        Ytpsc=(R+H)*cos(b)-R;
    }
    else
    {
        Ytpsc=-2*R+(R-(R+H)*cos(cdPi-b));
    }
    //Ytpsc = sqrt(pow(R,2)+pow((R+H)*cos(b), 2)-2*R*(R+H)*cos(b));

    if(Ytpsc==0)
    {
        return false;
    }
    double Vytpsc=1/Ytpsc*((R+H)*vh*pow(cos(b),2)-pow(R+H,2)*sin(b)*cos(b)*db-
                           R*vh*cos(b)+R*(R+H)*sin(b)*db);
    //    if ( H <= R * (1/cos(b) - 1) )
    //    {
    //        Ytpsc=-Ytpsc;
    //        Vytpsc=-Vytpsc;
    //    }
    double a;

    if( z >= 0)
    {
        a=acos(x/sqrt(pow(x,2)+pow(z,2)));
    }
    if( z < 0)
    {
        a=2*cdPi-acos(x/sqrt(pow(x,2)+pow(z,2)));
    }

    double dcosa=(vx*pow(z,2)-vz*x*z)/pow((sqrt(pow(x,2)+pow(z,2))),3);
    double dsina=(vz*pow(x,2)-vx*x*z)/pow((sqrt(pow(x,2)+pow(z,2))),3);
    TPC->m_dXt=D*cos(a);
    TPC->m_dZt=D*sin(a);
    TPC->m_dYt=Ytpsc;
    TPC_vel->m_dXt=dD*cos(a)+D*dcosa;
    TPC_vel->m_dZt=dD*sin(a)+D*dsina;
    TPC_vel->m_dYt=Vytpsc;

    return true;
}
