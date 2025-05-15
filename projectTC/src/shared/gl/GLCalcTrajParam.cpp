#include "GLCalcTrajParam.h"

#include "Constants.h"
#include "GLConvertMatrices.h"
#include "conversion/Geocentric_Geo.h"
#include "GLRandlib.h"

//*********************************************************************
//
//            GLCalcTrajParam implementation
//
//**********************************************************************

GLCalcTrajParam::GLCalcTrajParam()
{
    Reset();
}


void GLCalcTrajParam::Reset()
{
    IsInitialized = false;

    r = 0;
    S = 0;
    Vr = 0;
    V = 0;
    A = 0;
}


void GLCalcTrajParam::Initialize(double &X, double &Y, double &Z, double &VX, double &VY, double &VZ, double &AX, double &AY, double &AZ)
{
    IsInitialized = true;

    r = CalcVectMagnitude(X, Y, Z);

    V = CalcVectMagnitude(VX, VY, VZ);

    A = CalcVectMagnitude(AX, AY, AZ);

    CalcS(Z, r);

    CalcVr(X, Y, Z, VX, VY, VZ, r);
}


void GLCalcTrajParam::Initialize(GLVector &PosECEF)
{
    Initialize(PosECEF.Vec[0], PosECEF.Vec[1], PosECEF.Vec[2],
               PosECEF.Vec[3], PosECEF.Vec[4], PosECEF.Vec[5],
               PosECEF.Vec[6], PosECEF.Vec[7], PosECEF.Vec[8]);
}


double GLCalcTrajParam::CalcVectMagnitude(const double &X, const double &Y, const double &Z)
{
    return sqrt(X*X + Y*Y + Z*Z);
}


double GLCalcTrajParam::CalcHeight(const double &X, const double &Y, const double &Z)
{
    double H = 0;

    if (!IsInitialized)
        r = CalcVectMagnitude(X, Y, Z);

    if (fabs(r) > con_par_eps)
    {
        double Radicand = pow(1. - con_Eath_compress_coeff ,2)
                             + (2.*con_Eath_compress_coeff - pow(con_Eath_compress_coeff, 2))*Z*Z/(r*r);
        if (Radicand >= 0)
        {
            H = r - con_b_half_axis * (1. - con_Eath_compress_coeff) / sqrt(Radicand);
        }
    }
    return H;
}


double GLCalcTrajParam::CalcHeight_NUE(double &X, double &Y, double &Z, double &Hsrc)
{
    return sqrt(sqr(X) + sqr(Y + con_Eath_middle_radius + Hsrc) + sqr(Z)) - con_Eath_middle_radius;
}


double GLCalcTrajParam::CalcVertVel(double &X, double &Y, double &Z, double &VX, double &VY, double &VZ)
{
    double Vh = 0;

    if (!IsInitialized)
    {
        r = CalcVectMagnitude(X, Y, Z);
        CalcS(Z, r);
        CalcVr(X, Y, Z, VX, VY, VZ, r);
        V = CalcAbsVelocity(VX, VY, VZ);
    }

    if (fabs(r) > con_par_eps)
    {
        Vh = Vr + con_b_half_axis * S * (Z/(r*r)) * (VZ - Vr*Z/r);
        if (fabs(Vh) > V)
        {
            Vh = Vr;
            if (fabs(Vh) > V)
            {
                double signVh = (Vh > 0) ? 1. : ((Vh < 0) ? -1. : 0.);
                Vh = signVh * V;
            }
        }
    }
    return Vh;
}


double GLCalcTrajParam::CalcVertAccel(double &X, double &Y, double &Z, double &AX, double &AY, double &AZ)
{
    double Ah = 0;

    if (!IsInitialized)
    {
        r = CalcVectMagnitude(X, Y, Z);
    }

    if (fabs(r) > con_par_eps)
    {
        Ah = (X*AX + Y*AY + Z*AZ)/r;
    }
    return Ah;
}


double GLCalcTrajParam::CalcLongitudinalAccel(double &VX, double &VY, double &VZ, double &AX, double &AY, double &AZ)
{
    double Al = 0;

    if (!IsInitialized)
    {
        V = CalcAbsVelocity(VX, VY, VZ);
    }

    if (fabs(V) > con_par_eps)
    {
        Al = (AX*VX + AY*VY + AZ*VZ) / V;
    }
    return Al;
}


double GLCalcTrajParam::CalcLongitudinalAccel(GLPointDouble3D &Vel, GLPointDouble3D &Acc)
{
    return CalcLongitudinalAccel(Vel.x, Vel.y, Vel.z, Acc.x, Acc.y, Acc.z);
}


double GLCalcTrajParam::CalcVertAccelWithoutCoriolis(double &X, double &Y, double &Z, double &VX, double &VY, double &AX, double &AY, double &AZ)
{
    double AhNoC = 0;
    if (!IsInitialized)
    {
        r = CalcVectMagnitude(X, Y, Z);
    }

    if (fabs(r) > con_par_eps)
    {
        AhNoC = (X*(AX - 2*con_Eath_angle_speed*VY) + Y*(AY + 2*con_Eath_angle_speed*VX) + Z*AZ)/r;
    }
    return AhNoC;
}


double GLCalcTrajParam::CalcHorizVel(double &X, double &Y, double &Z, double &VX, double &VY, double &VZ)
{
    double Vh=0, Vhor=0;

    Vh = CalcVertVel(X, Y, Z, VX, VY, VZ);

    if (!IsInitialized)
    {
        V = CalcVectMagnitude(VX, VY, VZ);
    }
    double Radicand = V*V - Vh*Vh;
    if (Radicand >= 0)
    {
        Vhor = sqrt(Radicand);
    }
    return Vhor;
}


double GLCalcTrajParam::CalcAngleVelHorizont(double Vh, double V_absolute)
{
    double Angle = 0.;
    if (V_absolute > con_par_eps)
    {
        double sin_Angle = Vh / V_absolute;
        if (fabs(sin_Angle) <= 1.)
        {
            Angle = asin(sin_Angle);
        }
    }
    return Angle;
}


double GLCalcTrajParam::CalcAngleVelHorizont(double &X, double &Y, double &Z, double &VX, double &VY, double &VZ)
{
    if (!IsInitialized)
        V = CalcVectMagnitude(VX, VY, VZ);
    double Vh = CalcVertVel(X, Y, Z, VX, VY, VZ);
    double Angle = CalcAngleVelHorizont(Vh, V);
    return Angle;
}


double GLCalcTrajParam::CalcEnHeight_EarthCenter(double &X, double &Y, double &Z, double &V)
{
    double He = 0;

    if (!IsInitialized)
        r = CalcVectMagnitude(X, Y, Z);

    if (fabs(r) > 0)
    {
        He = (con_Grav_Const * con_Earth_Mass /
                (con_Grav_Const*con_Earth_Mass/r - V*V/2.)) - con_Eath_middle_radius;
    }
    return He;
}


double GLCalcTrajParam::CalcEnHeight_EarthCenter(GLVector &PosECEF)
{
    if (!IsInitialized)
        V = CalcVectMagnitude(PosECEF.Vec[3], PosECEF.Vec[4], PosECEF.Vec[5]);

    return CalcEnHeight_EarthCenter(PosECEF.Vec[0], PosECEF.Vec[1], PosECEF.Vec[2], V);
}


double GLCalcTrajParam::CalcEnHeight_EarthCenter(GLPointDouble3D &PosECEF, GLPointDouble3D &VelECEF)
{
    if (!IsInitialized)
        V = CalcVectMagnitude(VelECEF);

    return CalcEnHeight_EarthCenter(PosECEF.x, PosECEF.y, PosECEF.z, V);
}


double GLCalcTrajParam::CalcEnHeight_EarthSurface(double &V, double &H)
{
    return V*V/(2.*con_g) + H;
}


double GLCalcTrajParam::CalcEnHeight_EarthSurface(GLVector &PosECEF)
{
    if (!IsInitialized)
        V = CalcVectMagnitude(PosECEF.Vec[3], PosECEF.Vec[4], PosECEF.Vec[5]);

    double H = CalcHeight(PosECEF);

    return CalcEnHeight_EarthSurface(V, H) ;
}


double GLCalcTrajParam::CalcEnHeight_EarthSurface(GLPointDouble3D &PosECEF, GLPointDouble3D &VelECEF)
{
    if (!IsInitialized)
        V = CalcVectMagnitude(VelECEF);

    double H = CalcHeight(PosECEF);

    return CalcEnHeight_EarthSurface(V, H);
}


double GLCalcTrajParam::CalcVelEnHeight_EarthSurface(double &VX, double &VY, double &VZ, double &AX, double &AY, double &AZ, double &VH)
{
    return (AX*VX + AY*VY + AZ*VZ)/con_g + VH;
}


double GLCalcTrajParam::CalcVelEnHeight_EarthSurface(GLVector &PosECEF)
{
    double VH = CalcVertVel(PosECEF);

    return CalcVelEnHeight_EarthSurface(PosECEF.Vec[3], PosECEF.Vec[4], PosECEF.Vec[5],
            PosECEF.Vec[6], PosECEF.Vec[7], PosECEF.Vec[8], VH);
}


double GLCalcTrajParam::CalcVelEnHeight_EarthSurface(GLPointDouble3D &PosECEF, GLPointDouble3D &VelECEF, GLPointDouble3D &AccECEF)
{
    double VH = CalcVertVel(PosECEF, VelECEF);

    return CalcVelEnHeight_EarthSurface(VelECEF.x, VelECEF.y, VelECEF.z, AccECEF.x, AccECEF.y, AccECEF.z, VH);
}


double GLCalcTrajParam::CalcDistance(double &X1, double &Y1, double &Z1, double &X2, double &Y2, double &Z2)
{
    double d = sqrt((X1-X2)*(X1-X2) + (Y1-Y2)*(Y1-Y2) + (Z1-Z2)*(Z1-Z2));

    return d;
}


double GLCalcTrajParam::CalcDistanceFromPointToVector(double PX, double PY, double PZ, double VX, double VY, double VZ)
{
    double Distance = 0;
    double mod2Vect = VX*VX + VY*VY + VZ*VZ;
    if (mod2Vect > con_par_eps)
    {
        double PointScalarVect = PX*VX + PY*VY + PZ*VZ;
        double Lambda = PointScalarVect / mod2Vect;
        Distance = sqrt(sqr(PX - Lambda*VX) + sqr(PY - Lambda*VY) + sqr(PZ - Lambda*VZ));
    }
    else
    {
        Distance = sqrt(PX*PX + PY*PY + PZ*PZ);
    }

    return Distance;
}


double GLCalcTrajParam::CalcDistanceFromPointToVector(GLPointDouble3D &P, GLPointDouble3D &V)
{
    return CalcDistanceFromPointToVector(P.x, P.y, P.z, V.x, V.y, V.z);
}


double GLCalcTrajParam::CalcMahalanobisDistance(GLVector &V1, GLVector &V2, GLMatrix &K)
{
    double Result = -1;
    if (V1.s > 0 && V1.s == V2.s && V1.s == K.s_m && K.s_m == K.s_n)
    {
        qint32 dim = V1.s;
        bool bOK = true, bResOK = true;
        CMatrix fMatr;
        GLVector DeltaVec;
        GLMatrix Delta(dim,1), DeltaT(1,dim), K_inv(dim, dim), d_Matr(1,1);

        bOK = fMatr.InvMatrixSymmPos(K, K_inv); bResOK = bResOK && bOK;
        if (bOK)
        {
            bOK = fMatr.Subtr(V1, V2, DeltaVec); bResOK = bResOK && bOK; //subtraction of vectors
            bOK = fMatr.Matching(DeltaVec, Delta); bResOK = bResOK && bOK; //Delta is DeltaVec in the matrix representation
            bOK = fMatr.Transpon(Delta, DeltaT); bResOK = bResOK && bOK; //DeltaT is Delta transposed

            bOK = fMatr.MatrXMatrXMatr(DeltaT, K_inv, Delta, d_Matr); bResOK = bResOK && bOK; //d_Matr = DeltaT*K_inv*Delta
            if (bResOK && d_Matr.M[0][0] >= 0)
            {
                Result = sqrt(d_Matr.M[0][0]);
            }
        }
    }
    return Result;
}


double GLCalcTrajParam::CalcMahalanobisDistance(GLPointDouble3D &P1, GLPointDouble3D &P2, TMatrix<SIZE_M> &K)
{
    double Result = -1;
    if (K.s_m == 3 && K.s_n == 3)
    {
        GLVector V1(3), V2(3);
        V1.Vec[0] = P1.x;
        V1.Vec[1] = P1.y;
        V1.Vec[2] = P1.z;
        V2.Vec[0] = P2.x;
        V2.Vec[1] = P2.y;
        V2.Vec[2] = P2.z;
        Result = CalcMahalanobisDistance(V1, V2, K);
    }
    return Result;
}


double GLCalcTrajParam::CalcAngle2Vect(double &X1, double &Y1, double &Z1, double &X2, double &Y2, double &Z2)
{
    double Result = 0;
    double Denominator = CalcVectMagnitude(X1, Y1, Z1) * CalcVectMagnitude(X2, Y2, Z2);
    if (fabs(Denominator) > con_eps2)
    {
        double cos_Angle = (X1*X2 + Y1*Y2 + Z1*Z2) / Denominator;
        if (fabs(cos_Angle) <= 1.)
        {
            Result = acos(cos_Angle);
        }
    }
    return Result;
}


bool GLCalcTrajParam::CalcAerodynamicAccel(double X, double Y, double Z, double VX, double VY, double VZ, double AX, double AY, double Az,
                                           double &ModAccDrag, double &ModAccLift, double &ModAccLateral)
{
    bool bOK = true;
    ModAccDrag = 0;
    ModAccLift = 0;
    ModAccLateral = 0;

    double ModCoord = CalcVectMagnitude(X, Y, Z);
    GLPointDouble3D Vel(VX, VY, VZ); //velocity vector
    double ModVel = Vel.moduleFloat(); //absolute velocity
    if (ModCoord > con_eps2 && ModVel > con_eps2)
    {
        GLPointDouble3D CoordNorm(X/ModCoord, Y/ModCoord, Z/ModCoord); //normalized vector of coordinates
        GLPointDouble3D AccAerodyn(AX + con_g*CoordNorm.x, AY + con_g*CoordNorm.y, Az + con_g*CoordNorm.z); //common vector of aerodynamic accelerations = a_drag + a_lift + a_lateral = a - a_gravitational
                                                        //+g because (a-a_gravitational)=a-(-g*CoordNorm)
        ModAccDrag = fabs(AccAerodyn.ScalarProduct(Vel)) / ModVel;

        GLPointDouble3D NLift; //directional vector for lift acceleration
        NLift = CoordNorm * (ModVel*ModVel) - (Vel*(CoordNorm.ScalarProduct(Vel)));

        double ModNLift = NLift.moduleFloat();
        if (ModNLift > con_eps2)
        {
            ModAccLift = fabs(AccAerodyn.ScalarProduct(NLift)) / ModNLift;
            double Radicand = sqr(AccAerodyn.x) + sqr(AccAerodyn.y) + sqr(AccAerodyn.z) - sqr(ModAccDrag) - sqr(ModAccLift);
            if (Radicand >= 0)
            {
                ModAccLateral = sqrt(Radicand);
            }
        }
        else
        {
            bOK = false;
        }
    }
    else
    {
        bOK = false;
    }
    return bOK;
}


double GLCalcTrajParam::Calc_g(const double &_r)
{
    double Result = con_g;
    if (_r > con_eps2)
    {
        Result = con_Grav_Const * con_Earth_Mass / sqr(_r);
    }
    return Result;
}


double GLCalcTrajParam::Calc_g(GLPointDouble3D &_P)
{
    return Calc_g(_P.moduleFloat());
}


double GLCalcTrajParam::Calc_g_h(const double &_h)
{
    return Calc_g(_h + con_Eath_middle_radius);
}


bool GLCalcTrajParam::CalcRPergiee(GLPointDouble3D &CoordGC, GLPointDouble3D &VelGC, double &ResRp)
{
    bool bOK = true;
    ResRp = 0;
    GLPointDouble3D Eta(0., 0., 1);
    GLPointDouble3D r_vect, v_vect;
    r_vect = CoordGC;
    v_vect = VelGC + (Eta.VectorProduct(CoordGC) * con_Eath_angle_speed);

    double mod_r = r_vect.moduleFloat();
    double mod_v = v_vect.moduleFloat();
    if (mod_r > con_eps && mod_v > con_eps)
    {
        double sinTheta = r_vect.ScalarProduct(v_vect) / (mod_r * mod_v);
        double Gamma = mod_r * sqr(mod_v) / con_mu;
        double p = Gamma * mod_r * (1 - sqr(sinTheta));
        double e2 = sqr(1 - Gamma) + Gamma * (2. - Gamma) * sqr(sinTheta);
        if (e2 < 0)
        {
            bOK = false;
        }
        else
        {
            double e = sqrt(e2);
            if (fabs(1. + e) > con_par_eps)
            {
                ResRp = p / (1. + e);
            }
            else
            {
                bOK = false;
            }
        }
    }
    else
    {
        bOK = false;
    }
    return bOK;
}


bool GLCalcTrajParam::CalcAzVel(double X, double Y, double Z, double VX, double VY, double VZ, double &ResAzVel)
{
    bool bOK = true;
    ResAzVel = 0;
    if (fabs(X) > con_par_eps || fabs(Y) > con_par_eps || fabs(Z) > con_par_eps)
    {
        CGeocentric PointGC(X, Y, Z);
        CGeodesic PointGD;
        CGeocentric VelGC(VX, VY, VZ);
        CTopocentric VelTP;

        bOK = bOK && GEOCENTRIC_GEO(&PointGC, &PointGD);
        bOK = bOK && GEOCENTRIC_TOPO(&PointGD, 0, &VelGC, 0, 0, &VelTP, 0);
        bOK = bOK && TOPO__GET_AZIMUTH(&VelTP, &ResAzVel);
    }
    else
    {
        bOK = false;
    }
    return bOK;
}


bool GLCalcTrajParam::CalcAzVel(GLPointDouble3D &Coord, GLPointDouble3D &Vel, double &ResAzVel)
{
    return CalcAzVel(Coord.x, Coord.y, Coord.z, Vel.x, Vel.y, Vel.z, ResAzVel);
}


bool GLCalcTrajParam::GetVector_VertVel(GLPointDouble3D &P, GLPointDouble3D &V, GLPointDouble3D &ResVertV)
{
    bool bRes = true; //true if result is ok
    ResVertV.clear();

    if (P.IsZero())
    {
        bRes = false;
    }
    else
    {
        double VP = V.ScalarProduct(P);
        double PP = P.ScalarProduct(P);
        if (PP > con_eps2)
        {
            ResVertV = P * (VP/PP);
        }
    }
    return bRes;
}


bool GLCalcTrajParam::GetVector_HorizVel(GLPointDouble3D &P, GLPointDouble3D &V, GLPointDouble3D &ResHorizV)
{
    bool bRes = true; //true if result is ok
    ResHorizV.clear();

    if (P.IsZero())
    {
        bRes = false;
    }
    else
    {
        double VP = V.ScalarProduct(P);
        double PP = P.ScalarProduct(P);
        if (PP > con_eps2)
        {
            ResHorizV = V - (P * (VP/PP));
        }
    }
    return bRes;
}


double GLCalcTrajParam::CalcRMSE_VectMagnitude(double &X, double &Y, double &Z, GLMatrix &K)
{
    double SigmaMagn = 0;

    double r_curr = CalcVectMagnitude(X, Y, Z);
    bool ItsOK = true;

    if (r_curr > con_eps2)
    {
        GLMatrix J(1,3), JT(3,1), D_Matr(1,1), K3x3(3,3);
        CMatrix cl_Matr;

        J.M[0][0] = X / r_curr;
        J.M[0][1] = Y / r_curr;
        J.M[0][2] = Z / r_curr;

        ItsOK &= cl_Matr.Transpon(J, JT); //JT is J transposed
        bool ItsOK2 = true;
        K3x3 = K.PickOut(0,2,0,2, ItsOK2);
        if (ItsOK && ItsOK2)
        {
            ItsOK &= cl_Matr.MatrXMatrXMatr(J, K3x3, JT, D_Matr); //D_Matr = J*K*JT

            if (ItsOK && D_Matr.M[0][0] >= 0)
            {
                SigmaMagn = sqrt(D_Matr.M[0][0]);
            }
        }
    }
    return SigmaMagn;
}


double GLCalcTrajParam::CalcRMSE_AbsVelocity(double &VX, double &VY, double &VZ, GLMatrix &Kv)
{
    double SigmaV = 0;
    bool ItsOK = true;

    if (!IsInitialized)
    {
        V = CalcVectMagnitude(VX, VY, VZ);
    }
    if (V > con_eps2)
    {
        GLMatrix Jv(1,3), JvT(3,1), D_Matr(1,1);
        CMatrix cl_Matr;

        Jv.M[0][0] = VX / V;
        Jv.M[0][1] = VY / V;
        Jv.M[0][2] = VZ / V;

        ItsOK &= cl_Matr.Transpon(Jv, JvT); //JvT is Jv transposed

        ItsOK &= cl_Matr.MatrXMatrXMatr(Jv, Kv, JvT, D_Matr); //D_Matr = Jv*Kv*JvT

        if (ItsOK && D_Matr.M[0][0] >= 0)
        {
            SigmaV = sqrt(D_Matr.M[0][0]);
        }
    }
    return SigmaV;
}


double GLCalcTrajParam::CalcRMSE_Height(double &X, double &Y, double &Z, GLMatrix &Kc)
{
    if (!IsInitialized)
    {
        r = CalcVectMagnitude(X, Y, Z);
        CalcS(Z, r);
    }

    double SigmaH = 0;
    bool ItsOK = true;
    if (r > con_eps2)
    {
        GLMatrix Jh(1,3), JhT(3,1), D_Matr(1,1), Kc3x3(3,3);
        CMatrix cl_Matr;

        Jh.M[0][0] = X/r - con_b_half_axis*X*Z*Z*S / pow(r, 4);
        Jh.M[0][1] = Y/r - con_b_half_axis*Y*Z*Z*S / pow(r, 4);
        Jh.M[0][2] = Z/r + con_b_half_axis*Z*(r*r-Z*Z)*S / pow(r, 4);

        ItsOK &= cl_Matr.Transpon(Jh, JhT); //JhT is Jh transposed
        if (ItsOK)
        {
            Kc3x3 = Kc.PickOut(0,2,0,2, ItsOK);

            ItsOK &= cl_Matr.MatrXMatrXMatr(Jh, Kc3x3, JhT, D_Matr); //D_Matr = Jh*Kc*JhT

            if (ItsOK && D_Matr.M[0][0] >= 0)
            {
                SigmaH = sqrt(D_Matr.M[0][0]);
            }
        }
    }
    return SigmaH;
}


double GLCalcTrajParam::CalcRMSE_VertVel(double &X, double &Y, double &Z, double &VX, double &VY, double &VZ, GLMatrix &Kcv)
{
    if (!IsInitialized)
    {
        r = CalcVectMagnitude(X, Y, Z);
        CalcVr(X, Y, Z, VX, VY, VZ, r);
    }

    double SigmaVH = 0;
    bool ItsOK = true;
    if (r > con_eps2)
    {
        GLMatrix Jvh(1,6), JvhT(6,1), D_Matr(1,1), Kcv6x6(6,6);
        CMatrix cl_Matr;

        Jvh.M[0][0] = VX/r - X*Vr/(r*r);
        Jvh.M[0][1] = VY/r - Y*Vr/(r*r);
        Jvh.M[0][2] = VZ/r - Z*Vr/(r*r);
        Jvh.M[0][3] = X/r;
        Jvh.M[0][4] = Y/r;
        Jvh.M[0][5] = Z/r;

        Kcv6x6 = Kcv.PickOut(0,5,0,5, ItsOK);

        ItsOK &= cl_Matr.Transpon(Jvh, JvhT); //JvhT is Jvh transposed

        ItsOK &= cl_Matr.MatrXMatrXMatr(Jvh, Kcv6x6, JvhT, D_Matr);

        if (ItsOK && D_Matr.M[0][0] >= 0)
        {
            SigmaVH = sqrt(D_Matr.M[0][0]);
        }
    }
    return SigmaVH;
}


double GLCalcTrajParam::CalcRMSE_HorizVel(double &X, double &Y, double &Z, double &VX, double &VY, double &VZ, GLMatrix &Kcv)
{
    double vh, vhor;
    vh = CalcVertVel(X, Y, Z, VX, VY, VZ);
    vhor = CalcHorizVel(X, Y, Z, VX, VY, VZ);

    if (!IsInitialized)
    {
        r = CalcVectMagnitude(X, Y, Z);
    }

    double SigmaVhor = 0;
    bool ItsOK = true;
    if (r > con_eps2 && fabs(vhor) > con_eps2)
    {
        GLMatrix Jvhor(1,6), JvhorT(6,1), D_Matr(1,1), Kcv6x6(6,6);
        CMatrix cl_Matr;

        Jvhor.M[0][0] = (vh/(vhor*r)) * (vh*X/r - VX);
        Jvhor.M[0][1] = (vh/(vhor*r)) * (vh*Y/r - VY);
        Jvhor.M[0][2] = (vh/(vhor*r)) * (vh*Z/r - VZ);
        Jvhor.M[0][3] = (VX - vh*X/r)/vhor;
        Jvhor.M[0][4] = (VY - vh*Y/r)/vhor;
        Jvhor.M[0][5] = (VZ - vh*Z/r)/vhor;

        Kcv6x6 = Kcv.PickOut(0,5,0,5, ItsOK);

        ItsOK &= cl_Matr.Transpon(Jvhor, JvhorT); //JvhorT is Jvhor transposed

        ItsOK &= cl_Matr.MatrXMatrXMatr(Jvhor, Kcv6x6, JvhorT, D_Matr);

        if (ItsOK && D_Matr.M[0][0] >= 0)
        {
            SigmaVhor = sqrt(D_Matr.M[0][0]);
        }
    }
    return SigmaVhor;
}


double GLCalcTrajParam::CalcRMSE_AngleVelHorizont(double &X, double &Y, double &Z, double &VX, double &VY, double &VZ, GLMatrix &Kcv)
{
    double SigmaAngle = 0;
    if (!IsInitialized)
    {
        r = CalcVectMagnitude(X, Y, Z);
        V = CalcVectMagnitude(VX, VY, VZ);
    }
    double rV = r*V;
    if (rV > con_eps2)
    {
        double Q = (X*VX + Y*VY + Z*VZ) / rV;
        double C1 = Q/(r*r);
        double C2 = Q/(V*V);
        double Radicand = 1. - Q*Q;
        if (Radicand > con_eps2)
        {
            double C3 = sqrt(Radicand);
            bool ItsOK = true;
            GLMatrix Jangle(1,6), JangleT(6,1), D_Matr(1,1), Kcv6x6(6,6);
            CMatrix f_Matr;

            Jangle.M[0][0] = (VX/rV - X*C1) / C3;
            Jangle.M[0][1] = (VY/rV - Y*C1) / C3;
            Jangle.M[0][2] = (VZ/rV - Z*C1) / C3;
            Jangle.M[0][3] = (X/rV - VX*C2) / C3;
            Jangle.M[0][4] = (Y/rV - VY*C2) / C3;
            Jangle.M[0][5] = (Z/rV - VZ*C2) / C3;

            Kcv6x6 = Kcv.PickOut(0,5,0,5, ItsOK);

            ItsOK &= f_Matr.Transpon(Jangle, JangleT); //JangleT is Jangle transposed

            ItsOK &= f_Matr.MatrXMatrXMatr(Jangle, Kcv6x6, JangleT, D_Matr);

            if (ItsOK && D_Matr.M[0][0] >= 0)
            {
                SigmaAngle = sqrt(D_Matr.M[0][0]);
            }
        }
    }
    return SigmaAngle;
}


double GLCalcTrajParam::CalcRMSE_VertAccel(double &X, double &Y, double &Z, double &AX, double &AY, double &AZ, GLMatrix &Kca)
{
    double SigmaAH = 0;
    if (!IsInitialized)
    {
        r = CalcVectMagnitude(X, Y, Z);
    }

    double ah = CalcVertAccel(X, Y, Z, AX, AY, AZ);

    if (r > con_par_eps)
    {
        bool bOK;
        GLMatrix Jah(1,6), JahT(6,1), D_Matr(1,1);
        CMatrix cl_Matr;

        Jah.M[0][0] = AX/r - ah*X/(r*r);
        Jah.M[0][1] = AY/r - ah*Y/(r*r);
        Jah.M[0][2] = AZ/r - ah*Z/(r*r);
        Jah.M[0][3] = X/r;
        Jah.M[0][4] = Y/r;
        Jah.M[0][5] = Z/r;

        bOK = cl_Matr.Transpon(Jah, JahT); //JahT is Jah transposed

        bOK = bOK && cl_Matr.MatrXMatrXMatr(Jah, Kca, JahT, D_Matr);

        if (bOK && D_Matr.M[0][0] >= 0)
        {
            SigmaAH = sqrt(D_Matr.M[0][0]);
        }
    }

    return SigmaAH;
}


double GLCalcTrajParam::CalcRMSE_LongitudinalAccel(double &VX, double &VY, double &VZ, double &AX, double &AY, double &AZ, TMatrix<6> &Kva)
{
    double SigmaAl = 0;

    if (!IsInitialized)
    {
        V = CalcAbsVelocity(VX, VY, VZ);
    }

    if (V > con_par_eps)
    {
        double Al = CalcLongitudinalAccel(VX, VY, VZ, AX, AY, AZ);

        TMatrix<6> Jal(1,6), JalT(6,1), D_Matr(1,1);
        TCMatrixFunc<6> fMatr;
        double mult2 = Al/(V*V);

        Jal.M[0][0] = AX/V - mult2*VX;
        Jal.M[0][1] = AY/V - mult2*VY;
        Jal.M[0][2] = AZ/V - mult2*VZ;
        Jal.M[0][3] = VX/V;
        Jal.M[0][4] = VY/V;
        Jal.M[0][5] = VZ/V;

        bool bOK = fMatr.Transpon(Jal, JalT);

        bOK = bOK && fMatr.MatrXMatrXMatr(Jal, Kva, JalT, D_Matr);

        if (bOK && D_Matr.M[0][0] >= 0)
        {
            SigmaAl = sqrt(D_Matr.M[0][0]);
        }
    }
    return SigmaAl;
}


double GLCalcTrajParam::CalcRMSE_LongitudinalAccel(GLPointDouble3D &Vel, GLPointDouble3D &Acc, GLMatrix &Kcva)
{
    double SigmaAl = 0;
    TMatrix<6> Kva;
    TCMatrixFunc<6> fMatr;
    bool bOK = fMatr.Copy(Kva, Kcva, 3, 8, 3, 8);
    if (bOK)
    {
        SigmaAl = CalcRMSE_LongitudinalAccel(Vel.x, Vel.y, Vel.z, Acc.x, Acc.y, Acc.z, Kva);
    }
    return SigmaAl;
}


double GLCalcTrajParam::CalcRMSE_LongitudinalAccel(GLPointDouble3D &Vel, GLPointDouble3D &Acc, TMatrix<9> &Kcva)
{
    double SigmaAl = 0;
    TMatrix<6> Kva;
    TCMatrixFunc<6> fMatr;
    bool bOK = fMatr.Copy(Kva, Kcva, 3, 8, 3, 8);
    if (bOK)
    {
        SigmaAl = CalcRMSE_LongitudinalAccel(Vel.x, Vel.y, Vel.z, Acc.x, Acc.y, Acc.z, Kva);
    }
    return SigmaAl;
}


double GLCalcTrajParam::CalcRMSE_EnHeight_EarthSurface(double &X, double &Y, double &Z, double &VX, double &VY, double &VZ, GLMatrix &Kcv)
{
    double SigmaHe = 0;
    if (!IsInitialized)
    {
        r = CalcVectMagnitude(X, Y, Z);
        CalcS(Z, r);
    }
    if (r > con_eps2)
    {
        double C3 = 1/r - con_b_half_axis*Z*Z*S/pow(r,4);
        double C4 = con_b_half_axis*Z*S/pow(r,2);

        bool ItsOK = true;
        GLMatrix Jhe(1,6), JheT(6,1), D_Matr(1,1), Kcv6x6(6,6);
        CMatrix cl_Matr;

        Jhe.M[0][0] = C3*X;
        Jhe.M[0][1] = C3*Y;
        Jhe.M[0][2] = C3*Z + C4;
        Jhe.M[0][3] = VX / con_g;
        Jhe.M[0][4] = VY / con_g;
        Jhe.M[0][5] = VZ / con_g;

        Kcv6x6 = Kcv.PickOut(0,5,0,5, ItsOK);

        ItsOK &= cl_Matr.Transpon(Jhe, JheT); //JheT is Jhe transposed

        ItsOK &= cl_Matr.MatrXMatrXMatr(Jhe, Kcv6x6, JheT, D_Matr);

        if (ItsOK && D_Matr.M[0][0] >= 0)
        {
            SigmaHe = sqrt(D_Matr.M[0][0]);
        }
    }
    return SigmaHe;
}


double GLCalcTrajParam::CalcRMSE_EnHeight_EarthCenter(double &X, double &Y, double &Z, double &VX, double &VY, double &VZ, GLMatrix &Kcv)
{
    double SigmaHe = 0;
    if (!IsInitialized)
    {
        r = CalcVectMagnitude(X, Y, Z);
        V = CalcVectMagnitude(VX, VY, VZ);
    }

    double C1, C2, Denominator1, Denominator2;
    bool ItsOK = true;
    GLMatrix Jhe(1,6), JheT(6,1), D_Matr(1,1), Kcv6x6(6,6);
    CMatrix cl_Matr;

    Kcv6x6 = Kcv.PickOut(0,5,0,5, ItsOK);
    if (ItsOK && r > con_eps2)
    {
        Denominator1 = r* pow( (con_Grav_Const*con_Earth_Mass - r*V*V/2.), 2);
        Denominator2 = pow(con_Grav_Const*con_Earth_Mass/r - V*V/2., 2);
        if (fabs(Denominator1) > con_eps2 && fabs(Denominator2) > con_eps2)
        {
            C1 = con_Grav_Const*con_Grav_Const * con_Earth_Mass*con_Earth_Mass / Denominator1;
            C2 = con_Grav_Const * con_Earth_Mass /Denominator2;

            Jhe.M[0][0] = X*C1;
            Jhe.M[0][1] = Y*C1;
            Jhe.M[0][2] = Z*C1;
            Jhe.M[0][3] = VX*C2;
            Jhe.M[0][4] = VY*C2;
            Jhe.M[0][5] = VZ*C2;

            ItsOK &= cl_Matr.Transpon(Jhe, JheT); //JheT is Jhe transposed

            ItsOK &= cl_Matr.MatrXMatrXMatr(Jhe, Kcv6x6, JheT, D_Matr);

            if (ItsOK && D_Matr.M[0][0] >= 0)
            {
                SigmaHe = sqrt(D_Matr.M[0][0]);
            }
        }
    }
    return SigmaHe;
}


double GLCalcTrajParam::CalcRMSE_VelEnHeight_EarthSurface(double &X, double &Y, double &Z, double &VX, double &VY, double &VZ, double &AX, double &AY, double &AZ, GLMatrix &Kcva)
{
    if (!IsInitialized)
    {
        r = CalcVectMagnitude(X, Y, Z);
    }

    double SigmaVHe = 0;
    bool ItsOK = true;
    double CC1, CC2, CC3, CC4, Cxyz1, Cz1, Cxyz2, CVxyz1, CVz2, rc4;
    GLMatrix JVhe(1,9), JVheT(9,1), D_Matr(1,1);
    CMatrix cl_Matr;
    if (r > con_eps2)
    {
        CC1 = con_b_half_axis * (1. - con_Eath_compress_coeff) * (2.*con_Eath_compress_coeff - pow(con_Eath_compress_coeff, 2));
        CC2 = 3. * CC1 * (2.*con_Eath_compress_coeff - pow(con_Eath_compress_coeff, 2));
        CC3 = VX*X + VY*Y + VZ*Z;
        CC4 = pow(1. - con_Eath_compress_coeff, 2) + Z*(2.*con_Eath_compress_coeff - pow(con_Eath_compress_coeff, 2))/(r*r);

        rc4 = pow(r,4) * pow(CC4, 3./2.);
        if (fabs(CC4) > con_eps2 && fabs(rc4) > con_eps2)
        {
            Cxyz1 = - CC3/(r*r*r) + (Z*Z*Z*VZ*CC2/(r*r*CC4) -
                                     pow(Z,4)*CC3*CC2/(pow(r,4)*CC4) + 4.*Z*Z*CC3*CC1/(r*r) - 2.*Z*VZ*CC1) / rc4;

            Cz1 = (- Z*Z*VZ*CC2/CC4 + Z*Z*Z*CC2*CC3/(r*r*CC4) - 2.*Z*CC1*CC3 + VZ*r*r*CC1) / rc4;

            Cxyz2 = 1/r - Z*Z*CC1/rc4;

            CVxyz1 = Cxyz2;

            CVz2 = Z*CC1*r*r/rc4;

            JVhe.M[0][0] = X*Cxyz1 + VX*Cxyz2;
            JVhe.M[0][1] = Y*Cxyz1 + VY*Cxyz2;
            JVhe.M[0][2] = Z*Cxyz1 + VZ*Cxyz2 + Cz1;
            JVhe.M[0][3] = AX/con_g + X*CVxyz1;
            JVhe.M[0][4] = AY/con_g + Y*CVxyz1;
            JVhe.M[0][5] = AZ/con_g + Z*CVxyz1 + CVz2;
            JVhe.M[0][6] = VX/con_g;
            JVhe.M[0][7] = VY/con_g;
            JVhe.M[0][8] = VZ/con_g;

            ItsOK &= cl_Matr.Transpon(JVhe, JVheT); //JheT is Jhe transposed

            ItsOK &= cl_Matr.MatrXMatrXMatr(JVhe, Kcva, JVheT, D_Matr);

            if (ItsOK && D_Matr.M[0][0] >= 0)
            {
                SigmaVHe = sqrt(D_Matr.M[0][0]);
            }
        }
    }
    return SigmaVHe;
}


double GLCalcTrajParam::CalcRMSE_Distance(double &X1, double &Y1, double &Z1, double &X2, double &Y2, double &Z2, TMatrix<SIZE_M> &Kc1, TMatrix<SIZE_M> &Kc2)
{
    double D = CalcDistance(X1, Y1, Z1, X2, Y2, Z2);

    GLMatrix Zeros3x3(3,3);
    GLMatrix Cov_1_2(6,6);

    double SigmaD = 0;
    bool ItsOK=true, bResult=true;
    GLMatrix Jd(1,6), JdT(6,1), D_Matr(1,1), M1(3,6), M2(3,6);
    CMatrix cl_Matr;
    if (D > con_eps2)
    {
        ItsOK = cl_Matr.ConcatLeftRight(Kc1, Zeros3x3, M1); bResult = bResult && ItsOK; //M1 is Kc1 concatenated with Zeros3x3 left to right
        ItsOK = cl_Matr.ConcatLeftRight(Zeros3x3, Kc2, M2); bResult = bResult && ItsOK; //M2 is Zeros3x3 concatenated with Kc2 left to right
        ItsOK = cl_Matr.ConcatUpDown(M1, M2, Cov_1_2);      bResult = bResult && ItsOK; //Cov_1_2 is M1 concatenated with M2 up to down

        Jd.M[0][0] = (X1 - X2) / D;
        Jd.M[0][1] = (Y1 - Y2) / D;
        Jd.M[0][2] = (Z1 - Z2) / D;
        Jd.M[0][3] = - Jd.M[0][0];
        Jd.M[0][4] = - Jd.M[0][1];
        Jd.M[0][5] = - Jd.M[0][2];

        ItsOK = cl_Matr.Transpon(Jd, JdT); bResult = bResult && ItsOK; //JdT is Jd transposed
        ItsOK = cl_Matr.MatrXMatrXMatr(Jd, Cov_1_2, JdT, D_Matr); bResult = bResult && ItsOK; //D_Matr = Jd x Cov_1_2 x JdT

        if (bResult && D_Matr.M[0][0] >= 0)
        {
            SigmaD = sqrt(D_Matr.M[0][0]);
        }
    }
    return SigmaD;
}


double GLCalcTrajParam::CalcRMSE_Delta_Distance(double &X1, double &Y1, double &Z1, double &X2, double &Y2, double &Z2, double &XC, double &YC, double &ZC, GLMatrix &Kc1, GLMatrix &Kc2, GLMatrix &KcC)
{
    double D1 = CalcDistance(X1, Y1, Z1, XC, YC, ZC);
    double D2 = CalcDistance(X2, Y2, Z2, XC, YC, ZC);

    GLMatrix Zeros3x3(3,3);
    GLMatrix Cov_1_2_C(9,9);

    double SigmaDD = 0;
    bool ItsOK, bResult=true;
    GLMatrix Jd(1,9), JdT(9,1), D_Matr(1,1), M1(3,9), M2(3,9), M3(3,9);
    CMatrix cl_Matr;
    if (D1 > con_eps2 && D2 > con_eps2)
    {
        ItsOK = cl_Matr.ConcatLeftRight(Kc1, Zeros3x3, Zeros3x3, M1); bResult = bResult && ItsOK; //M1 is Kc1 concatenated with Zeros3x3 with Zeros3x3 left to right
        ItsOK = cl_Matr.ConcatLeftRight(Zeros3x3, Kc2, Zeros3x3, M2); bResult = bResult && ItsOK; //M2 is Zeros3x3 concatenated with Kc2 with Zeros3x3 left to right
        ItsOK = cl_Matr.ConcatLeftRight(Zeros3x3, Zeros3x3, KcC, M3); bResult = bResult && ItsOK; //M2 is Zeros3x3 concatenated with Zeros3x3 with KcC left to right
        ItsOK = cl_Matr.ConcatUpDown(M1, M2, M3, Cov_1_2_C);      bResult = bResult && ItsOK; //Cov_1_2_C is M1 concatenated with M2 with M3 up to down

        Jd.M[0][0] = (X1 - XC) / D1;
        Jd.M[0][1] = (Y1 - YC) / D1;
        Jd.M[0][2] = (Z1 - ZC) / D1;
        Jd.M[0][3] = - (X2 - XC) / D2;
        Jd.M[0][4] = - (Y2 - YC) / D2;
        Jd.M[0][5] = - (Z2 - ZC) / D2;
        Jd.M[0][6] = (D1*(X2 - XC) - D2*(X1 - XC)) / (D1*D2);
        Jd.M[0][7] = (D1*(Y2 - YC) - D2*(Y1 - YC)) / (D1*D2);
        Jd.M[0][8] = (D1*(Z2 - ZC) - D2*(Z1 - ZC)) / (D1*D2);

        ItsOK = cl_Matr.Transpon(Jd, JdT); bResult = bResult && ItsOK; //JdT is Jd transposed
        ItsOK = cl_Matr.MatrXMatrXMatr(Jd, Cov_1_2_C, JdT, D_Matr); bResult = bResult && ItsOK; //D_Matr = Jd x Cov_1_2_C x JdT

        if (bResult && D_Matr.M[0][0] >= 0)
        {
            SigmaDD = sqrt(D_Matr.M[0][0]);
        }
    }
    return SigmaDD;
}


double GLCalcTrajParam::CalcRMSE_Angle2vec(double &X1, double &Y1, double &Z1, double &X2, double &Y2, double &Z2, GLMatrix &Kc1, GLMatrix &Kc2)
{
    double SigmaAngle = 0;
    double mod_vec1 = CalcVectMagnitude(X1, Y1, Z1);
    double mod_vec2 = CalcVectMagnitude(X2, Y2, Z2);
    double scal_prod = X1*X2 + Y1*Y2 + Z1*Z2;
    if (mod_vec1 > con_eps2 && mod_vec2 > con_eps2)
    {
        double P1 = scal_prod / (sqr(mod_vec1));
        double P2 = scal_prod / (sqr(mod_vec2));
        double Radicand = sqr(mod_vec1*mod_vec2) - sqr(scal_prod);
        if (Radicand > con_eps2)
        {
            double Q = sqrt(Radicand);

            bool ItsOK=true, bResult=true;
            GLMatrix Ja(1,6), JaT(6,1), sA_Matr(1,1), Kc12_6x6(6,6), Kc1_3x3, Kc2_3x3;
            CMatrix fMatr;

            Kc1_3x3 = Kc1.PickOut(0,2,0,2, ItsOK); bResult &= ItsOK;

            Kc2_3x3 = Kc2.PickOut(0,2,0,2, ItsOK); bResult &= ItsOK;

            ItsOK = fMatr.Blkdiag2(Kc1_3x3, Kc2_3x3, Kc12_6x6); bResult &= ItsOK;

            Ja.M[0][0] = (P1*X1 - X2) / Q;
            Ja.M[0][1] = (P1*Y1 - Y2) / Q;
            Ja.M[0][2] = (P1*Z1 - Z2) / Q;
            Ja.M[0][3] = (P2*X2 - X1) / Q;
            Ja.M[0][4] = (P2*Y2 - Y1) / Q;
            Ja.M[0][3] = (P2*Z2 - Z1) / Q;

            ItsOK = fMatr.Transpon(Ja, JaT); bResult &= ItsOK; //JaT is Ja transposed
            ItsOK = fMatr.MatrXMatrXMatr(Ja, Kc12_6x6, JaT, sA_Matr); bResult &= ItsOK; //sA_Matr = Ja*Kc1*JaT

            if (bResult && sA_Matr.M[0][0] >= 0)
            {
                SigmaAngle = sqrt(sA_Matr.M[0][0]);
            }
        }
    }
    return SigmaAngle;
}


double GLCalcTrajParam::CalcRMSE_Angle2vec_1stRnd_2ndConst(double &X1, double &Y1, double &Z1, double &X2, double &Y2, double &Z2, GLMatrix &Kc1)
{
    double SigmaAngle = 0;
    double mod_vec1 = CalcVectMagnitude(X1, Y1, Z1);
    double mod_vec2 = CalcVectMagnitude(X2, Y2, Z2);
    double scal_prod = X1*X2 + Y1*Y2 + Z1*Z2;
    double Radicand = sqr(mod_vec1*mod_vec2) - sqr(scal_prod);
    if (mod_vec1 > con_eps2 && Radicand > con_eps2)
    {
        double P = scal_prod / (sqr(mod_vec1));
        double Q = sqrt(Radicand);

        bool ItsOK=true, bResult=true;
        GLMatrix Ja(1,3), JaT(3,1), sA_Matr(1,1), Kc1_3x3;
        CMatrix fMatr;

        Kc1_3x3 = Kc1.PickOut(0,2,0,2, ItsOK); bResult &= ItsOK;

        Ja.M[0][0] = (P*X1 - X2) / Q;
        Ja.M[0][1] = (P*Y1 - Y2) / Q;
        Ja.M[0][2] = (P*Z1 - Z2) / Q;

        ItsOK = fMatr.Transpon(Ja, JaT); bResult &= ItsOK; //JaT is Ja transposed
        ItsOK = fMatr.MatrXMatrXMatr(Ja, Kc1_3x3, JaT, sA_Matr); bResult &= ItsOK; //sA_Matr = Ja*Kc1*JaT

        if (bResult && sA_Matr.M[0][0] >= 0)
        {
            SigmaAngle = sqrt(sA_Matr.M[0][0]);
        }
    }
    return SigmaAngle;
}


double GLCalcTrajParam::CalcRMSE_ProjectionVect1OnVect2(GLPointDouble3D &Vect1, GLPointDouble3D &Vect2, TMatrix<3> &Cov1, TMatrix<3> &Cov2)
{
    double SigmaProj = 0;
    bool bOK = true;

    if (!Vect2.IsZero())
    {
        TMatrix<6> Jproj(1,6), JprojT(6,1), D_Matr(1,1), K12(6,6);
        TCMatrixFunc<6> fMatr;

        double Mod2 = Vect2.moduleFloat(); //magnitude of Vect2
        double Proj12 = Vect1.getProjectionOn(Vect2); //projection of Vect1 on Vect2
        double Coef2 = Proj12 / (Mod2 * Mod2);

        bOK = fMatr.Blkdiag2(Cov1, Cov2, K12);
        if (bOK)
        {
            Jproj.M[0][0] = Vect2.x / Mod2; //d_projection/d_x1
            Jproj.M[0][1] = Vect2.y / Mod2; //d_projection/d_y1
            Jproj.M[0][2] = Vect2.z / Mod2; //d_projection/d_z1
            Jproj.M[0][3] = Vect1.x / Mod2 - Vect2.x * Coef2; //d_projection/d_x2
            Jproj.M[0][4] = Vect1.y / Mod2 - Vect2.y * Coef2; //d_projection/d_y2
            Jproj.M[0][5] = Vect1.z / Mod2 - Vect2.z * Coef2; //d_projection/d_z2

            JprojT = fMatr.Transpon(Jproj); //JprojT is Jproj transposed

            bOK = fMatr.MatrXMatrXMatr(Jproj, K12, JprojT, D_Matr); // D_Matr = Jproj * K12 * JprojT;
            if (bOK)
            {
                if (D_Matr.M[0][0] >= 0)
                {
                    SigmaProj = sqrt(D_Matr.M[0][0]);
                }
            }
        }
    }
    return SigmaProj;
}


bool GLCalcTrajParam::CalcAngle_RMSE_Angle_OP1V1_P2P3(GLPointDouble3D &P1, GLPointDouble3D &V1, GLPointDouble3D &P2, GLPointDouble3D &P3, TMatrix<3> &CovP1, TMatrix<3> &CovV1, TMatrix<3> &CovP2, TMatrix<3> &CovP3, double &ResAngle, double &ResRMSE_Angle)
{
    bool bResOK = true; //true if result is OK
    bool bOK = true;
    ResAngle = 0;
    ResRMSE_Angle = 0;

    double A, B, C, DirX, DirY, DirZ,
            mod_plane, mod_dir, P, Q, S,
            dQ_dA, dQ_dB, dQ_dC, dQ_dDirX, dQ_dDirY, dQ_dDirZ; //auxiliary variables

    //directions from P2 to P3
    DirX = P3.x - P2.x;
    DirY = P3.y - P2.y;
    DirZ = P3.z - P2.z;

    //coefficients A, B, C of equation of plane "Origin of coordinates" - "Point P1" - "Velocity V1"
    A = P1.y * V1.z - P1.z * V1.y;
    B = P1.z * V1.x - P1.x * V1.z;
    C = P1.x * V1.y - P1.y * V1.x;

    mod_plane = sqrt(A*A + B*B + C*C);
    mod_dir = sqrt(DirX*DirX + DirY*DirY + DirZ*DirZ);

    S = sqrt(sqr(A*DirY - B*DirX) + sqr(B*DirZ - C*DirY) + sqr(C*DirX - A*DirZ));

    if (fabs(S) > con_par_eps && fabs(mod_plane) > con_par_eps && fabs(mod_dir) > con_par_eps)
    {
        Q = (DirX * A + DirY * B + DirZ * C) / (mod_plane * mod_dir);
        if (fabs(Q) <= 1.)
        {
            ResAngle = asin(fabs(Q)); //angle (Alpha) is calculated

            P = mod_plane * mod_dir / S; //P = dAlpha/dQ

            dQ_dA = DirX/(mod_plane*mod_dir) - A*Q/sqr(mod_plane);
            dQ_dB = DirY/(mod_plane*mod_dir) - B*Q/sqr(mod_plane);
            dQ_dC = DirZ/(mod_plane*mod_dir) - C*Q/sqr(mod_plane);
            dQ_dDirX = A/(mod_plane*mod_dir) - DirX*Q/sqr(mod_dir);
            dQ_dDirY = B/(mod_plane*mod_dir) - DirY*Q/sqr(mod_dir);
            dQ_dDirZ = C/(mod_plane*mod_dir) - DirZ*Q/sqr(mod_dir);

            const qint32 dm = 12;
            TMatrix<dm> J(1,dm), //Jacobi matrix: dAlpha/dX2, Y2, Z2, X3, Y3, Z3, X1, Y1, Z1, VX1, VY1, VZ1
                    JT(dm,1), //J transposed
                    Cov(dm,dm), //simplified covariance matrix for X2, Y2, Z2, X3, Y3, Z3, X1, Y1, Z1, VX1, VY1, VZ1
                    D_matr(1,1); //dispersion in matrix representation

            J.M[0][0]  = - P * dQ_dDirX; //dAlpha/dX2
            J.M[0][1]  = - P * dQ_dDirY; //dAlpha/dY2
            J.M[0][2]  = - P * dQ_dDirX; //dAlpha/dZ2
            J.M[0][3]  = P * dQ_dDirX; //dAlpha/dX3
            J.M[0][4]  = P * dQ_dDirY; //dAlpha/dY3
            J.M[0][5]  = P * dQ_dDirZ; //dAlpha/dZ3
            J.M[0][6]  = P * ( dQ_dC * V1.y - dQ_dB * V1.z ); //dAlpha/dX1
            J.M[0][7]  = P * ( dQ_dA * V1.z - dQ_dC * V1.x ); //dAlpha/dY1
            J.M[0][8]  = P * ( dQ_dB * V1.x - dQ_dA * V1.y ); //dAlpha/dZ1
            J.M[0][9]  = P * ( dQ_dB * P1.z - dQ_dC * P1.y ); //dAlpha/dVX1
            J.M[0][10] = P * ( dQ_dC * P1.x - dQ_dA * P1.z ); //dAlpha/dVY1
            J.M[0][11] = P * ( dQ_dA * P1.y - dQ_dB * P1.x ); //dAlpha/dVZ1

            TCMatrixFunc<dm> fMatr;
            bOK = fMatr.Transpon(J, JT); bResOK = bResOK && bOK;

            bOK = fMatr.Blkdiag4(CovP2, CovP3, CovP1, CovV1, Cov); bResOK = bResOK && bOK; //Matrice Cov contains CovP2, CovP3, CovP1, CovV1 on the diagonal

            bOK = fMatr.MatrXMatrXMatr(J, Cov, JT, D_matr); bResOK = bResOK && bOK; //D_matr = J * Cov * JT
            if (bResOK && D_matr.M[0][0] >= 0)
            {
                ResRMSE_Angle = sqrt(D_matr.M[0][0]);
            }
        }
        else
        {       //division by 0
            bResOK = false;
        }
    }
    return bResOK;
}


bool GLCalcTrajParam::CalcRMSE_AerodynamicAccel(double X, double Y, double Z, double VX, double VY, double VZ, double AX, double AY, double Az, GLMatrix &Kcva, double &SigAccDrag, double &SigAccLift, double &SigAccLateral)
{
    bool bOK = true;

    SigAccDrag = 0;
    SigAccLift = 0;
    SigAccLateral = 0;

    double ModCoord = CalcVectMagnitude(X, Y, Z);
    GLPointDouble3D Vel(VX, VY, VZ); //velocity vector
    double ModVel = Vel.moduleFloat(); //absolute velocity
    if (ModCoord > con_eps2 && ModVel > con_eps2)
    {
        GLPointDouble3D CoordNorm(X/ModCoord, Y/ModCoord, Z/ModCoord); //normalized vector of coordinates
        GLPointDouble3D AccAerodyn(AX + con_g*CoordNorm.x, AY + con_g*CoordNorm.y, Az + con_g*CoordNorm.z); //common vector of aerodynamic accelerations = a_drag + a_lift + a_lateral = a - a_gravitational
                                                    //+g because (a-a_gravitational)=a-(-g*CoordNorm)
        GLPointDouble3D NLift; //directional vector for lift acceleration
        NLift = CoordNorm * (ModVel*ModVel) - (Vel*(CoordNorm.ScalarProduct(Vel)));

        double ModNLift = NLift.moduleFloat();
        if (ModNLift > con_eps2)
        {
            TCMatrixFunc<3> fMatr;
            TMatrix<3> Kv(3,3), Ka(3,3),
                       K_NLift(3,3), //covariance matrix of the vector NLift
                       J_NLift_V(3,3), JT_NLift_V(3,3); //Jacobi matrix d(NLift)/d(Vel)
            bOK = fMatr.Copy(Kv, Kcva, 3, 5, 3, 5);
            bOK = bOK && fMatr.Copy(Ka, Kcva, 6, 8, 6, 8);
            if (bOK)
            {
                SigAccDrag = CalcRMSE_ProjectionVect1OnVect2(AccAerodyn, Vel, Ka, Kv);

                J_NLift_V.M[0][0] = - CoordNorm.y * Vel.y - CoordNorm.z * Vel.z;
                J_NLift_V.M[0][1] = 2. * CoordNorm.x * Vel.y - CoordNorm.y * Vel.x;
                J_NLift_V.M[0][2] = 2. * CoordNorm.x * Vel.z - CoordNorm.z * Vel.x;

                J_NLift_V.M[1][0] = 2. * CoordNorm.y * Vel.x - CoordNorm.x * Vel.y;
                J_NLift_V.M[1][1] = - CoordNorm.x * Vel.x - CoordNorm.z * Vel.z;
                J_NLift_V.M[1][2] = 2. * CoordNorm.y * Vel.z - CoordNorm.z * Vel.y;

                J_NLift_V.M[2][0] = 2. * CoordNorm.z * Vel.x - CoordNorm.x * Vel.z;
                J_NLift_V.M[2][1] = 2. * CoordNorm.z * Vel.y - CoordNorm.y * Vel.z;
                J_NLift_V.M[2][2] = - CoordNorm.x * Vel.x - CoordNorm.y * Vel.y;

                bOK = fMatr.Transpon(J_NLift_V, JT_NLift_V); //JT_NLift_V is J_NLift_V transposed
                bOK = bOK && fMatr.MatrXMatrXMatr(J_NLift_V, Kv, JT_NLift_V, K_NLift); //K_NLift = J_NLift_V * Kv * JT_NLift_V
                if (bOK)
                {
                    SigAccLift = CalcRMSE_ProjectionVect1OnVect2(AccAerodyn, NLift, Ka, K_NLift);

                    GLPointDouble3D NLat; //directional vector for lateral acceleration
                    NLat.x = CoordNorm.y * Vel.z - CoordNorm.z * Vel.y;
                    NLat.y = CoordNorm.z * Vel.x - CoordNorm.x * Vel.z;
                    NLat.z = CoordNorm.x * Vel.y - CoordNorm.y * Vel.x;

                    TMatrix<3> K_NLat(3,3), //covariance matrix of the vector NLat
                               J_NLat_V(3,3), JT_NLat_V(3,3); //Jacobi matrix d(NLift)/d(Vel)

                    J_NLat_V.M[0][0] = 0;
                    J_NLat_V.M[0][1] = - CoordNorm.z;
                    J_NLat_V.M[0][2] = CoordNorm.y;

                    J_NLat_V.M[1][0] = CoordNorm.z;
                    J_NLat_V.M[1][1] = 0;
                    J_NLat_V.M[1][2] = - CoordNorm.x;

                    J_NLat_V.M[2][0] = - CoordNorm.y;
                    J_NLat_V.M[2][1] = CoordNorm.x;
                    J_NLat_V.M[2][2] = 0;

                    bOK = fMatr.Transpon(J_NLat_V, JT_NLat_V); //JT_NLat_V is J_NLat_V transposed
                    bOK = bOK && fMatr.MatrXMatrXMatr(J_NLat_V, Kv, JT_NLat_V, K_NLat); //K_NLat = J_NLat_V * Kv * JT_NLat_V
                    if (bOK)
                    {
                        SigAccLateral = CalcRMSE_ProjectionVect1OnVect2(AccAerodyn, NLat, Ka, K_NLat);
                    }
                }
            }
        }
        else
        {
            bOK = false;
        }
    }
    else
    {
        bOK = false;
    }
    return bOK;
}


bool GLCalcTrajParam::CalcRMSE_AerodynamicAccel(GLPointDouble3D &Coord, GLPointDouble3D &Vel, GLPointDouble3D &Acc, GLMatrix &Kcva, double &SigAccDrag, double &SigAccLift, double &SigAccLateral)
{
    return CalcRMSE_AerodynamicAccel(Coord.x, Coord.y, Coord.z, Vel.x, Vel.y, Vel.z, Acc.x, Acc.y, Acc.z,
                                     Kcva, SigAccDrag, SigAccLift, SigAccLateral);
}


bool GLCalcTrajParam::CalcRMSE_AerodynamicAccel(GLPointDouble3D &Coord, GLPointDouble3D &Vel, GLPointDouble3D &Acc, TMatrix<9> &Kcva, double &SigAccDrag, double &SigAccLift, double &SigAccLateral)
{
    bool bOK = true;
    GLMatrix Kcva1(9,9);
    CMatrix FMatr;
    bOK = FMatr.Copy(Kcva1, Kcva, 0, 8, 0, 8);
    if (bOK)
    {
        bOK = CalcRMSE_AerodynamicAccel(Coord, Vel, Acc, Kcva1, SigAccDrag, SigAccLift, SigAccLateral);
    }
    return bOK;
}


bool GLCalcTrajParam::CalcRPerigee_RMSE_RPergiee(GLPointDouble3D &CoordGC, GLPointDouble3D &VelGC, TMatrix<SIZE_M> &Cov, double &ResRp, double &ResRMSERp)
{
    bool bOK = true;
    ResRp = 0.;
    ResRMSERp = 0.;
    GLPointDouble3D Eta(0., 0., 1); //direction vector of the Earth rotation angular velocity
    GLPointDouble3D r_vect, v_vect;
    r_vect = CoordGC;
    v_vect = VelGC + (Eta.VectorProduct(CoordGC) * con_Eath_angle_speed);

    double mod_r = r_vect.moduleFloat();
    double mod_v = v_vect.moduleFloat();
    if (mod_r > con_eps && mod_v > con_eps)
    {
        double r_scalar_v = r_vect.ScalarProduct(v_vect);
        double sinTheta = r_scalar_v / (mod_r * mod_v);
        double Gamma = mod_r * sqr(mod_v) / con_mu;
        double p = Gamma * mod_r * (1 - sqr(sinTheta));
        double e2 = sqr(1 - Gamma) + Gamma * (2. - Gamma) * sqr(sinTheta);
        if (e2 < con_par_eps)
        {
            bOK = false;
        }
        else
        {
            double e = sqrt(e2);
            if (fabs(1. + e) > con_par_eps)
            {
                ResRp = p / (1. + e);

                GLPointDouble3D d_mod_r_d_CoordGC = r_vect / mod_r;
                GLPointDouble3D d_mod_v_d_VelGC = v_vect / mod_v;
                GLPointDouble3D d_mod_v_d_CoordGC = (Eta.VectorProduct(v_vect)) * (-con_Eath_angle_speed / mod_v);
                GLPointDouble3D d_sinTh_d_CoordGC = (VelGC - (((r_vect / sqr(mod_r)) + (d_mod_v_d_CoordGC / mod_v)) * r_scalar_v)) / (mod_r * mod_v);
                GLPointDouble3D d_sinTh_d_VelGC = (r_vect - (v_vect * (r_scalar_v /sqr(mod_v)))) / (mod_r * mod_v);

                double dRp_d_mod_r = 0, dRp_d_mod_v = 0, dRp_d_sinTh = 0;
                if (e > con_eps)
                {
//                    double de_dGamma = (Gamma - 1.)*(1. - sqr(sinTheta)) / e;
                    double de_d_mod_r = p * (Gamma - 1.) / (sqr(mod_r) * e);
                    double de_d_mod_v = 2. * de_d_mod_r * mod_r / mod_v;
                    double de_d_sinTh = Gamma * (2. - Gamma) * sinTheta / e;

                    dRp_d_mod_r = (2. * p / mod_r - ResRp * de_d_mod_r) / (1. + e);
                    dRp_d_mod_v = (2. * p / mod_v - ResRp * de_d_mod_v) / (1. + e);
                    dRp_d_sinTh = (-2. * Gamma * mod_r * sinTheta + ResRp * de_d_sinTh) / (1. + e);
                }
                else
                {
                    dRp_d_mod_r = 3.;
                    dRp_d_mod_v = 4. * mod_r / mod_v;
                    dRp_d_sinTh = - mod_r * signum(sinTheta);
                }

                GLPointDouble3D dRp_d_CoordGC = d_mod_r_d_CoordGC * dRp_d_mod_r
                                                + d_mod_v_d_CoordGC * dRp_d_mod_v
                                                + d_sinTh_d_CoordGC * dRp_d_sinTh;
                GLPointDouble3D dRp_d_VelGC = d_mod_v_d_VelGC * dRp_d_mod_v
                                              + d_sinTh_d_VelGC * dRp_d_sinTh;

                GLMatrix Cov6(6, 6), J_Rp(1,6), J_Rp_T(6, 1), D_Matr(1, 1);
                CMatrix fMatr;

                Cov6 = Cov.PickOut(0, 5, 0, 5, bOK);

                J_Rp.M[0][0] = dRp_d_CoordGC.x;
                J_Rp.M[0][1] = dRp_d_CoordGC.y;
                J_Rp.M[0][2] = dRp_d_CoordGC.z;
                J_Rp.M[0][3] = dRp_d_VelGC.x;
                J_Rp.M[0][4] = dRp_d_VelGC.y;
                J_Rp.M[0][5] = dRp_d_VelGC.z;

                J_Rp_T = fMatr.Transpon(J_Rp);

                bOK = bOK && fMatr.MatrXMatrXMatr(J_Rp, Cov6, J_Rp_T, D_Matr);
                if (bOK && D_Matr.M[0][0] > 0)
                {
                    ResRMSERp = sqrt(D_Matr.M[0][0]);
                }
                else
                {
                    bOK = false;
                }
            }
            else
            {
                bOK = false;
            }
        }
    }
    else
    {
        bOK = false;
    }
    return bOK;
}

bool GLCalcTrajParam::CalcAzVel_RMSE_AzVel(double X, double Y, double Z, double VX, double VY, double VZ, TMatrix<3> &CovV, double &ResAzVel, double &ResSigAz)
{
    bool bOK = true;
    ResAzVel = 0;
    if (fabs(X) > con_par_eps || fabs(Y) > con_par_eps || fabs(Z) > con_par_eps)
    {
        CGeocentric PointGC(X, Y, Z);
        CGeodesic PointGD;
        CGeocentric VelGC(VX, VY, VZ);
        CTopocentric VelTP;

        bOK = bOK && GEOCENTRIC_GEO(&PointGC, &PointGD);
        bOK = bOK && GEOCENTRIC_TOPO(&PointGD, 0, &VelGC, 0, 0, &VelTP, 0);
        bOK = bOK && TOPO__GET_AZIMUTH(&VelTP, &ResAzVel);

        TMatrix<3> CovVelTP;
        bOK = bOK && GL_MATR::Recount_CovMatr_GeocentricToTopo(&PointGD, &CovV, &CovVelTP);

        double d2vxz = sqr(VelTP.m_dXt) + sqr(VelTP.m_dZt);
        double Radicand =  sqr(VelTP.m_dXt) * CovVelTP.M[0][0]
                            + sqr(VelTP.m_dZt) * CovVelTP.M[2][2]
                            - 2. * VelTP.m_dXt * VelTP.m_dZt * CovVelTP.M[0][2]  ;
        if (bOK && d2vxz > con_par_eps && Radicand >= 0)
        {
            ResSigAz = sqrt(Radicand) / d2vxz;
        }
        else
        {
            bOK = false;
        }
    }
    else
    {
        bOK = false;
    }
    return bOK;
}


bool GLCalcTrajParam::CalcAzVel_RMSE_AzVel(GLPointDouble3D &Coord, GLPointDouble3D &Vel, TMatrix<3> &CovV, double &ResAzVel, double &ResSigAz)
{
    return CalcAzVel_RMSE_AzVel(Coord.x, Coord.y, Coord.z, Vel.x, Vel.y, Vel.z, CovV, ResAzVel, ResSigAz);
}


bool GLCalcTrajParam::ConvertEllipseToCovMatrNUE(double aI, double bI, double betaI, TMatrix<3> &ResCov)
{
    bool bResOK = true; //true if result is OK
    ResCov.Reset(3,3);
    bool bOK = true;
    if (aI > con_par_eps && bI > con_par_eps)
    {
        TMatrix<2> K(2,2), K0(2,2), Q(2,2), QT(2,2);

        K0.M[0][0] = aI*aI; //covariance matrix 2x2 in the coordinate system x0, z0:
        K0.M[1][1] = bI*bI; //x0 - along aI, z0 - along bI

        Q.M[0][0] = cos(betaI); //rotation matrix in the plane X-Z, x0->x, z0->z (turn at angle betaI)
        Q.M[0][1] = sin(betaI);
        Q.M[1][0] = -sin(betaI);
        Q.M[1][1] = cos(betaI);

        TCMatrixFunc<2> fMatr; //instance of class for working with matrices
        QT = fMatr.Transpon(Q); //QT is Q transposed

        bOK = fMatr.MatrXMatrXMatr(QT, K0, Q, K); //K = QT * K0 * Q
        if (bOK)
        {
            double sY = con_Eath_middle_radius - sqrt( fabs(sqr(con_Eath_middle_radius) - sqr(aI)) );
            ResCov.M[0][0] = K.M[0][0];
            ResCov.M[0][2] = K.M[0][1];
            ResCov.M[2][0] = K.M[1][0];
            ResCov.M[2][2] = K.M[1][1];
            ResCov.M[1][1] = sY * sY;
        }
        else
        {
            bResOK = false;
        }
    }
    else
    {
        bResOK = false;
    }

    return bResOK;
}


bool GLCalcTrajParam::ConvertEllipseToCovMatrNUE(double aI, double bI, double betaI, GLMatrix &ResCov)
{
    bool bOK = true;
    ResCov.Reset(3,3);
    TMatrix<3> ResCov3(3,3);
    bOK = ConvertEllipseToCovMatrNUE(aI, bI, betaI, ResCov3);
    if (bOK)
    {
        TCMatrixFunc<3> fMatr;
        bOK = fMatr.Copy(ResCov, ResCov3, 0, 2, 0, 2);
    }
    return bOK;
}


bool GLCalcTrajParam::ConvertEllipseToCovMatrECEF(double aI, double bI, double betaI, GLPointDouble3D &FP, TMatrix<3> &ResCov)
{
    bool bResOK = true; //true if result is OK

    TMatrix<3> CovNUE(3,3); //covariance matrix in NUE
    bResOK = ConvertEllipseToCovMatrNUE(aI, bI, betaI, CovNUE);
    bResOK = (FP.IsZero() ? false : bResOK);

    if (bResOK)
    {
        CGeocentric Center_gc(FP.x, FP.y, FP.z); //center of NUE in geocentric coordinate system
        CGeodesic Center_gd; //center of NUE in geodesic coordinate system
        bResOK = GEOCENTRIC_GEO(&Center_gc, &Center_gd); //recalculation from geocentric to the geodesic coordinate system
        if (bResOK)
        {
            bResOK = GL_MATR::Recount_CovMatr_TopoToGeocentric(&Center_gd, &CovNUE, &ResCov);
        }
    }
    return bResOK;
}


bool GLCalcTrajParam::ConvertEllipseToCovMatrECEF(double aI, double bI, double betaI, GLPointDouble3D &FP, GLMatrix &ResCov)
{
    TMatrix<3> Cov3;
    ResCov.Reset();
    bool bResOK = ConvertEllipseToCovMatrECEF(aI, bI, betaI, FP, Cov3);
    if (bResOK)
    {
        CMatrix fMatr;
        bResOK = fMatr.Copy(ResCov, Cov3);
    }
    return bResOK;
}


bool GLCalcTrajParam::Weighting2PointsUsingCov(GLPointDouble3D &P1, GLPointDouble3D &P2, TMatrix<3> &Cov1, TMatrix<3> &Cov2, GLPointDouble3D &PRes, TMatrix<3> &CovRes)
{
    bool bRes = true, bOK = true;

    TCMatrixFunc<3> fMatr;
    TVector<3> P1vec, P2vec; //points P1, P2 represented as vectors
    GLCoordToVector(P1, P1vec);
    GLCoordToVector(P2, P2vec);

    TMatrix<3> Matr1, Matr2, CovSum, CovSumInv; //auxiliary matrices and vectors
    TVector<3> Vec1, Vec2, VecRes;

    CovSum = Cov1 + Cov2; //sum
    bOK = fMatr.InvMatrixSymmPos(CovSum, CovSumInv); //inverse sum
    if (!bOK)
    {
        bOK = fMatr.InvMatrix(CovSum, CovSumInv);
    }
    bRes = bRes && bOK;

    if (bRes) //inverse matrix is calculated
    {
        bOK = fMatr.MatrXMatr(Cov1, CovSumInv, Matr1); bRes = bRes && bOK; //Matr1 = Cov1*(Cov1+Cov2)^-1
        bOK = fMatr.MatrXMatr(Cov2, CovSumInv, Matr2); bRes = bRes && bOK; //Matr2 = Cov2*(Cov1+Cov2)^-1

        bOK = fMatr.MatrXVec(Matr2, P1vec, Vec1); bRes = bRes && bOK; //Vec1 = (Cov2*(Cov1+Cov2)^-1)*P1
        bOK = fMatr.MatrXVec(Matr1, P2vec, Vec2); bRes = bRes && bOK; //Vec2 = (Cov1*(Cov1+Cov2)^-1)*P2

        VecRes = Vec1 + Vec2; //VecRes = (Cov2*(Cov1+Cov2)^-1)*P1 + (Cov1*(Cov1+Cov2)^-1)*P2
        GLVectorToCoord(VecRes, PRes);
        bOK = fMatr.MatrXMatr(Matr2, Cov1, CovRes); bRes = bRes && bOK; //CovRes =  Cov2*((Cov1+Cov2)^-1)*Cov1
    }
    return bRes;
}


void GLCalcTrajParam::EstimateTimeEarthSurfaceIntersection(const double &X, const double &Y, const double &Z, const double &VX, const double &VY, const double &VZ, const double &AX, const double &AY, const double &AZ, GLComplexNumb &T1c, GLComplexNumb &T2c, GLComplexNumb &T3c, GLComplexNumb &T4c)
{
    T1c.Reset(-1.,-1.);
    T2c.Reset(-1.,-1.);
    T3c.Reset(-1.,-1.);
    T4c.Reset(-1.,-1.);

    double q = sqr(1. - con_Eath_compress_coeff);
    double p = sqr(con_Eath_equator_radius);

    double a, b, c, d, e;
    //coefficients of equation a*t^4+b*t^3+c*t^2+d*t+e=0 are obtained from expression h(t)=0,
    //where h=sqrt(x^2+y^2+z^2)-equator_radius*(1-compress_coeff)/sqrt((1-compress_coeff)^2+(2*compress_coeff-compress_coeff^2)*z^2/sqrt(x^2+y^2+z^2));
    //by converting the expression we obtain equation z^2+(1-compress_coeff)^2*(x^2+y^2-equator_radius^2)=0,
    //i.e. z^2+q*(x^2+y^2-p)=0.
    //Considering x(t)=x0+Vx0*t+ax0*t^2/2, y(t)=y0+Vy0*t+ay0*t^2/2, x(t)=z0+Vz0*t+az0*t^2/2,
    //we obtain coefficients a,b,c,d,e, calculated below

    a = 0.25 * (sqr(AZ) + q * (sqr(AX) + sqr(AY)));
    b = VZ * AZ +  q * (VX * AX + VY * AY);
    c = sqr(VZ) + q * (sqr(VX) + sqr(VY)) + Z * AZ + q * (X * AX + Y * AY);
    d = 2. * (Z * VZ +  q * (X * VX + Y * VY));
    e = sqr(Z) + q * (sqr(X) + sqr(Y)) - q*p;

    //solution of the equation a*t^4+b*t^3+c*t^2+d*t+e=0
    GLSolve_eq_2_3_4_deg fSolveEq;

    if (fabs(a) < con_par_eps)
    {
        if (fabs(b) < con_par_eps)
        {
            if (fabs(c) < con_par_eps)
            {
                if (fabs(d) < con_par_eps)
                {
                    if (fabs(e) < con_par_eps)
                    {
                        T1c.Reset(0,0);
                    }
                }
                else
                {
                    T1c.SetReal( - e/d);
                    T1c.SetImaginary(0);
                }
            }
            else
            {
                fSolveEq.Find_roots_2deg(d/c, e/c, T1c, T2c);
            }
        }
        else
        {
            fSolveEq.Find_roots_3deg(c/b, d/b, e/b, T1c, T2c, T3c);
        }
    }
    else
    {
        fSolveEq.Find_roots_4deg_Ferrari(b/a, c/a, d/a, e/a, T1c, T2c, T3c, T4c);
    }
}


void GLCalcTrajParam::EstimateTimeEarthSurfaceIntersection(const double &X, const double &Y, const double &Z, const double &VX, const double &VY, const double &VZ, GLComplexNumb &T1c, GLComplexNumb &T2c)
{
    T1c.Reset(-1.,-1.);
    T2c.Reset(-1.,-1.);

    double q = sqr(1. - con_Eath_compress_coeff);
    double p = sqr(con_Eath_equator_radius);

    double a, b, c;
    //coefficients of equation a*t^2+b*t+c=0 are obtained from expression h(t)=0,
    //where h=sqrt(x^2+y^2+z^2)-equator_radius*(1-compress_coeff)/sqrt((1-compress_coeff)^2+(2*compress_coeff-compress_coeff^2)*z^2/sqrt(x^2+y^2+z^2));
    //by converting the expression we obtain equation z^2+(1-compress_coeff)^2*(x^2+y^2-equator_radius^2)=0,
    //i.e. z^2+q*(x^2+y^2-p)=0.
    //Considering x(t)=x0+Vx0*t, y(t)=y0+Vy0*t, z(t)=z0+Vz0*t, we obtain coefficients a,b,c, calculated below

    a = sqr(VZ) + q * (sqr(VX) + sqr(VY));
    b = 2. * (Z * VZ + q * (X * VX + Y * VY));
    c = sqr(Z) + q * (sqr(X) + sqr(Y)) - q*p;

    //solution of the equation a*t^2+b*t+c=0
    GLSolve_eq_2_3_4_deg fSolveEq;
    if (fabs(a) < con_par_eps)
    {
        if (fabs(b) < con_par_eps)
        {
            if (fabs(c) < con_par_eps)
            {
                T1c.Reset(0,0);
            }
        }
        else
        {
            T1c.SetReal(-c/b);
            T1c.SetImaginary(0);
        }
    }
    else
    {
        fSolveEq.Find_roots_2deg(b/a, c/a, T1c, T2c);
    }
}


void GLCalcTrajParam::CalcS(double &Z, double &_r)
{
    if (_r > con_eps2)
    {
        double Denominator = pow(pow(1. - con_Eath_compress_coeff, 2) + (2.*con_Eath_compress_coeff - pow(con_Eath_compress_coeff, 2))*Z*Z/(_r*_r), 1.5);
        if (fabs(Denominator) > con_eps2)
        {
            S = (1. - con_Eath_compress_coeff) * (2.*con_Eath_compress_coeff - pow(con_Eath_compress_coeff, 2)) / Denominator;
        }
    }
}


void GLCalcTrajParam::CalcVr(double &X, double &Y, double &Z, double &VX, double &VY, double &VZ, double &_r)
{
    if (_r > con_eps2)
    {
        Vr = (X*VX + Y*VY + Z*VZ)/_r;
    }
}


EstimationMedian::EstimationMedian()
{
    clear();
}


void EstimationMedian::clear()
{
    this->Size = 0;
    this->ValuesSet.clear();
    this->MaxSize = 0;
    this->Median = 0;
    this->Lambda = 0;
}


void EstimationMedian::SetConstants(qint16 _DefaultSize, double _DefaultMedian, double _Lambda)
{
    this->MaxSize   = _DefaultSize;
    this->Median    = _DefaultMedian;
    this->Lambda    = _Lambda;
}


double EstimationMedian::GetMedian()
{
    return this->Median;
}


void EstimationMedian::AddValue(double _NewVal)
{
    qint16 i=0;
    if (Size < MaxSize)
    {
        Size++;
    }
    else
    {
        GLRandLib rnd;
        qint16 RandNum = static_cast<qint16>((fabs(rnd.Rand(0, 30000))));
        qint16 pos_del = RandNum % MaxSize;
        std::multiset<double>::iterator it_del = ValuesSet.begin();
        for (i=0; i<pos_del; i++)
        {
            it_del ++;
        }
        if (it_del != ValuesSet.end())
        {
            ValuesSet.erase(it_del);
        }
    }
    ValuesSet.insert(_NewVal);

    qint16 iMiddle = static_cast<qint16>(ceil(static_cast<double>(Size)/2.));

    std::multiset<double>::iterator it = ValuesSet.begin();
    for (i=0; i<iMiddle-1; i++)
    {
        it++;
    }
    if (it != ValuesSet.end())
    {
        double NewMedian = (*it);
        Median = Lambda*NewMedian + (1.-Lambda)*Median;
    }
}
