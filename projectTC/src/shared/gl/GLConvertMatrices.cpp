#include "GLConvertMatrices.h"

namespace GL_MATR
{
    void MatrMatching_3x3(TMatrix<3> &M_inp, Square_Matrix<3> &M_out)
    {
        M_out.Reset(M_inp.s_m, M_inp.s_n);

        for (int i=0; i< M_inp.s_m; i++)
            for (int j=0; j<M_inp.s_n; j++)
                M_out.M[i][j] = M_inp.M[i][j];

        return;
    }

    void MatrMatching_3x3(Square_Matrix<3> &M_inp, TMatrix<3> &M_out)
    {
        M_out.Reset(M_inp.s_m, M_inp.s_n);

        for (int i=0; i< M_inp.s_m; i++)
            for (int j=0; j<M_inp.s_n; j++)
                M_out.M[i][j] = M_inp.M[i][j];

        return;
    }

    void MatrMatching_6x6(TMatrix<6> &M_inp, Square_Matrix<6> &M_out)
    {
        M_out.Reset(M_inp.s_m, M_inp.s_n);

        for (int i=0; i< M_inp.s_m; i++)
            for (int j=0; j<M_inp.s_n; j++)
                M_out.M[i][j] = M_inp.M[i][j];

        return;
    }

    void MatrMatching_6x6(Square_Matrix<6> &M_inp, TMatrix<6> &M_out)
    {
        M_out.Reset(M_inp.s_m, M_inp.s_n);

        for (int i=0; i< M_inp.s_m; i++)
            for (int j=0; j<M_inp.s_n; j++)
                M_out.M[i][j] = M_inp.M[i][j];

        return;
    }

    void MatrMatching_9x9(TMatrix<9> &M_inp, Square_Matrix<9> &M_out)
    {
        M_out.Reset(M_inp.s_m, M_inp.s_n);

        for (int i=0; i< M_inp.s_m; i++)
            for (int j=0; j<M_inp.s_n; j++)
                M_out.M[i][j] = M_inp.M[i][j];

        return;
    }

    void MatrMatching_9x9(Square_Matrix<9> &M_inp, TMatrix<9> &M_out)
    {
        M_out.Reset(M_inp.s_m, M_inp.s_n);

        for (int i=0; i< M_inp.s_m; i++)
            for (int j=0; j<M_inp.s_n; j++)
                M_out.M[i][j] = M_inp.M[i][j];

        return;
    }


    bool MatrMatching_SIZE_Mto3(GLMatrix &M_inp, Square_Matrix<3> &M_out)
    {
        const int dim = 3;
        if (M_inp.s_m != M_inp.s_n || M_inp.s_m != dim)
            return false;

        M_out.Reset(dim, dim);
        for (int i=0; i<dim; i++)
            for (int j=0; j<dim; j++)
                M_out.M[i][j] = M_inp.M[i][j];
        return true;
    }


    bool MatrMatching_3toSIZE_M(Square_Matrix<3> &M_inp, GLMatrix &M_out)
    {
        const int dim = 3;
        if (M_inp.s_m != M_inp.s_n || M_inp.s_m != dim)
            return false;

        M_out.Reset(dim, dim);
        for (int i=0; i<dim; i++)
            for (int j=0; j<dim; j++)
                M_out.M[i][j] = M_inp.M[i][j];
        return true;
    }


    bool MatrMatching_SIZE_Mto6(GLMatrix &M_inp, Square_Matrix<6> &M_out)
    {
        const int dim = 6;
        if (M_inp.s_m != M_inp.s_n || M_inp.s_m != dim)
            return false;

        M_out.Reset(dim, dim);
        for (int i=0; i<dim; i++)
            for (int j=0; j<dim; j++)
                M_out.M[i][j] = M_inp.M[i][j];
        return true;
    }


    bool MatrMatching_6toSIZE_M(Square_Matrix<6> &M_inp, GLMatrix &M_out)
    {
        const int dim = 6;
        if (M_inp.s_m != M_inp.s_n || M_inp.s_m != dim)
            return false;

        M_out.Reset(dim, dim);
        for (int i=0; i<dim; i++)
            for (int j=0; j<dim; j++)
                M_out.M[i][j] = M_inp.M[i][j];
        return true;
    }


    bool MatrMatching_SIZE_Mto9(GLMatrix &M_inp, Square_Matrix<9> &M_out)
    {
        const int dim = 9;
        if (M_inp.s_m != M_inp.s_n || M_inp.s_m != dim)
            return false;

        M_out.Reset(dim, dim);
        for (int i=0; i<dim; i++)
            for (int j=0; j<dim; j++)
                M_out.M[i][j] = M_inp.M[i][j];
        return true;
    }


    bool MatrMatching_9toSIZE_M(Square_Matrix<9> &M_inp, GLMatrix &M_out)
    {
        const int dim = 9;
        if (M_inp.s_m != M_inp.s_n || M_inp.s_m != dim)
            return false;

        M_out.Reset(dim, dim);
        for (int i=0; i<dim; i++)
            for (int j=0; j<dim; j++)
                M_out.M[i][j] = M_inp.M[i][j];
        return true;
    }


    bool Recount_CovMatr_GeocentricToTopo(CGeodesic *pGEO, TMatrix<3> *CM_Geo, TMatrix<3> *CM_NUE)
    {
        Square_Matrix<3> CM_Geo_sq, CM_NUE_sq;
        MatrMatching_3x3(*CM_Geo, CM_Geo_sq);
        bool ItsOK = Recount_CovMatr_GeocentricToTopo(pGEO, &CM_Geo_sq, &CM_NUE_sq);
        if (!ItsOK) return false;
        MatrMatching_3x3(CM_NUE_sq, *CM_NUE);
        return true;
    }

    bool Recount_CovMatr_GeocentricToTopo(CGeodesic *pGEO, TMatrix<6> *CM_Geo, TMatrix<6> *CM_NUE)
    {
        Square_Matrix<6> CM_Geo_sq, CM_NUE_sq;
        MatrMatching_6x6(*CM_Geo, CM_Geo_sq);
        bool ItsOK = Recount_CovMatr_GeocentricToTopo(pGEO, &CM_Geo_sq, &CM_NUE_sq);
        if (!ItsOK) return false;
        MatrMatching_6x6(CM_NUE_sq, *CM_NUE);
        return true;
    }

    bool Recount_CovMatr_GeocentricToTopo(CGeodesic *pGEO, TMatrix<9> *CM_Geo, TMatrix<9> *CM_NUE)
    {
        Square_Matrix<9> CM_Geo_sq, CM_NUE_sq;
        MatrMatching_9x9(*CM_Geo, CM_Geo_sq);
        bool ItsOK = Recount_CovMatr_GeocentricToTopo(pGEO, &CM_Geo_sq, &CM_NUE_sq);
        if (!ItsOK) return false;
        MatrMatching_9x9(CM_NUE_sq, *CM_NUE);
        return true;
    }


    bool Recount_CovMatr_GeocentricToTopo(CGeodesic *pGEO, GLMatrix *CM_Geo, GLMatrix *CM_NUE)
    {
        int dim = CM_Geo->s_m;
        if (CM_Geo->s_n != dim)
            return false;
        bool ItsOK;

        if (dim == 3)
        {
            const int c_dim = 3;
            Square_Matrix<c_dim> CM_Geo_dim(c_dim,c_dim), CM_NUE_dim(c_dim,c_dim);
            MatrMatching_SIZE_Mto3(*CM_Geo, CM_Geo_dim);
            ItsOK = Recount_CovMatr_GeocentricToTopo(pGEO, &CM_Geo_dim, &CM_NUE_dim);
            if (!ItsOK) return false;
            MatrMatching_3toSIZE_M(CM_NUE_dim, *CM_NUE);
        }
        else if (dim == 6)
        {
            const int c_dim = 6;
            Square_Matrix<c_dim> CM_Geo_dim(c_dim,c_dim), CM_NUE_dim(c_dim,c_dim);
            MatrMatching_SIZE_Mto6(*CM_Geo, CM_Geo_dim);
            ItsOK = Recount_CovMatr_GeocentricToTopo(pGEO, &CM_Geo_dim, &CM_NUE_dim);
            if (!ItsOK) return false;
            MatrMatching_6toSIZE_M(CM_NUE_dim, *CM_NUE);
        }
        else if (dim == 9)
        {
            const int c_dim = 9;
            Square_Matrix<c_dim> CM_Geo_dim(c_dim,c_dim), CM_NUE_dim(c_dim,c_dim);
            MatrMatching_SIZE_Mto9(*CM_Geo, CM_Geo_dim);
            ItsOK = Recount_CovMatr_GeocentricToTopo(pGEO, &CM_Geo_dim, &CM_NUE_dim);
            if (!ItsOK) return false;
            MatrMatching_9toSIZE_M(CM_NUE_dim, *CM_NUE);
        }
        else
            return false;

        return true;
    }


    bool Recount_CovMatr_TopoToGeocentric(CGeodesic *pGEO, TMatrix<3> *CM_NUE, TMatrix<3> *CM_Geo)
    {
        Square_Matrix<3> CM_Geo_sq, CM_NUE_sq;
        MatrMatching_3x3(*CM_NUE, CM_NUE_sq);
        bool ItsOK = Recount_CovMatr_TopoToGeocentric(pGEO, &CM_NUE_sq, &CM_Geo_sq);
        if (!ItsOK) return false;
        MatrMatching_3x3(CM_Geo_sq, *CM_Geo);
        return true;
    }

    bool Recount_CovMatr_TopoToGeocentric(CGeodesic *pGEO, TMatrix<6> *CM_NUE, TMatrix<6> *CM_Geo)
    {
        Square_Matrix<6> CM_Geo_sq, CM_NUE_sq;
        MatrMatching_6x6(*CM_NUE, CM_NUE_sq);
        bool ItsOK = Recount_CovMatr_TopoToGeocentric(pGEO, &CM_NUE_sq, &CM_Geo_sq);
        if (!ItsOK) return false;
        MatrMatching_6x6(CM_Geo_sq, *CM_Geo);
        return true;
    }

    bool Recount_CovMatr_TopoToGeocentric(CGeodesic *pGEO, TMatrix<9> *CM_NUE, TMatrix<9> *CM_Geo)
    {
        Square_Matrix<9> CM_Geo_sq, CM_NUE_sq;
        MatrMatching_9x9(*CM_NUE, CM_NUE_sq);
        bool ItsOK = Recount_CovMatr_TopoToGeocentric(pGEO, &CM_NUE_sq, &CM_Geo_sq);
        if (!ItsOK) return false;
        MatrMatching_9x9(CM_Geo_sq, *CM_Geo);
        return true;
    }

    bool Recount_CovMatr_TopoToGeocentric(CGeodesic *pGEO, GLMatrix *CM_NUE, GLMatrix *CM_Geo)
    {
        int dim = CM_NUE->s_m;
        if (CM_NUE->s_n != dim)
            return false;
        bool ItsOK;

        if (dim == 3)
        {
            const int c_dim = 3;
            Square_Matrix<c_dim> CM_Geo_dim(c_dim,c_dim), CM_NUE_dim(c_dim,c_dim);

            MatrMatching_SIZE_Mto3(*CM_NUE , CM_NUE_dim);
            ItsOK = Recount_CovMatr_TopoToGeocentric(pGEO, &CM_NUE_dim, &CM_Geo_dim);
            if (!ItsOK) return false;
            MatrMatching_3toSIZE_M(CM_Geo_dim, *CM_Geo);
        }
        else if (dim == 6)
        {
            const int c_dim = 6;
            Square_Matrix<c_dim> CM_Geo_dim(c_dim,c_dim), CM_NUE_dim(c_dim,c_dim);
            MatrMatching_SIZE_Mto6(*CM_NUE, CM_NUE_dim);
            ItsOK = Recount_CovMatr_TopoToGeocentric(pGEO, &CM_NUE_dim, &CM_Geo_dim);
            if (!ItsOK) return false;
            MatrMatching_6toSIZE_M(CM_Geo_dim, *CM_Geo );
        }
        else if (dim == 9)
        {
            const int c_dim = 9;
            Square_Matrix<c_dim> CM_Geo_dim(c_dim,c_dim), CM_NUE_dim(c_dim,c_dim);
            MatrMatching_SIZE_Mto9(*CM_NUE, CM_NUE_dim);
            ItsOK = Recount_CovMatr_TopoToGeocentric(pGEO, &CM_NUE_dim, &CM_Geo_dim);
            if (!ItsOK) return false;
            MatrMatching_9toSIZE_M(CM_Geo_dim, *CM_Geo );
        }
        else
            return false;

        return true;
    }

    bool Recount_CovMatr_NUEtoSPHCS_coord(CTopocentric *Coord_TOPO,TMatrix<3> *CM_NUE, TMatrix<3> *CM_SPH)
    {
        Square_Matrix<3> CM_SPH_sq, CM_NUE_sq;
        MatrMatching_3x3(*CM_NUE, CM_NUE_sq);
        bool ItsOK = Recount_CovMatr_NUEtoSPHCS_coord(Coord_TOPO, &CM_NUE_sq, &CM_SPH_sq);
        if (!ItsOK) return false;
        MatrMatching_3x3(CM_SPH_sq, *CM_SPH);
        return true;
    }

    bool Recount_CovMatr_NUEtoSPHCS_coord(CTopocentric *Coord_TOPO,GLMatrix *CM_NUE, GLMatrix *CM_SPH)
    {
        int dim = CM_NUE->s_m;
        if (CM_NUE->s_n != dim)
            return false;

        const int c_dim = 3;
        if (dim != c_dim)
            return false;
        Square_Matrix<c_dim> CM_SPH_dim(c_dim,c_dim), CM_NUE_dim(c_dim,c_dim);
        MatrMatching_SIZE_Mto3(*CM_NUE, CM_NUE_dim);
        bool ItsOK = Recount_CovMatr_NUEtoSPHCS_coord(Coord_TOPO, &CM_NUE_dim, &CM_SPH_dim);
        if (!ItsOK) return false;
        MatrMatching_3toSIZE_M(CM_SPH_dim, *CM_SPH);
        return true;
    }

    bool Recount_CovMatr_NUEtoSPHCS_coord_vel(CTopocentric *Coord_TOPO,CTopocentric *Vel_TOPO,TMatrix<6> *CM_NUE, TMatrix<6> *CM_SPH)
    {
        Square_Matrix<6> CM_SPH_sq, CM_NUE_sq;
        MatrMatching_6x6(*CM_NUE, CM_NUE_sq);
        bool ItsOK = Recount_CovMatr_NUEtoSPHCS_coord_vel(Coord_TOPO,Vel_TOPO, &CM_NUE_sq, &CM_SPH_sq);
        if (!ItsOK) return false;
        MatrMatching_6x6(CM_SPH_sq, *CM_SPH);
        return true;
    }

    bool Recount_CovMatr_NUEtoSPHCS_coord_vel(CTopocentric *Coord_TOPO,CTopocentric *Vel_TOPO,GLMatrix *CM_NUE, GLMatrix *CM_SPH)
    {
        int dim = CM_NUE->s_m;
        if (CM_NUE->s_n != dim)
            return false;

        const int c_dim = 6;
        if (dim != c_dim)
            return false;
        Square_Matrix<c_dim> CM_SPH_dim(c_dim,c_dim), CM_NUE_dim(c_dim,c_dim);
        MatrMatching_SIZE_Mto6(*CM_NUE, CM_NUE_dim);
        bool ItsOK = Recount_CovMatr_NUEtoSPHCS_coord_vel(Coord_TOPO,Vel_TOPO, &CM_NUE_dim, &CM_SPH_dim);
        if (!ItsOK) return false;
        MatrMatching_6toSIZE_M(CM_SPH_dim, *CM_SPH);
        return true;
    }

    bool Recount_CovMatr_NUEtoSPHCS_coord_vel_accel(CTopocentric *Coord_TOPO,CTopocentric *Vel_TOPO,CTopocentric *Acc_TOPO,
                                                                TMatrix<9> *CM_NUE, TMatrix<9> *CM_SPH)
    {
        Square_Matrix<9> CM_SPH_sq, CM_NUE_sq;
        MatrMatching_9x9(*CM_NUE, CM_NUE_sq);
        bool ItsOK = Recount_CovMatr_NUEtoSPHCS_coord_vel_accel(Coord_TOPO,Vel_TOPO,Acc_TOPO, &CM_NUE_sq, &CM_SPH_sq);
        if (!ItsOK) return false;
        MatrMatching_9x9(CM_SPH_sq, *CM_SPH);
        return true;
    }

    bool Recount_CovMatr_NUEtoSPHCS_coord_vel_accel(CTopocentric *Coord_TOPO,CTopocentric *Vel_TOPO,CTopocentric *Acc_TOPO,
                                                                GLMatrix *CM_NUE, GLMatrix *CM_SPH)
    {
        int dim = CM_NUE->s_m;
        if (CM_NUE->s_n != dim)
            return false;

        const int c_dim = 9;
        if (dim != c_dim)
            return false;
        Square_Matrix<c_dim> CM_SPH_dim(c_dim,c_dim), CM_NUE_dim(c_dim,c_dim);
        MatrMatching_SIZE_Mto9(*CM_NUE, CM_NUE_dim);
        bool ItsOK = Recount_CovMatr_NUEtoSPHCS_coord_vel_accel(Coord_TOPO,Vel_TOPO,Acc_TOPO, &CM_NUE_dim, &CM_SPH_dim);
        if (!ItsOK) return false;
        MatrMatching_9toSIZE_M(CM_SPH_dim, *CM_SPH);
        return true;
    }

    bool Recount_CovMatr_SPHCStoNUEcoord(CSpherical *Coord_Sf, TMatrix<3> *CM_SPH, TMatrix<3> *CM_NUE)
    {
        Square_Matrix<3> CM_SPH_sq, CM_NUE_sq;
        MatrMatching_3x3(*CM_SPH, CM_SPH_sq);
        bool ItsOK = Recount_CovMatr_SPHCStoNUEcoord(Coord_Sf, &CM_SPH_sq, &CM_NUE_sq);
        if (!ItsOK) return false;
        MatrMatching_3x3(CM_NUE_sq, *CM_NUE);
        return true;
    }

     bool Recount_CovMatr_SPHCStoNUEcoord(CSpherical *Coord_Sf, GLMatrix *CM_SPH, GLMatrix *CM_NUE)
     {
         int dim = CM_SPH->s_m;
         if (CM_SPH->s_n != dim)
             return false;

         const int c_dim = 3;
         if (dim != c_dim)
             return false;
         Square_Matrix<c_dim> CM_SPH_dim(c_dim,c_dim), CM_NUE_dim(c_dim,c_dim);
         MatrMatching_SIZE_Mto3(*CM_SPH, CM_SPH_dim);
         bool ItsOK = Recount_CovMatr_SPHCStoNUEcoord(Coord_Sf, &CM_SPH_dim, &CM_NUE_dim);
         if (!ItsOK) return false;
         MatrMatching_3toSIZE_M(CM_NUE_dim, *CM_NUE);
         return true;
     }

    bool Recount_CovMatr_SPHCStoNUEcoord_vel(CSpherical *Coord_Sf,CSpherical *Vel_Sf, TMatrix<6> *CM_SPH, TMatrix<6> *CM_NUE)
    {
        Square_Matrix<6> CM_SPH_sq, CM_NUE_sq;
        MatrMatching_6x6(*CM_SPH, CM_SPH_sq);
        bool ItsOK = Recount_CovMatr_SPHCStoNUEcoord_vel(Coord_Sf,Vel_Sf, &CM_SPH_sq, &CM_NUE_sq);
        if (!ItsOK) return false;
        MatrMatching_6x6(CM_NUE_sq, *CM_NUE);
        return true;
    }

    bool Recount_CovMatr_SPHCStoNUEcoord_vel(CSpherical *Coord_Sf,CSpherical *Vel_Sf, GLMatrix *CM_SPH, GLMatrix *CM_NUE)
    {
        int dim = CM_SPH->s_m;
        if (CM_SPH->s_n != dim)
            return false;

        const int c_dim = 6;
        if (dim != c_dim)
            return false;
        Square_Matrix<c_dim> CM_SPH_dim(c_dim,c_dim), CM_NUE_dim(c_dim,c_dim);
        MatrMatching_SIZE_Mto6(*CM_SPH, CM_SPH_dim);
        bool ItsOK = Recount_CovMatr_SPHCStoNUEcoord_vel(Coord_Sf,Vel_Sf, &CM_SPH_dim, &CM_NUE_dim);
        if (!ItsOK) return false;
        MatrMatching_6toSIZE_M(CM_NUE_dim, *CM_NUE);
        return true;
    }

    bool Recount_CovMatr_SPHCStoNUEcoord_vel_acc(CSpherical *Coord_Sf,CSpherical *Vel_Sf,CSpherical *Acc_Sf,
                                                             TMatrix<9> *CM_SPH, TMatrix<9> *CM_NUE)
    {
        Square_Matrix<9> CM_SPH_sq, CM_NUE_sq;
        MatrMatching_9x9(*CM_SPH, CM_SPH_sq);
        bool ItsOK = Recount_CovMatr_SPHCStoNUEcoord_vel_acc(Coord_Sf,Vel_Sf,Acc_Sf, &CM_SPH_sq, &CM_NUE_sq);
        if (!ItsOK) return false;
        MatrMatching_9x9(CM_NUE_sq, *CM_NUE);
        return true;
    }

    bool Recount_CovMatr_SPHCStoNUEcoord_vel_acc(CSpherical *Coord_Sf,CSpherical *Vel_Sf,CSpherical *Acc_Sf,
                                                            GLMatrix *CM_SPH,GLMatrix *CM_NUE)
    {
        int dim = CM_SPH->s_m;
        if (CM_SPH->s_n != dim)
            return false;

        const int c_dim = 9;
        if (dim != c_dim)
            return false;
        Square_Matrix<c_dim> CM_SPH_dim(c_dim,c_dim), CM_NUE_dim(c_dim,c_dim);
        MatrMatching_SIZE_Mto9(*CM_SPH, CM_SPH_dim);
        bool ItsOK = Recount_CovMatr_SPHCStoNUEcoord_vel_acc(Coord_Sf,Vel_Sf,Acc_Sf, &CM_SPH_dim, &CM_NUE_dim);
        if (!ItsOK) return false;
        MatrMatching_9toSIZE_M(CM_NUE_dim, *CM_NUE);
        return true;
    }

    bool Recount_CovMatr_TopoToTopo(CGeodesic *pGEO_1,CGeodesic *pGEO_2, TMatrix<3> *CM_NUE_1, TMatrix<3> *CM_NUE_2){
        int dim = CM_NUE_1->s_m;
        if (CM_NUE_1->s_n != dim)
            return false;

        const int c_dim = 3;
        if (dim != c_dim)
            return false;
        Square_Matrix<c_dim> CM_NUE_1_dim(c_dim,c_dim), CM_NUE_2_dim(c_dim,c_dim);

        SKoefRecal pRecal;
        if(!InitTopo1_Topo2_SHIFT(pGEO_1, pGEO_2, &pRecal))
            return false;

        Square_Matrix<c_dim> Matr;
        Matr.s_m=c_dim;
        Matr.s_n=c_dim;

        Matr.M[0][0]=pRecal.m_dKx1;
        Matr.M[0][1]=pRecal.m_dKx2;
        Matr.M[0][2]=pRecal.m_dKx3;

        Matr.M[1][0]=pRecal.m_dKy1;
        Matr.M[1][1]=pRecal.m_dKy2;
        Matr.M[1][2]=pRecal.m_dKy3;

        Matr.M[2][0]=pRecal.m_dKz1;
        Matr.M[2][1]=pRecal.m_dKz2;
        Matr.M[2][2]=pRecal.m_dKz3;

        Square_Matrix<c_dim> Matr_tr;
        Matr.transp(&Matr_tr);

        MatrMatching_3x3(*CM_NUE_1, CM_NUE_1_dim);

        Matr.MatrXMatrXMatr(&CM_NUE_1_dim,&Matr_tr,&CM_NUE_2_dim);

        MatrMatching_3x3(CM_NUE_2_dim, *CM_NUE_2);
        return true;



    }

    bool Recount_CovMatr_TopoToTopo(CGeodesic *pGEO_1,CGeodesic *pGEO_2, TMatrix<6> *CM_NUE_1, TMatrix<6> *CM_NUE_2){
        int dim = CM_NUE_1->s_m;
        if (CM_NUE_1->s_n != dim)
            return false;

        const int c_dim = 6;
        if (dim != c_dim)
            return false;
        Square_Matrix<c_dim> CM_NUE_1_dim(c_dim,c_dim), CM_NUE_2_dim(c_dim,c_dim);

        SKoefRecal pRecal;
        if(!InitTopo1_Topo2_SHIFT(pGEO_1, pGEO_2, &pRecal))
            return false;

        Square_Matrix<c_dim> Matr;
        Matr.s_m=c_dim;
        Matr.s_n=c_dim;

        Matr.M[0][0]=pRecal.m_dKx1;
        Matr.M[0][1]=pRecal.m_dKx2;
        Matr.M[0][2]=pRecal.m_dKx3;

        Matr.M[1][0]=pRecal.m_dKy1;
        Matr.M[1][1]=pRecal.m_dKy2;
        Matr.M[1][2]=pRecal.m_dKy3;

        Matr.M[2][0]=pRecal.m_dKz1;
        Matr.M[2][1]=pRecal.m_dKz2;
        Matr.M[2][2]=pRecal.m_dKz3;

        Matr.M[3][3]=pRecal.m_dKx1;
        Matr.M[3][4]=pRecal.m_dKx2;
        Matr.M[3][5]=pRecal.m_dKx3;

        Matr.M[4][3]=pRecal.m_dKy1;
        Matr.M[4][4]=pRecal.m_dKy2;
        Matr.M[4][5]=pRecal.m_dKy3;

        Matr.M[5][3]=pRecal.m_dKz1;
        Matr.M[5][4]=pRecal.m_dKz2;
        Matr.M[5][5]=pRecal.m_dKz3;

        Square_Matrix<c_dim> Matr_tr;
        Matr.transp(&Matr_tr);

        MatrMatching_6x6(*CM_NUE_1, CM_NUE_1_dim);

        Matr.MatrXMatrXMatr(&CM_NUE_1_dim,&Matr_tr,&CM_NUE_2_dim);

        MatrMatching_6x6(CM_NUE_2_dim, *CM_NUE_2);
        return true;
    }

    bool Recount_CovMatr_TopoToTopo(CGeodesic *pGEO_1,CGeodesic *pGEO_2, TMatrix<9> *CM_NUE_1, TMatrix<9> *CM_NUE_2){

        int dim = CM_NUE_1->s_m;
        if (CM_NUE_1->s_n != dim)
            return false;

        const int c_dim = 9;
        if (dim != c_dim)
            return false;
        Square_Matrix<c_dim> CM_NUE_1_dim(c_dim,c_dim), CM_NUE_2_dim(c_dim,c_dim);

        SKoefRecal pRecal;
        if(!InitTopo1_Topo2_SHIFT(pGEO_1, pGEO_2, &pRecal))
            return false;

        Square_Matrix<c_dim> Matr;
        Matr.s_m=c_dim;
        Matr.s_n=c_dim;

        Matr.M[0][0]=pRecal.m_dKx1;
        Matr.M[0][1]=pRecal.m_dKx2;
        Matr.M[0][2]=pRecal.m_dKx3;

        Matr.M[1][0]=pRecal.m_dKy1;
        Matr.M[1][1]=pRecal.m_dKy2;
        Matr.M[1][2]=pRecal.m_dKy3;

        Matr.M[2][0]=pRecal.m_dKz1;
        Matr.M[2][1]=pRecal.m_dKz2;
        Matr.M[2][2]=pRecal.m_dKz3;

        Matr.M[3][3]=pRecal.m_dKx1;
        Matr.M[3][4]=pRecal.m_dKx2;
        Matr.M[3][5]=pRecal.m_dKx3;

        Matr.M[4][3]=pRecal.m_dKy1;
        Matr.M[4][4]=pRecal.m_dKy2;
        Matr.M[4][5]=pRecal.m_dKy3;

        Matr.M[5][3]=pRecal.m_dKz1;
        Matr.M[5][4]=pRecal.m_dKz2;
        Matr.M[5][5]=pRecal.m_dKz3;

        Matr.M[6][6]=pRecal.m_dKx1;
        Matr.M[6][7]=pRecal.m_dKx2;
        Matr.M[6][8]=pRecal.m_dKx3;

        Matr.M[7][6]=pRecal.m_dKy1;
        Matr.M[7][7]=pRecal.m_dKy2;
        Matr.M[7][8]=pRecal.m_dKy3;

        Matr.M[8][6]=pRecal.m_dKz1;
        Matr.M[8][7]=pRecal.m_dKz2;
        Matr.M[8][8]=pRecal.m_dKz3;

        Square_Matrix<c_dim> Matr_tr;
        Matr.transp(&Matr_tr);

        MatrMatching_9x9(*CM_NUE_1, CM_NUE_1_dim);

        Matr.MatrXMatrXMatr(&CM_NUE_1_dim,&Matr_tr,&CM_NUE_2_dim);

        MatrMatching_9x9(CM_NUE_2_dim, *CM_NUE_2);
        return true;
    }

    bool Recount_CovMatr_TopoToTopo(CGeodesic *pGEO_1,CGeodesic *pGEO_2, GLMatrix *CM_NUE_1, GLMatrix *CM_NUE_2){
        int dim = CM_NUE_1->s_m;
        if (CM_NUE_1->s_n != dim)
            return false;

        if(dim==3){
            const int c_dim = 3;
             TMatrix<c_dim> CM_NUE_1_T(c_dim,c_dim), CM_NUE_2_T(c_dim,c_dim);
             Square_Matrix<c_dim> CM_NUE_1_Sq(c_dim,c_dim), CM_NUE_2_Sq(c_dim,c_dim);
             MatrMatching_SIZE_Mto3(*CM_NUE_1, CM_NUE_1_Sq);
             MatrMatching_3x3(CM_NUE_1_Sq, CM_NUE_1_T);
             bool ItsOK =Recount_CovMatr_TopoToTopo(pGEO_1,pGEO_2, &CM_NUE_1_T, &CM_NUE_2_T);
             if (!ItsOK) return false;
             MatrMatching_3x3(CM_NUE_2_T, CM_NUE_2_Sq);
             MatrMatching_3toSIZE_M(CM_NUE_2_Sq,*CM_NUE_2);
             return true;
        }
        if(dim==6){
            const int c_dim = 6;
             TMatrix<c_dim> CM_NUE_1_T(c_dim,c_dim), CM_NUE_2_T(c_dim,c_dim);
             Square_Matrix<c_dim> CM_NUE_1_Sq(c_dim,c_dim), CM_NUE_2_Sq(c_dim,c_dim);
             MatrMatching_SIZE_Mto6(*CM_NUE_1, CM_NUE_1_Sq);
             MatrMatching_6x6(CM_NUE_1_Sq, CM_NUE_1_T);
             bool ItsOK =Recount_CovMatr_TopoToTopo(pGEO_1,pGEO_2, &CM_NUE_1_T, &CM_NUE_2_T);
             if (!ItsOK) return false;
             MatrMatching_6x6(CM_NUE_2_T, CM_NUE_2_Sq);
             MatrMatching_6toSIZE_M(CM_NUE_2_Sq,*CM_NUE_2);
             return true;
        }
        if(dim==9){
            const int c_dim = 9;
             TMatrix<c_dim> CM_NUE_1_T(c_dim,c_dim), CM_NUE_2_T(c_dim,c_dim);
             Square_Matrix<c_dim> CM_NUE_1_Sq(c_dim,c_dim), CM_NUE_2_Sq(c_dim,c_dim);
             MatrMatching_SIZE_Mto9(*CM_NUE_1, CM_NUE_1_Sq);
             MatrMatching_9x9(CM_NUE_1_Sq, CM_NUE_1_T);
             bool ItsOK =Recount_CovMatr_TopoToTopo(pGEO_1,pGEO_2, &CM_NUE_1_T, &CM_NUE_2_T);
             if (!ItsOK) return false;
             MatrMatching_9x9(CM_NUE_2_T, CM_NUE_2_Sq);
             MatrMatching_9toSIZE_M(CM_NUE_2_Sq,*CM_NUE_2);
             return true;
        }
        return false;

    }
    bool Recount_TVec_GeocentricToTopo(CGeodesic *pGEO, TVector<SIZE_M> *CM_Geo, TVector<SIZE_M> *CM_Topo)
    {
        CGeocentric xg(CM_Geo->Vec[0],CM_Geo->Vec[1],CM_Geo->Vec[2]);
        CGeocentric vg(CM_Geo->Vec[3],CM_Geo->Vec[4],CM_Geo->Vec[5]);
        CGeocentric ag(CM_Geo->Vec[6],CM_Geo->Vec[7],CM_Geo->Vec[8]);
        CTopocentric xt, vt, at;
        bool ItsOK = GEOCENTRIC_TOPO(pGEO,&xg,&vg,&ag,&xt,&vt,&at);
        CM_Topo->Vec[0] = xt.m_dXt;
        CM_Topo->Vec[1] = xt.m_dYt;
        CM_Topo->Vec[2] = xt.m_dZt;
        CM_Topo->Vec[3] = vt.m_dXt;
        CM_Topo->Vec[4] = vt.m_dYt;
        CM_Topo->Vec[5] = vt.m_dZt;
        CM_Topo->Vec[6] = at.m_dXt;
        CM_Topo->Vec[7] = at.m_dYt;
        CM_Topo->Vec[8] = at.m_dZt;
        CM_Topo->s = 9;
        return ItsOK;
    }
    bool Recount_TVec_TopocentricToGeo(CGeodesic *pGEO, TVector<SIZE_M> *CM_Topo, TVector<SIZE_M> *CM_Geo)
    {
        CTopocentric xt(CM_Topo->Vec[0],CM_Topo->Vec[1],CM_Topo->Vec[2]);
        CTopocentric vt(CM_Topo->Vec[3],CM_Topo->Vec[4],CM_Topo->Vec[5]);
        CTopocentric at(CM_Topo->Vec[6],CM_Topo->Vec[7],CM_Topo->Vec[8]);
        CGeocentric xg, vg, ag;
        bool ItsOK = TOPO_GEOCENTRIC(pGEO,&xt,&vt,&at,&xg,&vg,&ag);
        CM_Geo->Vec[0] = xg.m_dX;
        CM_Geo->Vec[1] = xg.m_dY;
        CM_Geo->Vec[2] = xg.m_dZ;
        CM_Geo->Vec[3] = vg.m_dX;
        CM_Geo->Vec[4] = vg.m_dY;
        CM_Geo->Vec[5] = vg.m_dZ;
        CM_Geo->Vec[6] = ag.m_dX;
        CM_Geo->Vec[7] = ag.m_dY;
        CM_Geo->Vec[8] = ag.m_dZ;
        CM_Geo->s = 9;
        return ItsOK;
    }
}