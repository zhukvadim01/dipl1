#ifndef AIR_BALL_AMOUNT_CONST_H
#define AIR_BALL_AMOUNT_CONST_H

#include <QtGlobal>

namespace AIR_BALL
{
    //const qint32   GT_FORM_AMOUNT    = 3500;      //total quantity of general tracks
    extern qint32   GT_FORM_AMOUNT;                 //total quantity of general tracks

//    const qint32   ST_FORM_AMOUNT    = 10000;     //total quantity of single tracks
    extern qint32   ST_FORM_AMOUNT;                 //total quantity of single tracks

//    const qint32   SRC_AMOUNT        = 200;       //total quantity of sources
    extern qint32   SRC_AMOUNT;                     //total quantity of sources

//    const qint32   ST_SRC_AMOUNT     = 1500;      //maximum trace number in the inner enumeration for source
    extern qint32   ST_SRC_AMOUNT;                  //maximum trace number in the inner enumeration for source

//    const qint32   BALL_FORM_AMOUNT  = 1000;      //total quantity of ballistic objects
    extern qint32   BALL_FORM_AMOUNT ;              //total quantity of ballistic objects


    const qint32   CLST_FORM_AMOUNT  = 200;         //maximum number of the clusters
    const qint32   BALL_LARGE_LOAD_AMOUNT = 100;    //quantity of ballistic object to large load definition

    const qint32   COVERED_OBJ_AMOUNT    = 200;     //maximum quantity of covered objects
    const qint16   N_ELEM_KAU_PARAM	= 50;           //Number of rows in Table of parameters of the end of active leg
    const qint16   N_BALL_SUBCL    	= 25;           //Number of Ballistic subclasses
    const qint16   N_H_DIAP_GAMMA      = 6;         //Number of diapasons by H to determine ballistic coefficient by table
    const qint16   N_MACH_DIAP_GAMMA   = 45;        //Number of diapasons by Mach number to determine ballistic coefficient by table
    const qint16   N_OUT_POINTS        = 50;        //Number of read-out points of trajectory
    const qint16   NUMB_BM_MARKS = 24;              //number of BT marks
    const qint16   NUMB_CELLS = 11725;              //number of cells in the array defining the correspondence between the cells on the plane "H-He" and the BT marks
    const qint16   INP_MSGS_AMOUNT     = 100;       //Maximum number of input messages to Air Ballist tack
}

#endif // AIR_BALL_AMOUNT_CONST_H
