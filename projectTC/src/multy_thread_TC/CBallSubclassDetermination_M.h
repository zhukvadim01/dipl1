#ifndef CBALLSUBCLASSDETERMINATIONM_H
#define CBALLSUBCLASSDETERMINATIONM_H

#include "CBallSubclassDet_structs.h"
#include "ball_amount_const.h"

#include "gl/GLArray.h"

//PACKAGE       :   AirBallist
//CLASS         :   CBallSubclassDetermination
//DESCRIPTION	:   Realizes the functions to determine a BT subclass and trajectory type (optimal, high, low)
class CBallSubclassDetermination_M
{
    friend class CAirBallist_Processing_M;

public:
    CBallSubclassDetermination_M();
    ~CBallSubclassDetermination_M();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CBallSubclassDetermination::Reset
    // DESCRIPTION	:   Reset of class attributes
    // INPUTS		:	None
    // RETURNS		:	None
    void Reset();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CBallSubclassDetermination::TrajTypeDet_Realization
    // DESCRIPTION	:   Main part of trajectory type determination algorithm
    // INPUTS		:	None
    // RETURNS		:	None
    void TrajTypeDet_Realization(const TrajType_InputData *pInData,
                                 TrajType_OutputData &rOutData);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CBallSubclassDetermination::BMMarkDet_Realization
    // DESCRIPTION	:   Main part of BT mark determination algorithm
    // INPUTS		:	None
    // RETURNS		:	None
    void BMMarkDet_Realization(const BTMark_InputData *pInData,
                               BTMark_OutputData &rOutData);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CBallSubclassDetermination::FillHeBoundaryValues
    // DESCRIPTION	:   Fills boundary values of energetic height
    // INPUTS		:	Pointer to the array of cells with data about BM Marks
    // RETURNS		:	None
    void FillHeBoundaryValues(Cells_BMMark_Data *pArrCells);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CBallSubclassDetermination::CalculateAndCheckMark
    // DESCRIPTION	:   2nd part of BT mark determination algorithm
    // INPUTS		:	Sign of mark calculation
    // RETURNS		:	None
    QString CalculateAndCheckMark(const bool p_calc_mark,
                                  const BTMark_InputData *pInData,
                                  BTMark_OutputData &rOutData);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CBallSubclassDetermination::Drop_BallTrack
    // DESCRIPTION	:   Drop ballistic track
    // INPUTS		:	Index of ballistic track
    // RETURNS		:	None
    void Drop_BallTrack(const qint32 ind_Ball);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CBallSubclassDetermination::ClearInnerData_TType_1Track
    // DESCRIPTION	:   Clear inner data containing information about given track used for trajectory type determination
    // INPUTS		:	Index of ballistic track
    // RETURNS		:	None
    void ClearInnerData_TType_1Track(const qint32 ind_Ball);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CBallSubclassDetermination::ClearInnerData_TType_1Track
    // DESCRIPTION	:   Clear inner data containing information about given track used for BO subclass determination
    // INPUTS		:	Index of ballistic track
    // RETURNS		:	None
    void ClearInnerData_Subcl_1Track(const qint32 ind_Ball);

private:

    CGLVector <TrajType_InnerData>   m_arTTypeData;  //array of structures containing inner data used for trajectory type determination
    CGLVector <BTMark_InnerData>     m_arMarkData;   //array of structures containing inner data used for BM mark determination

    CGLArrayFix <BM_Diapason, AIR_BALL::NUMB_BM_MARKS> m_ar_BM_Diapasons; //array of diapasons for marks of ballistic missiles

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CBallSubclassDetermination::OnceMarkDet
    // DESCRIPTION	:   BT mark determination for current update
    // INPUTS		:	None
    // RETURNS		:	Resulting value of BT mark and BT mark diapason
    QString OnceMarkDet(const BTMark_InputData *pInData,
                        BMMark_AndDiapason &BMMark);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CBallSubclassDetermination::MarkDet_H_He
    // DESCRIPTION	:   BT mark determination using height and energetic height
    // INPUTS		:	None
    // RETURNS		:	Resulting value of BT mark and BT mark diapason
    void MarkDet_H_He(const BTMark_InputData *pInData,
                      BMMark_AndDiapason &BMMark, QString &log);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CBallSubclassDetermination::MarkDet_He
    // DESCRIPTION	:   BT mark determination using energetic height
    // INPUTS		:	None
    // RETURNS		:	Resulting value of BT mark and BT mark diapason
    void MarkDet_He(const BTMark_InputData *pInData,
                    BMMark_AndDiapason &BMMark);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CBallSubclassDetermination::GridScaleCalculation
    // DESCRIPTION	:   Calculating scale of grid in correspondence of input value
    // INPUTS		:	Some value of H or He in H-He plane
    // RETURNS		:	Scale of grid in correspondence of input value
    qint32 GridScaleCalculation(const qint32 H_He_value);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CBallSubclassDetermination::MarkCorr_Theta_He
    // DESCRIPTION	:   BT mark correction using throw angle at the end of active path and energetic height
    // INPUTS		:	None
    // RETURNS		:	Current value of BT mark and BT mark diapason
    void MarkCorr_Theta_He(BMMark_AndDiapason &BMMark);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CBallSubclassDetermination::MarkCorr_Theta_Veap
    // DESCRIPTION	:   BT mark correction using throw angle at the end of active path
    //              :   and velocity at the end of active path
    // INPUTS		:	None
    // RETURNS		:	Current value of BT mark and BT mark diapason
    void MarkCorr_Theta_Veap(BMMark_AndDiapason &BMMark);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CBallSubclassDetermination::MarkCorr_Distance
    // DESCRIPTION	:   BT mark correction using distance from the start point to the fall point
    // INPUTS		:	None
    // RETURNS		:	Current value of BT mark and BT mark diapason
    void MarkCorr_Distance(const BTMark_InputData *pInData,
                           BMMark_AndDiapason &BMMark);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CBallSubclassDetermination::GetMarkByCode
    // DESCRIPTION	:   Output of information about mark based on the hash-code
    // INPUTS		:	Hash-code of list of BM-marks
    // RETURNS		:	Current value of BM mark and BM mark diapason
    void GetBMMarkByCode(const qint32 Code_ListBMMarks, BMMark_AndDiapason &BMMark);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CBallSubclassDetermination::GetNominalRange
    // DESCRIPTION	:   Get nominal range using value of mark (enum BM_Marks)
    // INPUTS		:	Value of mark (see enum BM_Marks)
    // RETURNS		:	Nominal range, m
    qreal GetNominalRange(const qint32 MarkValue);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CBallSubclassDetermination::CalculateCellNumber
    // DESCRIPTION	:   Determination of cell number depending on row, column and diapason
    //              :   for grid on the plane "H-He"
    // INPUTS		:	row number (numeration from 1), column number (numeration from 1), diapason (numeration from 0)
    // RETURNS		:	cell number (numeration from 1)
    qint32 CalculateCellNumber(const qint32 i, const qint32 j, const qint32 diapason);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CBallSubclassDetermination::CalculateCellNumber_H_He
    // DESCRIPTION	:   Determination of cell number depending on height and energetic height
    //              :   for grid on the plane "H-He"
    // INPUTS		:	height (m), energetic height (m)
    // RETURNS		:	cell number (numeration from 1)
    qint32 CalculateCellNumber_H_He(const qreal H, const qreal He);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CBallSubclassDetermination::CalculateLimitingHeight
    // DESCRIPTION	:   Calculation of limiting height to determine mark on descending branch
    // INPUTS		:	None
    // RETURNS		:	Value of limiting height
    qreal CalculateLimitingHeight(const BTMark_InputData *pInData);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CBallSubclassDetermination::ClearInnerData_TType
    // DESCRIPTION	:   Clear inner data used for trajectory type determination
    // INPUTS		:	None
    // RETURNS		:	None
    void ClearInnerData_TType();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CBallSubclassDetermination::ClearInnerData_Subcl
    // DESCRIPTION	:   Clear inner data used for BO subclass determination
    // INPUTS		:	None
    // RETURNS		:	None
    void ClearInnerData_Subcl();

};

#endif // CBALLSUBCLASSDETERMINATIONM_H
