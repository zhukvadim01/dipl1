#ifndef AIR_BALLIST_LITERAL_H
#define AIR_BALLIST_LITERAL_H

namespace AIR_BALL
{
    enum InDataModes //enum containing information about used structure of input data
    {
        TEST_DATA = 0, //data filled in unit tests
        GT_DATA = 1, //input data for general track
        ST_DATA = 2, //input data for single track
    };

    enum Prognosis_Signes	//signes of prognosis
    {
        OLD_PROGNOSIS = 0,
        NEW_PROGNOSIS = 1,
        INCORRECT_PROGNOSIS = 2,
        PROGNOSIS_WITHOUT_FALL_POINT = 3,
        PROGNOSIS_WITH_LARGE_ELLIPSE = 4
    };

    enum Act_Progn_Signes //signes of prediction on active path
    {
        PREVIOUS_ACT_PROGN = 0,
        NEW_ACT_PROGN = 1,
        UNRELIABLE_ACT_PROGN = 2
    };

    enum BTPaths //paths of ballistic trajectory
    {
        UNDEFINED_PATH = 0,
        ACTIVE_PATH = 1,
        PASSIVE_PATH = 2,
        END_OF_ACTIVE_PATH = 3
    };

    enum BTBranches //branches of ballistic trajectory
    {
        UNDEFINED_BRANCH = 0,
        ASCENDING_BRANCH = 1,
        DESCENDING_BRANCH = 2,
        TRANSITION_BRANCHES = 3 //transition between branches
    };

    enum BallTrajTypes //types of ballistic trajectory
    {
        UNDEFINED_TRAJ_TYPE = 0,
        OPTIMAL = 1,
        LOFTED = 2, //high trajectory
        FLAT = 3 //low trajectory
    };

    enum StartPointFinding //methods of start point finding
    {
        UNDEFINED_START_POINT = 0,
        START_POINT_BY_DOWN_PROGNOSIS = 1, //start point is determined using down prognosis
        START_POINT_BY_EAP = 2, //start point is determined using table of the ends of active path
        START_POINT_BY_EAP_MULTIVAL = 3, //start point is determined using several tables of the ends of active path, in the case of multivalued mark of BM
        START_POINT_BY_SOURCE = 4, //start point is obtained from source
        START_POINT_BY_1ST_POINT = 5, //start point is corrected using information about 1st point of the track
        START_POINT_ABSENCE = 6 //it is impossible to estimate start point
    };

    enum TimeEAPFinding //method of time EAP finding
    {
        UNDEFINED_TEAP_METHOD = 0,
        TEAP_BY_TRANSITION_ACT_PASS = 1, //if transition "Active path - passive path" was observed
        TEAP_BY_TABLE_EAP = 2, //EAP time determined by table of EAP
        TEAP_BY_START_TIME = 3 //EAP time determined using start time
    };

    enum BM_Marks //marks of BM
    {
        UNDEFINED_BM_MARK = 0,
        UNKNOWN_OF_SHORT_RANGE = 1,
        BM60	= 2,
        BM100	= 3,
        BM200	= 4,
        BM300	= 5,
        BM600	= 6,
        BM750	= 7,
        BM1000	= 8,
        BM1500	= 9,
        BM1800	= 10,
        BM2000	= 11,
        BM2500	= 12,
        BM2800	= 13,
        BM3000	= 14,
        BM3500	= 15,
        BM5000	= 16,
        BM6000	= 17,
        BM7000	= 18,
        BM8000  = 19,
        BM9000  = 20,
        BM10000 = 21,
        BM11000 = 22,
        BM12000 = 23,
        UNKNOWN_OF_LONG_RANGE = 24
    };

    extern const char* log_path;
    extern const char* hhe_ini_file;
}

#endif // AIR_BALLIST_LITERAL_H
