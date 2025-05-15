//FILE : GDEnum.h
//AUTHOR : Damankova Vera
//DESCRIPTION : ENUMs for global data (GD).

#ifndef _GDEnum_h_
#define _GDEnum_h_

enum EClass
{
    // CLASSES
    CLASS_UNKNOWN		= 0,
    CLASS_AERODINAMIC	= 1,		// Aerodynamic
    CLASS_BALLISTIC     = 2,		// Ballistic
    CLASS_SATELITE      = 3,        // Satellite
    CLASS_SHELL         = 4,        // Interceptor
    // AMOUNT
    CLASSES_AMOUNT
};

enum ESubClass
{
    // CLASS_BALLISTIC
    SUBCLASS_UNKNOWN    = 0,
    SUBCLASS_WARHEAD  	= 1,
    SUBCLASS_BOOSTER	= 2,
    SUBCLASS_FRAGMENT   = 3,
    SUBCLASS_DECOY      = 4,
    // CLASS_SHELL
    SUBCLASS_HYPERSONIC = 5,
    // AMOUNT
    SUBCLASSES_AMOUNT
};

enum EType	//type
{
    // SUBCLASS_WARHEAD
    TYPE_UNDEFINED	= 0,
    TYPE_BM300      = 1,
    TYPE_BM600      = 2,
    TYPE_BM750      = 3,
    TYPE_BM1000     = 4,
    TYPE_BM1500     = 5,
    TYPE_BM1800     = 6,
    TYPE_BM2000     = 7,
    TYPE_BM2500     = 8,
    TYPE_BM2800     = 9,
    TYPE_BM3000     = 10,
    TYPE_BM3500     = 11,
    // CLASS_SHELL
    TYPE_IC         = 12,
    // AMOUNT
    TYPES_AMOUNT
};

enum EManeuver	//maneuver sign
{
    NO_MANEUVER = 0,
    MNVR_NOT_SPECIFIED = 1,
    MNVR_AEROBALLISTIC = 1 << 1,
    MNVR_LAUNCH_PLANE  = 1 << 2
};

enum EPartBall
{
    PAYLOAD = 0,
    FIRST_BOOSTER = 1,
    SECOND_BOOSTER = 2,
    THIRD_BOOSTER = 3,
    PBV_MIRV = 4,
    LIGHT_DECOY = 5,
    HEAVY_DECOY = 6
};

enum EImitAO
{
    REAL	= 0,
    IMIT	= 1
};

// nationality of AO
enum ENationAO
{
    NATION_UNKNOWN      = 0,
    NATION_FOE			= 1,
    NATION_FRIEND		= 2
};

enum ETrackingSign
{
    RENEW_COORD			= 0,
    NEW_COORD			= 1,
    EXTRAPOLATION_COORD	= 2,
    HOLD_COORD			= 3,
    DROP_COORD			= 4
};

enum ETrackingQuality
{
    LOW_Rate = 1,
    HIGH_Rate = 2
};

enum EBearingStates
{
    SINGLE_BEARING		= 0,
    B2T_IDENT			= 1,
    TRIANG_GENTRACK		= 2,
    DETECT_AO           = 3,
    BEARING_EXCLUDED	= 4
};

const int BRNG_STATE_AMOUNT = 7;		//flying object's subclasses quantity

enum EActionPhaseAO
{
    NO_ACTION = 0,           // no action
    ASSIGNED,                // target is assigned
    CANCEL_ASSIGN,           // assignment is cancelled
    PERMIT_LAUNCH,           // launch command
    LAUNCH_DELAY,            // launch delay
    LAUNCHED,                // IC is launched
    MEET_POINT,              // meeting IC and target
    IC_EXPLOSION,            // IC explosion
    KILL_ASSESSMENT,         // kill assessment start
    ASSESSMENT_RESULT,       // kill assessment results
    DESTROYED,               // target is destroyed
    MISSED,                  // target is missed
    REASSIGNED,              // reassignment
    FIRE_PROHIBITED          // fire prohibited
};

//Threat estimation
enum EThreat
{
    THREAT_NO		=	0,	// No threat.
    THREAT_RESPZONE	=	1,
    THREAT_COVOBJ	=	2
};

enum EJammer
{
    NOT_JAMMER = 0,
    JAMMER = 1,
    TRIANG_POINT = 2,
    RELIABLE_TRIANG_POINT = 3
};

enum EHoming
{
    NO_HOMING = 0,
    ACTIVE_HOMING = 1,
    PASSIVE_HOMING = 2
};

enum EGroupSign
{
    GR_SINGLE = 0,
    GR_GROUP = 1
};

enum EGroup
{
    SINGLE_TARGET = 0,
    GROUP_UNDEF_AMOUNT = 1
};

enum EActionType
{
    TYPE_NO_ACTION = 0,
    ACTION_FU = 1,
    ACTION_FCP = 2,
    ACTION_FU_FCP = 3
};

enum EVelAccel
{
    VEL_ACCEL_ABSENCE			= 0,
    VEL_PRESENT_ACCEL_ABSENCE	= 1,
    VEL_ABSENCE_ACCEL_PRESENT	= 2,
    VEL_ACCEL_PRESENT			= 3
};

enum ERegionType
{
    START_AREA	= 0,
    FALL_AREA	= 1
};

enum EBallisticPathSign
{
    NONE	= 0,
    ACTIVE = 1,
    PASSIVE = 2,
    END_ACTIVE_PATH = 3
};

enum EBallisticBranchSign
{
    BRANCH_UNKNOWN = 0,
    ASCENDING = 1,
    DESCENDING = 2
};

enum EBallisticTrajType //type of ballistic trajectory
{
    UNDEFINED_TRAJ = 0,
    OPTIMAL_TRAJ = 1,   //optimal trajectory
    LOFTED_TRAJ = 2,    //lofted (high) trajectory
    FLAT_TRAJ = 3       //flat (low) trajectory
};

enum class EDataInOut: unsigned char
{
    in = 1,
    out
};

enum EAdditionalType	//additional type of BM
{
    A_TYPE_BM300_S        = 12,
    A_TYPE_BM600_QBM      = 13,
    A_TYPE_BM600_MaRV     = 14,
    A_TYPE_BM750_QBM      = 15,
    A_TYPE_BM750_MaRV     = 16,
    A_TYPE_BM1000_QBM     = 17,
    A_TYPE_BM1000_MaRV    = 18,
    A_TYPE_BM1500_QBM     = 19,
    A_TYPE_BM1500_MaRV    = 20,
    A_TYPE_BM3500_W3      = 21
};

enum EWavelength //radar wavelength
{
    WAVELENGTH_UNDEFINED    = 0,
    WAVELENGTH_CENT         = 1,    //centimeter
    WAVELENGTH_DECIM        = 2,    //decimeter
    WAVELENGTH_METER        = 3     //meter
};

#include <QtGlobal>
#include <map>

struct SAttributes;

namespace GD2msg
{
    // attribute conversion
    void classGDtoMsg11(const SAttributes&, quint8&, quint8&, quint8&);
    void classMsg11toGD(quint8, quint8, quint8, SAttributes&);
    void classGDtoMsg100(const SAttributes&, quint8&, quint8&, quint8&);
    void classGDtoMsg101(const SAttributes&, quint8&, quint8&, quint8&, quint8&, quint8&);

    // action convertion
    extern const std::map<quint8, QString>  actionNameMap;
    extern const std::map<quint8, quint8>   actionMsg513map;
    extern const std::map<quint8, quint8>   subclassMsg100map;
    extern const std::map<quint8, quint8>   typeMsg101map;

    quint8 subclassGDtoMsg100(quint8);
    quint8 typeGDtoMsg101(quint8);
    quint8 actionGDtoMsg513(quint8);
    quint8 actionMsg513toGD(quint8);
    QString actionName(quint8);
}

namespace GD2DB {
    qint32  BMtypeGD2DB(qint32);
    qint32  BMtypeDB2GD(qint32);
    qint32  BMtypeDB2GDExtended(qint32);

    qint32  BMpartGD2DB(qint32);
    qint32  BMpartDB2GD(qint32);

    extern const std::map<qint32, qint32> typeBMmap;    //first - BM type in GD enum, second - BM type in DB classification
    extern const std::map<qint32, qint32> typeBMmapExt; //first - BM type or additional type in GD enum, second - BM type in DB classification
    extern const std::map<qint32, qint32> partBMmap;    //first - BM part in GD enum, second - BM type in DB classification
}

#endif // _GDEnum_h_
