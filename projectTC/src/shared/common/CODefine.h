#ifndef _INC_CODEFINES_H
#define _INC_CODEFINES_H

#include <type_traits>

#include <QtGlobal>

#ifdef WIN32
    #define FILENAME_LEN _MAX_FNAME
#else // assuming UNIX
    #define FILENAME_LEN 256
#endif // WIN32

#define	ASSERT_ON
#define	NDEBUG

#include <cassert>
#ifdef	ASSERT_ON
    #define	G_ASSERT(sign)	assert(sign)
    #define DPS_ASSERT(sign) assert(sign)
#else
    #define	G_ASSERT(sign)	if(sign == 0)	throw -1
    #define	DPS_ASSERT(sign)	if(sign == 0)	throw -1

#endif

#ifdef	ASSERT_ON
    #define	LOCAL_ASSERT(sign)	assert(sign)
#else
    #define	LOCAL_ASSERT(sign)	assert(true)
#endif

#include <cstring>
#include <QTimeZone>
#include <QDateTime>

#define HI_BYTE( x ) ( ( (x) & 0xF0 ) >> 4 )
#define LO_BYTE( x ) (x) & 0xF

#define DEF_TIME_ZONE                    QTimeZone(0)
#define SECOND_IN_DAY                    86400
#define MSECOND_IN_DAY                   86400000

#define CURR_TIME_START_OF_DAY_MSEC      QDateTime::currentMSecsSinceEpoch()%MSECOND_IN_DAY
#define CURR_TIME_START_OF_DAY_SEC       qreal(QDateTime::currentMSecsSinceEpoch()%MSECOND_IN_DAY)/1000.

#define CURR_TIME_OF_70_SEC              qreal(QDateTime::currentMSecsSinceEpoch())/1000.
#define CURR_TIME_OF_70_MSEC             QDateTime::currentMSecsSinceEpoch()
#define CURR_TIME_QSTRING                QDateTime::currentDateTimeUtc().toString("hh:mm:ss.zzz")
#define CURR_TIME_PRINTABLE              QDateTime::currentDateTimeUtc().toString("hh:mm:ss.zzz").toLocal8Bit().data()
#define CURR_DATETIME_QSTRING            QDateTime::currentDateTimeUtc().toString("hh:mm:ss.zzz dd.MM.yyyy")
#define CURR_DATETIME_PRINTABLE          QDateTime::currentDateTimeUtc().toString("hh:mm:ss.zzz dd.MM.yyyy").toLocal8Bit().data()

#define TIME_MSEC_PRINTABLE(_varName)       QDateTime::fromMSecsSinceEpoch(_varName, DEF_TIME_ZONE).toString("hh:mm:ss.zzz").toLocal8Bit().data()
#define TIME_SEC_PRINTABLE(_varName)        QDateTime::fromMSecsSinceEpoch(qreal(_varName)*1000 , DEF_TIME_ZONE).toString("hh:mm:ss.zzz").toLocal8Bit().data()
#define TIME_MSEC_QSTRING(_varName)         QDateTime::fromMSecsSinceEpoch(_varName, DEF_TIME_ZONE).toString("hh:mm:ss.zzz")
#define TIME_SEC_QSTRING(_varName)          QDateTime::fromMSecsSinceEpoch(qreal(_varName)*1000, DEF_TIME_ZONE).toString("hh:mm:ss.zzz")

#define DATETIME_FROM70_MSEC_PRINTABLE(_varName)   QDateTime::fromMSecsSinceEpoch(_varName, DEF_TIME_ZONE).toString("hh:mm:ss.zzz dd.MM.yyyy").toLocal8Bit().data()
#define DATETIME_FROM70_SEC_PRINTABLE(_varName)    QDateTime::fromMSecsSinceEpoch(qreal(_varName)*1000., DEF_TIME_ZONE).toString("hh:mm:ss.zzz dd.MM.yyyy").toLocal8Bit().data()
#define DATETIME_FROM70_MSEC_QSTRING(_varName)     QDateTime::fromMSecsSinceEpoch(_varName, DEF_TIME_ZONE).toString("hh:mm:ss.zzz dd.MM.yyyy")
#define DATETIME_FROM70_SEC_QSTRING(_varName)      QDateTime::fromMSecsSinceEpoch(qreal(_varName)*1000., DEF_TIME_ZONE).toString("hh:mm:ss.zzz dd.MM.yyyy")

#define DeltaTIME(_t1,_t2)     _t1 - _t2
#define DeltaTIME_SEC_24H(_t1,_t2)   ((_t1 - _t2) <= 0 && (_t1 - _t2) < -SECOND_IN_DAY/2.)  ? ((_t1 - _t2)   + SECOND_IN_DAY)  : ((_t1-_t2))
#define DeltaTIME_MSEC_24H(_t1,_t2)  ((_t1 - _t2) <= 0 && (_t1 - _t2) < -MSECOND_IN_DAY/2.) ? ((_t1 - _t2)   + MSECOND_IN_DAY) : ((_t1-_t2))

#define SEC70_to_SEC_START_OF_DAY(_varName)         qreal(qint64((_varName) * 1000) % MSECOND_IN_DAY) / 1000.
#define SEC70_to_MSEC_START_OF_DAY(_varName)        qint64((_varName) * 1000) % MSECOND_IN_DAY
#define MSEC70_to_SEC_START_OF_DAY(_varName)        qreal(((qint64)(_varName)) % MSECOND_IN_DAY)/1000.
#define MSEC70_to_MSEC_START_OF_DAY(_varName)       (((qint64)(_varName)) % MSECOND_IN_DAY)
#define MSEC70_to_25MICRO_START_OF_DAY(_varName)    (((qint64)(_varName)) % MSECOND_IN_DAY) * (1000 / 25)
#define MSEC70_to_SEC70(_varName)                   qreal(_varName)/1000.
#define SEC70_to_MSEC70(_varName)                   qreal(_varName)*1000.
#define MSEC_START_OF_DAY_to_MSEC70(_varName)       quint64(CURR_TIME_OF_70_MSEC - CURR_TIME_START_OF_DAY_MSEC + _varName)
#define SEC_START_OF_DAY_to_SEC70(_varName)         CURR_TIME_OF_70_SEC - CURR_TIME_START_OF_DAY_SEC + _varName

const unsigned char g_cuchMAX_IC = 6;

#ifndef under_cast
#define under_cast(EnumValue) static_cast<std::underlying_type_t<decltype(EnumValue)>>(EnumValue)
#endif

#ifndef c_cast
#define c_cast(type, var) (type)(var)
#endif

#ifndef UNIQUE_VARNAME
#define UNIQUE_VARNAME_IMPL(lineno) _a_local_var##lineno
#define UNIQUE_VARNAME_IMPL_PREP(lineno) UNIQUE_VARNAME_IMPL(lineno)
#define UNIQUE_VARNAME UNIQUE_VARNAME_IMPL_PREP(__LINE__)
#endif

#define ENUM_CLASS_QDATASTREAM_FUNCTIONS(enumClassName) \
    inline QDataStream& operator<<(QDataStream& stream, const enumClassName& data) { stream << reinterpret_cast<const std::underlying_type<enumClassName>::type&>(data); return stream; } \
    inline QDataStream& operator>>(QDataStream& stream, enumClassName& data) { stream >> reinterpret_cast<std::underlying_type<enumClassName>::type &>(data); return stream; }

//===================================================================

#define CREATE_SINGLETONE(classname)                                            \
    \
    static std::unique_ptr<classname> sInstance;                                \
    classname *classname::instance() {                                          \
        if ( !sInstance ) {                                                     \
            static QMutex sMutex;                                               \
            QMutexLocker locker( &sMutex );                                     \
            if ( !sInstance ) {                                                 \
                sInstance = std::unique_ptr<classname>(new classname()); } }    \
        return sInstance.get(); }                                               \
    \
    void classname::reset() {                                                   \
        sInstance.reset(nullptr); }

#define DECLARE_SINGLETONE(classname) \
    static classname* instance();    \
    static void reset();

#endif //_INC_CODEFINES_H
