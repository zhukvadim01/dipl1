
TARGET   = one_thread_TC
TEMPLATE = lib
CONFIG  += staticlib warn_on c++17

include($$PWD/one_thread_TC.pri)

DESTDIR      = $$DESTDIR_OTC
OBJECTS_DIR  = $$clean_path($$DESTDIR_OTC/../obj/oTC)
MOC_DIR      = $$clean_path($$DESTDIR_OTC/../moc/oTC)

DEFINES *= _USE_MATH_DEFINES
DEFINES *= QT_DEPRECATED_WARNINGS
DEFINES *= KEEP_LOG

SOURCES += \
    CAirBallist_Processing.cpp \
    CAirBallist_Processing_structs.cpp \
    CBallSubclassDet_structs.cpp \
    CBallSubclassDetermination.cpp \
    CBallist_ActiveProgn.cpp \
    CBallist_ActiveProgn_structs.cpp \
    CBallisticProlong_structs.cpp \
    CBallisticProlongation.cpp \
    CPathBranchDet_structs.cpp \
    CPathBranchDetermination.cpp \
    CtrlAir_Ballist.cpp \
    CtrlAir_Ballist_structs.cpp \
    ball_one_prognoz.cpp \
    ball_prognoz_structs.cpp \
    cellipsepoint.cpp

HEADERS += \
    CAirBallist_Processing.h \
    CAirBallist_Processing_structs.h \
    CBallSubclassDet_structs.h \
    CBallSubclassDetermination.h \
    CBallist_ActiveProgn.h \
    CBallist_ActiveProgn_structs.h \
    CBallisticProlong_structs.h \
    CBallisticProlongation.h \
    CPathBranchDet_structs.h \
    CPathBranchDetermination.h \
    CtrlAir_Ballist.h \
    CtrlAir_Ballist_structs.h \
    air_ballist_constants.h \
    air_ballist_literal.h \
    ball_amount_const.h \
    ball_one_prognoz.h \
    ball_prognoz_structs.h \
    cellipsepoint.h
