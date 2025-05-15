TARGET   = multy_thread_TC
TEMPLATE = lib
CONFIG  += staticlib warn_on c++17

include($$PWD/multy_thread_TC.pri)

DESTDIR      = $$DESTDIR_MTC
OBJECTS_DIR  = $$clean_path($$DESTDIR_MTC/../obj/mTC)
MOC_DIR      = $$clean_path($$DESTDIR_MTC/../moc/mTC)

DEFINES *= _USE_MATH_DEFINES
DEFINES *= QT_DEPRECATED_WARNINGS
DEFINES *= KEEP_LOG

SOURCES += \
    CAirBallist_Processing_M.cpp \
    CBallSubclassDetermination_M.cpp \
    CBallist_ActiveProgn_M.cpp \
    CBallisticProlongation_M.cpp \
    CPathBranchDetermination_M.cpp \
    CtrlAir_Ballist_M.cpp \
    ball_one_prognoz_M.cpp

HEADERS += \
    CAirBallist_Processing_M.h \
    CBallSubclassDetermination_M.h \
    CBallist_ActiveProgn_M.h \
    CBallisticProlongation_M.h \
    CPathBranchDetermination_M.h \
    CtrlAir_Ballist_M.h \
    ball_one_prognoz_M.h
