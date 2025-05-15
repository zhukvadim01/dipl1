
TARGET = common
TEMPLATE = lib
CONFIG += staticlib warn_on c++17
unix: QMAKE_CXXFLAGS_WARN_ON += -Wno-unused-parameter -Wno-unused-but-set-parameter \
                                -Wno-overloaded-virtual -Wno-deprecated-copy -Wno-class-memaccess #let's hope somebody will fix those properly someday

include($$PWD/common.pri)

DESTDIR     = $$DESTDIR_common
OBJECTS_DIR = $$PWD/../../../obj/shared/common
MOC_DIR     = $$PWD/../../../moc/shared/common

HEADERS += \
    COEnum.h \
    GDEnum.h \
#    CheckMemLeak.h \
#    COCrcTab.h \
    CODefine.h \
#    LogicalAddress.h \
#    DataStreams.h \
#    GlobalConstants.h \
#    GlobalFunctions.h \
#    GlobalStructs.h \
    DefaultsConsts.h
#    CheckSumCalc.h \
#    Settings.h \
 #    ThreadPoolExtra.h


SOURCES +=
#    DataStreams.cpp \
#    CheckSumCalc.cpp \
#    GlobalFunctions.cpp \
#    GlobalStructs.cpp \
#    LogicalAddress.cpp \
 #    Settings.cpp
