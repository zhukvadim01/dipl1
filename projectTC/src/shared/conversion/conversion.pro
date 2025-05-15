
TARGET = conversion
TEMPLATE = lib
CONFIG += staticlib warn_on c++17

include($$PWD/conversion.pri)

DESTDIR     = $$DESTDIR_Conversion
OBJECTS_DIR = $$PWD/../../../obj/shared/Conversion
MOC_DIR     = $$PWD/../../../moc/shared/Conversion

HEADERS += \
    Altitude.h \
    ConstRecalc.h \
    Geocentric.h \
    Geocentric_Geo.h \
    Geocentric_ICS.h \
    Geocentric_Topo.h \
    Geodesic.h \
    Geoline.h \
    Spherical.h \
    Spherical_Topo.h \
    Structures.h \
    Topo1_Topo2.h \
    Topocentric.h \
    Topographic.h \
    Topo_Topographic.h \
    X_Z_H_TOPO.h \
    stdafx.h

SOURCES += \
    Altitude.cpp \
    Geocentric.cpp \
    Geocentric_Geo.cpp \
    Geocentric_Topo.cpp \
    Geodesic.cpp \
    Geoline.cpp \
    Spherical.cpp \
    Spherical_Topo.cpp \
    Topo1_Topo2.cpp \
    Topocentric.cpp \
    Topographic.cpp \
    Topo_Topographic.cpp \
    X_Z_H_TOPO.cpp \
    stdafx.cpp

DISTFILES += \
    Makefile
