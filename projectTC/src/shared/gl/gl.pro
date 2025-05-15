
TARGET = gl
TEMPLATE = lib
CONFIG += staticlib warn_on c++17
unix: QMAKE_CXXFLAGS_WARN_ON += -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual -Wno-deprecated-copy -Wno-class-memaccess #let's hope somebody will fix those properly someday

include($$PWD/gl.pri)

DESTDIR     = $$DESTDIR_gl
OBJECTS_DIR = $$PWD/../../../obj/shared/gl
MOC_DIR     = $$PWD/../../../moc/shared/gl

HEADERS += \
    Constants.h \
    GLArray.h \
    GLCalcTrajParam.h \
    GLConvertMatrices.h \
    GLEllipse.h \
    GLEllipse2D.h \
    GLFileLog.h \
    GLGeometry.h \
    GLLogSpecForm.h \
    GLMath.h \
    GLMatrix.h \
    GLMatrix_tmpl.h \
    GLPolygon.h \
    GLRandlib.h \
    GLSector.h \
    GLSolve_eq_2_3_4_deg.h \
    GLSrcEnumeration.h \
    GLTrackPoint.h \
    GLTrapeze.h

SOURCES += \
    GLCalcTrajParam.cpp \
    GLConvertMatrices.cpp \
    GLEllipse.cpp \
    GLEllipse2D.cpp \
    GLFileLog.cpp \
    GLGeometry.cpp \
    GLLogSpecForm.cpp \
    GLMatrix.cpp \
    GLPolygon.cpp \
    GLRandlib.cpp \
    GLSector.cpp \
    GLSolve_eq_2_3_4_deg.cpp \
    GLSrcEnumeration.cpp \
    GLTrackPoint.cpp \
    GLTrapeze.cpp



