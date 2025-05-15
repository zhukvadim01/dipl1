TARGET    = compareTC
TEMPLATE  = app
CONFIG   += \
    qt \
    console \
    c++17
QT       += concurrent

#include($$PWD/../shared/shared.pri)
include($$PWD/../one_thread_TC/one_thread_TC.pri)
include($$PWD/../multy_thread_TC/multy_thread_TC.pri)

LIBS  = $$LIBS_ALL
LIBS += $$reverse(LIBS_all)

message($$INCLUDEPATH)
message($$LIBS)

DESTDIR     = $$PWD/../../bin
OBJECTS_DIR = $$PWD/../../obj
MOC_DIR     = $$PWD/../../moc

SOURCES += \
        load_input_data.cpp \
        main.cpp

HEADERS += \
    load_input_data.h

