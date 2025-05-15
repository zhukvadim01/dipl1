include($$clean_path($$PWD/../common/common.pri))
include($$clean_path($$PWD/../conversion/conversion.pri))


QT *= \
    core    \
    network \
    testlib \
    sql     \
    widgets \
    xml

DESTDIR_gl = $$clean_path($$PWD/../../../lib)

INCLUDEPATH *= $$clean_path($$PWD/..)

LIBS_paths *= -L$$DESTDIR_gl
LIBS_libs  *=  -lgl
