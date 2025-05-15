QT *= core

DESTDIR_common = $$clean_path($$PWD/../../../lib)

INCLUDEPATH *= $$clean_path($$PWD/..)

LIBS_paths *= -L$$DESTDIR_common
LIBS_libs  *= -lcommon
