include($$PWD/../shared/shared.pri)
include($$PWD/../one_thread_TC/one_thread_TC.pri)

DESTDIR_MTC = $$clean_path($$PWD/../../lib)

INCLUDEPATH *= $$clean_path($$PWD)

LIBS_ALL *= -L$$clean_path($$DESTDIR_MTC)
LIBS_all *= -lmulty_thread_TC
