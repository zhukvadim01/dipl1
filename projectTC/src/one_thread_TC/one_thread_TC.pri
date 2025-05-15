include($$PWD/../shared/shared.pri)

DESTDIR_OTC = $$clean_path($$PWD/../../lib)

INCLUDEPATH *= $$clean_path($$PWD)

LIBS_ALL *= -L$$clean_path($$DESTDIR_OTC)
LIBS_all *= -lone_thread_TC
