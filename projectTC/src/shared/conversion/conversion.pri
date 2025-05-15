QT *= core

DESTDIR_Conversion = $$clean_path($$PWD/../../../lib)

DEFINES *=  _USE_MATH_DEFINES
INCLUDEPATH *= $$clean_path($$PWD/..)

LIBS_paths *= -L$$DESTDIR_Conversion
LIBS_libs  *= -lconversion
