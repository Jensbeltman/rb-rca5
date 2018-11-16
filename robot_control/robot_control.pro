TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    localizor.cpp \
    laserscanner.cpp \
    mp.cpp \
    montecarlo.cpp \
    line.cpp



CONFIG += link_pkgconfig
PKGCONFIG += gazebo
PKGCONFIG += opencv

HEADERS += \
    localizor.h \
    laserscanner.h \
    mp.h \
    montecarlo.h \
    line.h \
    Visibility/visibility.hpp \
    Visibility/floats.hpp \
    Visibility/vector2.hpp \
    Visibility/primitives.hpp
