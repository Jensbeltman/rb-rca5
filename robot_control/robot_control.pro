TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    localizor.cpp \
    laserscanner.cpp \
    mp.cpp \
    montecarlo.cpp

CONFIG += link_pkgconfig
PKGCONFIG += gazebo
PKGCONFIG += opencv



HEADERS += \
    localizor.h \
    laserscanner.h \
    mp.h \
    montecarlo.h
