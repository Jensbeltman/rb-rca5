TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp

CONFIG += link_pkgconfig
PKGCONFIG += gazebo
PKGCONFIG += opencv

INCLUDEPATH += /opt/fuzzylite-6.0/fuzzylite
LIBS += -L /opt/fuzzylite-6.0/fuzzylite/release/bin -lfuzzylite-static

