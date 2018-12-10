TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    fuzzy_control.cpp \
    laserscanner.cpp \
    movetopoint.cpp

CONFIG += link_pkgconfig
PKGCONFIG += gazebo
PKGCONFIG += opencv

HEADERS += \
    fuzzy_control.h \
    laserscanner.h \
    movetopoint.h

INCLUDEPATH += $$PWD/fuzzylite/

#LIBS += -L /home/bl/git/fuzzy/robot_control/fuzzylite/release/bin -l fuzzylite-static
LIBS += -L $$PWD/fuzzylite/release/bin -l fuzzylite-static
