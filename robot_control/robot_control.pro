TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    fuzzy1.cpp

CONFIG += link_pkgconfig
PKGCONFIG += gazebo
PKGCONFIG += opencv

HEADERS += \
    fuzzy1.h

INCLUDEPATH += /home/bjarke/Documents/.fuzzylite/fuzzylite-6.0/fuzzylite/

LIBS += -L /home/bjarke/Documents/.fuzzylite/fuzzylite-6.0/fuzzylite/release/bin -l fuzzylite-static
