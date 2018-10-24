TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    fuzzy1.cpp \
    buffer.cpp \
    circledetect.cpp \
    fuzzy2.cpp \
    fuzzy_control.cpp

CONFIG += link_pkgconfig
PKGCONFIG += gazebo
PKGCONFIG += opencv

HEADERS += \
    fuzzy1.h \
    buffer.h \
    circledetect.h \
    fuzzy2.h \
    fuzzy_control.h

INCLUDEPATH += /home/bjarke/Documents/.fuzzylite/fuzzylite-6.0/fuzzylite/

LIBS += -L /home/bjarke/Documents/.fuzzylite/fuzzylite-6.0/fuzzylite/release/bin -l fuzzylite-static
