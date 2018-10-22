TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    mp.cpp \
    AvlTree.cpp \
    graph.cpp \
    Bezier.cpp \
    indxavltree.cpp \
    cv_operators.cpp

CONFIG += link_pkgconfig
PKGCONFIG += gazebo
PKGCONFIG += opencv

HEADERS += \
    mp.h \
    AvlTree.h \
    graph.h \
    Bezier.h \
    indxavltree.h
