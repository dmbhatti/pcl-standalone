#-------------------------------------------------
#
# Project created by QtCreator 2015-12-14T15:39:30
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = pclApp
TEMPLATE = app


SOURCES += main.cpp\
        pclviewer.cpp \
    capture.cpp \
    instanceid.cpp \
    algos/planeextractor.cpp \
    cloudemitterreceiver.cpp

HEADERS  += pclviewer.h \
    globals.h \
    capture.h \
    instanceid.h \
    algos/planeextractor.h \
    cloudemitterreceiver.h

FORMS    += pclviewer.ui

INCLUDEPATH += /usr/include/pcl-1.7
