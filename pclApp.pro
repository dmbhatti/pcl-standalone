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
    cloudemitterreceiver.cpp \
    algos/downsampler.cpp \
    algos/ADEVision/ExtractedPlane.cpp

HEADERS  += pclviewer.h \
    globals.h \
    capture.h \
    instanceid.h \
    algos/planeextractor.h \
    cloudemitterreceiver.h \
    algos/downsampler.h \
    algos/ADEVision/ExtractedPlane.hpp \
    algos/ADEVision/PCLFunctions.hpp

FORMS    += pclviewer.ui

INCLUDEPATH += /usr/include/pcl-1.7
INCLUDEPATH += /usr/include/opencv
INCLUDEPATH += /usr/include/opencv2
INCLUDEPATH += /usr/include/eigen3
