#-------------------------------------------------
#
# Project created by QtCreator 2016-03-30T15:40:52
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = qMAVlink
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp

HEADERS  += mainwindow.h

FORMS    += mainwindow.ui

include(qextserialport/src/qextserialport.pri)

INCLUDEPATH += mavlink/include/mavlink/v1.0
