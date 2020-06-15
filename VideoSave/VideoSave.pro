
INCLUDEPATH += ${GENICAM_ROOT_V2_3}/library/CPP/include
INCLUDEPATH += ${GENICAM_ROOT_V2_3}/../../sdk/include

LIBS += -lgxiapi \
#opencv header

INCLUDEPATH +=usr/local/include \
              usr/local/include/opencv \
              usr/local/include/opencv2 \
              usr/include/eigen3

#opencv lib

LIBS+=/usr/lib/libopencv_highgui.so\
      /usr/lib/libopencv_core.so\
      /usr/lib/libopencv_videoio.so\
      /usr/lib/libopencv_imgcodecs.so\
      /usr/lib/libopencv_imgproc.so\
      /usr/lib/libopencv_calib3d.so



QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = VideoSave
TEMPLATE = app


SOURCES += \
           src/main.cpp \
           src/TemplateMatch.cpp \
           src/mainwindow.cpp \
           src/ModulesDetect.cpp

HEADERS += inc/mainwindow.h\
           inc/TemplateMatch.h\
           inc/GxIAPI.h\
           inc/DxImageProc.h\
           inc/ModulesDetect.h


FORMS    += mainwindow.ui

