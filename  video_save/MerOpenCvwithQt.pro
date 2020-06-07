
INCLUDEPATH += ${GENICAM_ROOT_V2_3}/library/CPP/include
INCLUDEPATH += ${GENICAM_ROOT_V2_3}/../../sdk/include

LIBS += -lgxiapi \#-ldximageproc\
#-L${GENICAM_ROOT_V2_3}/bin/Linux32_i86 \
#-lGCBase_gcc40_v2_3 -lGenApi_gcc40_v2_3 -llog4cpp_gcc40_v2_3 -lLog_gcc40_v2_3 -lMathParser_gcc40_v2_3

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

TARGET = MerOpenCvwithQt
TEMPLATE = app


SOURCES += main.cpp\
        TemplateMatch.cpp \
        mainwindow.cpp

HEADERS  += mainwindow.h \
    TemplateMatch.h

FORMS    += mainwindow.ui


