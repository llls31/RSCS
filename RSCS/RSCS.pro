QT      += core gui
QT      += multimedia
QT      += multimediawidgets
QT      += serialport
QT += webenginewidgets

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17
CONFIG+=sdk_no_version_check
# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    GPS/nmeaparser.cpp \
    GPS/serialport.cpp \
    main.cpp \
    mainwindow.cpp \
    objectdetector.cpp \
    permission.m

HEADERS += \
    GPS/nmeaparser.h \
    GPS/serialport.h \
    Types.h \
    mainwindow.h \
    objectdetector.h

FORMS += \
    mainwindow.ui

QMAKE_POST_LINK = cp $$PWD/Info.plist $$OUT_PWD/$${TARGET}.app/Contents/

QMAKE_INFO_PLIST = Info.plist

INCLUDEPATH += /opt/homebrew/Cellar/opencv/4.7.0_4/include/opencv4
LIBS += -L/opt/homebrew/Cellar/opencv/4.7.0_2/lib \
        -lopencv_core \
        -lopencv_imgproc \
        -lopencv_imgcodecs \
        -lopencv_videoio \
        -lopencv_highgui \
        -lopencv_dnn \
        -lopencv_objdetect


LIBS += -framework Foundation
LIBS += -framework AVFoundation

QMAKE_CXXFLAGS += -x objective-c++ -fobjc-arc

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

RESOURCES += \
    Resource.qrc
