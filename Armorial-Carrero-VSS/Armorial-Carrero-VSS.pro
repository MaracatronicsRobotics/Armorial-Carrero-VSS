QT += network
QT += widgets

CONFIG += c++14 console
CONFIG -= app_bundle

VERSION  = 1.0.0
DESTDIR  = ../bin
TARGET   = Armorial-Carrero-VSS

# Temporary dirs
OBJECTS_DIR = tmp/obj
MOC_DIR = tmp/moc
UI_DIR = tmp/moc
RCC_DIR = tmp/rc

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS
LIBS *= -lprotobuf -lGLU -pthread -lGEARSystem -lomniORB4 -lomnithread -lQt5Core -lpthread

system(echo "compiling protobuf" && cd include/3rd_party/protobuf && protoc --cpp_out=../ *.proto && cd ../../..)

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
        entity/armorialvisionclient/armorialvisionclient.cpp \
        entity/armorialvisionupdater/armorialvisionupdater.cpp \
        entity/armorialvisionupdater/robottracker.cpp \
        entity/entity.cpp \
        exithandler.cpp \
        include/3rd_party/command.pb.cc \
        include/3rd_party/common.pb.cc \
        include/3rd_party/netraw.cpp \
        include/3rd_party/packet.pb.cc \
        include/3rd_party/replacement.pb.cc \
        include/3rd_party/robocup_ssl_client.cpp \
        main.cpp \
        utils/fieldareas/fieldareas.cpp \
        utils/fieldtypes/fieldtypes.cpp \
        utils/filters/kalman/kalmanfilter.cpp \
        utils/filters/kalman/kalmanstate.cpp \
        utils/filters/kalman/matrix.cpp \
        utils/filters/loss/lossfilter.cpp \
        utils/filters/multiobject/multiobject.cpp \
        utils/filters/multiobject/multiobjectfilter.cpp \
        utils/filters/noise/noisefilter.cpp \
        utils/mrctimer/mrctimer.cpp \
        utils/types/ball/ball.cpp \
        utils/types/object.cpp \
        utils/types/robot/robot.cpp

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

HEADERS += \
    entity/armorialvisionclient/armorialvisionclient.h \
    entity/armorialvisionupdater/armorialvisionupdater.h \
    entity/armorialvisionupdater/robottracker.h \
    entity/entity.h \
    exithandler.h \
    include/3rd_party/command.pb.h \
    include/3rd_party/common.pb.h \
    include/3rd_party/netraw.h \
    include/3rd_party/packet.pb.h \
    include/3rd_party/replacement.pb.h \
    include/3rd_party/robocup_ssl_client.h \
    include/3rd_party/util.h \
    include/filters.h \
    include/timer.h \
    utils/color/color.h \
    utils/fieldareas/fieldareas.h \
    utils/fieldtypes/fieldtypes.h \
    utils/filters/kalman/kalmanfilter.h \
    utils/filters/kalman/kalmanstate.h \
    utils/filters/kalman/matrix.h \
    utils/filters/loss/lossfilter.h \
    utils/filters/multiobject/multiobject.h \
    utils/filters/multiobject/multiobjectfilter.h \
    utils/filters/noise/noisefilter.h \
    utils/mrctimer/mrctimer.h \
    utils/types/ball/ball.h \
    utils/types/object.h \
    utils/types/robot/robot.h

DISTFILES += \
    include/3rd_party/protobuf/command.proto \
    include/3rd_party/protobuf/common.proto \
    include/3rd_party/protobuf/packet.proto \
    include/3rd_party/protobuf/protobuf.sh \
    include/3rd_party/protobuf/replacement.proto
