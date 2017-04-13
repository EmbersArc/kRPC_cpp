TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt
LIBS += \
    -lprotobuf \
    -lkrpc

SOURCES += \
    Arm_VesselControl.cpp \
    CalculateWheelTorque.cpp \
    IKSolver.cpp \
    main_arm.cpp \
    pid.cpp \
    TupleOperations.cpp

HEADERS += \
    Arm_VesselControl.h \
    CalculateWheelTorque.h \
    IKSolver.h \
    pid.h \
    TupleOperations.h
