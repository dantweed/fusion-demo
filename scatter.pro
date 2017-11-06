android|ios|winrt {
    error( "This example is not supported for android, ios, or winrt." )
}

!include( examples.pri ) {
    error( "Couldn't find the examples.pri file!" )
}

SOURCES += scatterdatamodifier.cpp \
    wrnch/Camera.cpp \
    wrnch/Device.cpp \
    wrnch/DeviceState.cpp \
    wrnch/IMU.cpp \
    wrnch/kalman.cpp \    
    main.cpp

HEADERS += scatterdatamodifier.h \
    wrnch/etk/array.h \
    wrnch/etk/bits.h \
    wrnch/etk/conversions.h \
    wrnch/etk/etk.h \
    wrnch/etk/filters.h \
    wrnch/etk/forward_list.h \
    wrnch/etk/fuzzy.h \
    wrnch/etk/linked_list.h \
    wrnch/etk/list.h \
    wrnch/etk/loop_range.h \
    wrnch/etk/math_util.h \
    wrnch/etk/matrix.h \
    wrnch/etk/navigation.h \
    wrnch/etk/objpool.h \
    wrnch/etk/pool.h \
    wrnch/etk/pool_ptr.h \
    wrnch/etk/quaternion.h \
    wrnch/etk/ring_buffer.h \
    wrnch/etk/rope.h \
    wrnch/etk/sigslot.h \
    wrnch/etk/smrt_ptr.h \
    wrnch/etk/state_machine.h \
    wrnch/etk/staticstring.h \
    wrnch/etk/stm.h \
    wrnch/etk/stream.h \
    wrnch/etk/time.h \
    wrnch/etk/tokeniser.h \
    wrnch/etk/types.h \
    wrnch/etk/vector.h \
    wrnch/ArrayArithmetic.h \
    wrnch/Camera.h \
    wrnch/Device.h \
    wrnch/DeviceState.h \
    wrnch/IMU.h \
    wrnch/kalman.h \
    wrnch/MDKalman.h \

QT += widgets

OTHER_FILES += doc/src/* \
               doc/images/*
