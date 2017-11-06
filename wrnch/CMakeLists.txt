cmake_minimum_required(VERSION 3.8)
project(wrnch)

set(CMAKE_EXE_LINKER_FLAGS "-static -lpthread" )

set(CMAKE_CXX_STANDARD 11)
find_package( Qt5Core REQUIRED )
find_package( Qt5Widgets REQUIRED )
find_package( Qt5Gui REQUIRED )
find_package( Qt5DataVisualization )
set(SOURCE_FILES main.cpp IMU.cpp IMU.h Device.cpp Device.h Camera.cpp Camera.h DeviceState.cpp DeviceState.h kalman.cpp kalman.h MDKalman.h ArrayArithmetic.h)
add_executable(wrnch ${SOURCE_FILES})
target_link_libraries(wrnch Qt5::Core Qt5::Widgets Qt5::Gui Qt5::DataVisualization)