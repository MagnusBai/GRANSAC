CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(GRANSAC_SAMPLES)

MESSAGE("## project name: " ${PROJECT_NAME})

IF(MSVC)
  ADD_DEFINITIONS(-DNOMINMAX)
  # Another MSVC quirk. http://stackoverflow.com/questions/6832666/lnk2019-when-including-asio-headers-solution-generated-with-cmake
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc /MP") # For multi-threaded compilation on MSVC
ELSE()
  IF(APPLE)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++") # C++11 support on Apple clang
  ELSE()
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")# C++11 support on GCC
  ENDIF()
ENDIF()

FIND_PACKAGE(OpenCV 2.4 REQUIRED) # NEED OpenCV built with OpenGL support!
# OpenMP
FIND_PACKAGE(OpenMP)
IF(OPENMP_FOUND)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  MESSAGE(STATUS "Found OpenMP")
ENDIF()

MESSAGE("## cmake source dir: " ${CMAKE_SOURCE_DIR})

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})


SET(VC_LIBS ${OpenCV_LIBS})


# -- locations of test_sample files --
SET(CPP_SAMPLE_0 examples/sample0_line_fitting.cpp)
SET(CPP_SAMPLE_1 examples/sample1_lines_fitting.cpp)

# -- names of executables --
SET(EXE_SAMPLE_0 sample_0_fitting_line)
SET(EXE_SAMPLE_1 sample_1_fitting_lines)


ADD_EXECUTABLE(${EXE_SAMPLE_0} ${CPP_SAMPLE_0} ${HPP_FILES})
TARGET_LINK_LIBRARIES(${EXE_SAMPLE_0} ${VC_LIBS})

ADD_EXECUTABLE(${EXE_SAMPLE_1} ${CPP_SAMPLE_1} ${HPP_FILES})
TARGET_LINK_LIBRARIES(${EXE_SAMPLE_1} ${VC_LIBS})