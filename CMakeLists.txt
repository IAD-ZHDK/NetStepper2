# minimal required cmake version
cmake_minimum_required(VERSION 3.7)

# you can set your own project name
project(NetStepper2)

# this should not be changed
set(CMAKE_C_STANDARD 99)

# add your source files
set(SOURCE_FILES
        src/buttons.c
        src/buttons.h
        src/encoder.c
        src/encoder.h
        src/end_stop.c
        src/end_stop.h
        src/l6470.c
        src/l6470.h
        src/led.c
        src/led.h
        src/main.c
        src/sharp.c
        src/sharp.h)

# create a fake library target
add_library(${CMAKE_PROJECT_NAME} ${SOURCE_FILES})

# include naos include paths
add_subdirectory(naos)
