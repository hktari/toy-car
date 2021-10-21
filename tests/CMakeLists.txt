cmake_minimum_required(VERSION 3.10)
# set the project name
project(toy-car VERSION 1.0)

include(FetchContent)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

FetchContent_Declare(Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG v2.11.1)

FetchContent_MakeAvailable(Catch2)

FetchContent_Declare(googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG release-1.10.0)

FetchContent_MakeAvailable(googletest)

FetchContent_Declare(arduino-mock
        GIT_REPOSITORY https://github.com/balp/arduino-mock.git
        GIT_TAG cmake3)

FetchContent_MakeAvailable(arduino-mock)

# add the executable
add_executable(toy-car toy-car-tests.cxx)

target_include_directories(toy-car
        PRIVATE "../build/_deps/arduino-mock-src/include"
        PUBLIC "../build/_deps/arduino-mock-src/include/arduino-mock"
        INTERFACE "../build/_deps/arduino-mock-src/include/arduino-mock"
        )
target_include_directories(toy-car
        PUBLIC "../build/_deps/googletest-src/googlemock/include"
        INTERFACE "../build/_deps/googletest-src/googlemock/include"
        PUBLIC "../build/_deps/googletest-src/googletest/include"
        INTERFACE "../build/_deps/googletest-src/googlemock/include"
        )        


target_link_libraries(toy-car
    arduino_mock
)
