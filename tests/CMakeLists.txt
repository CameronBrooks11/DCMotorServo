cmake_minimum_required(VERSION 3.16)
project(DCMotorServoTests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
# br3ttb/PID 1.2.1, the version the PlatformIO registry serves.
FetchContent_Declare(pid
  GIT_REPOSITORY https://github.com/br3ttb/Arduino-PID-Library.git
  GIT_TAG 524a4268fc01e6ea397e7fc5b5d820741e9b662f)
FetchContent_MakeAvailable(pid)

add_library(dcmotorservo STATIC
  ../src/DCMotorServo.cpp
  ../src/DCMotorTacho.cpp
  ${pid_SOURCE_DIR}/PID_v1.cpp
  fakes/Arduino.cpp)
target_include_directories(dcmotorservo PUBLIC ../src fakes ${pid_SOURCE_DIR})
target_compile_definitions(dcmotorservo PUBLIC ARDUINO=100)
target_compile_options(dcmotorservo PRIVATE -Wall -Wextra)

enable_testing()
foreach(name test_servo_smoke test_servo_stop test_tacho_restart)
  add_executable(${name} ${name}.cpp)
  target_link_libraries(${name} PRIVATE dcmotorservo)
  add_test(NAME ${name} COMMAND ${name})
endforeach()
