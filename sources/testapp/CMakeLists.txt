cmake_minimum_required(VERSION 3.10)
project(SerialTools)

add_executable(serial_responder  src/serial_responder.cpp)
add_executable(serial_sender     src/serial_sender.cpp)

# Link libraries to each executable
target_link_libraries(serial_responder
    PRIVATE
    uLogger
    uSerial
)

target_link_libraries(serial_sender
    PRIVATE
    uLogger
    uSerial
)
