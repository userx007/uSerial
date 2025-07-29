cmake_minimum_required(VERSION 3.3)

project (uSerial)

if( CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT )
  message(STATUS "Setting default CMAKE_INSTALL_PREFIX path to ../")
  set(CMAKE_INSTALL_PREFIX "../" CACHE STRING "The path to use for make install" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC OR MSYS OR MINGW)
    SET(INSTALL_APP_DIR    "bin/windows")
else()
    SET(INSTALL_APP_DIR    "bin/linux")
endif()

add_subdirectory(sources)

install ( TARGETS uart_sender    DESTINATION ${INSTALL_APP_DIR} )
install ( TARGETS uart_responder DESTINATION ${INSTALL_APP_DIR} )




