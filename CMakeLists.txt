set(CMAKE_SYSTEM_NAME Generic)

set(CMAKE_C_COMPILER /usr/bin/avr-gcc)
set(CMAKE_CXX_COMPILER /usr/bin/avr-g++)
set(CMAKE_FIND_ROOT_PATH /usr/lib/avr)

set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)


cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_VERBOSE_MAKEFILE ON)


project("ledcristmas")

SET(flags "-mmcu=atmega8 -DF_CPU=16000000UL -Wall")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${flags}")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${flags}")


file(GLOB_RECURSE EXE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c)

add_executable(${PROJECT_NAME} ${EXE_SRC})


add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND
	avr-size -A -t ${PROJECT_NAME})
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND
	avr-objcopy -j .text -j .data -O ihex ${PROJECT_NAME} ${PROJECT_NAME}.hex)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND
	avr-objcopy -j .eeprom --change-section-lma .eeprom=0 -O ihex ${PROJECT_NAME} ${PROJECT_NAME}_eeprom.hex)

#program to board
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND
	avrdude -v -D -patmega8 -carduino -P/dev/ttyUSB3 -b115200 -Uflash:w:${PROJECT_NAME}.hex:i)
