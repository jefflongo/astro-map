cmake_minimum_required(VERSION 3.21)

project(astro C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# generate compiled stars
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(GENERATED_DIR ${CMAKE_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${GENERATED_DIR})

add_custom_command(
  OUTPUT ${GENERATED_DIR}/bsc.h ${GENERATED_DIR}/bsc.c
  COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${CMAKE_SOURCE_DIR}
          ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/gen_stars.py ${VMAG_MAX}
  WORKING_DIRECTORY ${GENERATED_DIR}
  DEPENDS ${CMAKE_SOURCE_DIR}/gen_stars.py
  COMMENT "Generating stars...")

add_executable(astro ${CMAKE_SOURCE_DIR}/src/main.c
                     ${CMAKE_SOURCE_DIR}/src/astro.c ${GENERATED_DIR}/bsc.c)
target_include_directories(astro PRIVATE ${ERFA_INCLUDE_DIRS} ${GENERATED_DIR})

# build ERFA
file(GLOB ERFA_SOURCES ${CMAKE_SOURCE_DIR}/ext/erfa/src/*.c)
list(FILTER ERFA_SOURCES EXCLUDE REGEX ".*(erfaversion|t_erfa_c_extra)\\.c$")
add_library(erfa STATIC ${ERFA_SOURCES})
target_include_directories(erfa PUBLIC ${CMAKE_SOURCE_DIR}/ext/erfa/src)
target_link_libraries(astro PRIVATE m erfa)

# choose board
add_subdirectory(src/board)
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" AND (CMAKE_SYSTEM_NAME STREQUAL
                                                 "Linux"))
  target_link_libraries(astro PRIVATE board::linux)
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "cortex-m4")
  target_link_libraries(astro PRIVATE board::stm32g4)
else()
  message(FATAL_ERROR "Could not find a board for ${CMAKE_SYSTEM_PROCESSOR}")
endif()
