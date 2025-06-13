cmake_minimum_required(VERSION 3.21)

project(astro C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ERFA REQUIRED erfa)

add_subdirectory(src/board)

# generate compiled stars
set(GENERATED_DIR ${CMAKE_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${GENERATED_DIR})

add_custom_command(
  OUTPUT ${GENERATED_DIR}/bsc.h ${GENERATED_DIR}/bsc.c
  COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${CMAKE_SOURCE_DIR}
          ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/gen_stars.py
  WORKING_DIRECTORY ${GENERATED_DIR}
  DEPENDS ${CMAKE_SOURCE_DIR}/gen_stars.py
  COMMENT "Generating stars...")

add_executable(astro src/main.c src/astro.c src/render.c ${GENERATED_DIR}/bsc.c)
target_include_directories(astro PRIVATE ${ERFA_INCLUDE_DIRS} ${GENERATED_DIR})

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" AND (CMAKE_SYSTEM_NAME STREQUAL
                                                 "Linux"))
  target_link_libraries(astro PRIVATE board::linux)
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "cortex-m4")
  target_link_libraries(astro PRIVATE board::stm32g4)
else()
  message(FATAL_ERROR "Could not find a board for ${CMAKE_SYSTEM_PROCESSOR}")
endif()

target_link_libraries(astro PRIVATE m ${ERFA_LIBRARIES} freertos_kernel)
