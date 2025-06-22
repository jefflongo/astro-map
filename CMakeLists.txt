cmake_minimum_required(VERSION 3.21)

project(astro C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdouble-promotion)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

option(VMAG_MAX "Maximum star magnitude to render" 6.0)
set(ERFA_SINGLE_PRECISION ON) # double precision not currently supported

set(GENERATED_DIR ${CMAKE_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${GENERATED_DIR})

# build ERFA
set(ERFA_DIR ${CMAKE_SOURCE_DIR}/ext/erfa)
file(GLOB ERFA_SOURCES ${ERFA_DIR}/src/*.c)

if(ERFA_SINGLE_PRECISION)
  # patch library for single precision
  set(PATCH_SCRIPT ${CMAKE_SOURCE_DIR}/scripts/double_to_float.py)
  set(NEW_ERFA_DIR ${GENERATED_DIR}/erfa)
  list(TRANSFORM ERFA_SOURCES REPLACE "^${ERFA_DIR}" "${NEW_ERFA_DIR}")

  add_custom_command(
    OUTPUT ${ERFA_SOURCES}
    COMMAND ${Python3_EXECUTABLE} ${PATCH_SCRIPT} ${ERFA_DIR}
    WORKING_DIRECTORY ${GENERATED_DIR}
    DEPENDS ${PATCH_SCRIPT} ${ERFA_DIR}/src
    COMMENT "Generating single precision ERFA library..."
    VERBATIM)

  set(ERFA_DIR ${NEW_ERFA_DIR})
endif()

# build BSC
set(GEN_STARS_SCRIPT ${CMAKE_SOURCE_DIR}/scripts/gen_stars.py)
set(BSC_DIR ${GENERATED_DIR}/bsc)
set(BSC_OPTIONS_FILE ${GENERATED_DIR}/bsc_options.txt)
file(MAKE_DIRECTORY ${BSC_DIR})

set(BSC_OPTIONS ${VMAG_MAX})
if(ERFA_SINGLE_PRECISION)
  list(APPEND BSC_OPTIONS "--single-precision")
endif()
file(WRITE ${BSC_OPTIONS_FILE} "${BSC_OPTIONS}")

add_custom_command(
  OUTPUT ${BSC_DIR}/bsc.h ${BSC_DIR}/bsc.c
  COMMAND ${Python3_EXECUTABLE} ${GEN_STARS_SCRIPT} ${BSC_OPTIONS}
  WORKING_DIRECTORY ${BSC_DIR}
  DEPENDS ${GEN_STARS_SCRIPT} ${BSC_OPTIONS_FILE}
  COMMENT "Generating stars (Vmag < ${VMAG_MAX})..."
  VERBATIM)

add_library(bsc STATIC ${BSC_DIR}/bsc.c)
target_include_directories(bsc PUBLIC ${BSC_DIR})

list(FILTER ERFA_SOURCES EXCLUDE REGEX
     ".*(erfaversion|t_erfa_c|t_erfa_c_extra)\\.c$")
add_library(erfa STATIC ${ERFA_SOURCES})
target_include_directories(erfa PUBLIC ${ERFA_DIR}/src)
target_link_libraries(erfa PRIVATE m)

add_executable(
  astro ${CMAKE_SOURCE_DIR}/src/main.c ${CMAKE_SOURCE_DIR}/src/astro.c
        ${CMAKE_SOURCE_DIR}/src/render.c)
target_link_libraries(astro PRIVATE bsc erfa m)

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
