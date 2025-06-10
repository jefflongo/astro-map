cmake_minimum_required(VERSION 3.21)

project(astro C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(PkgConfig REQUIRED)
pkg_check_modules(ERFA REQUIRED erfa)

add_subdirectory(src/board)

# add_custom_target( gen_bsc COMMAND python3
# ${CMAKE_CURRENT_SOURCE_DIR}/gen_stars.py BYPRODUCTS
# ${CMAKE_CURRENT_BINARY_DIR}/bsc.h ${CMAKE_CURRENT_BINARY_DIR}/bsc.c)

add_executable(astro src/main.c src/astro.c src/render.c src/bsc.c)
target_include_directories(astro PRIVATE ${ERFA_INCLUDE_DIRS})
target_link_libraries(astro PRIVATE m ${ERFA_LIBRARIES} board::desktop)
