set(STM32CUBEG4_PATH ${PROJECT_SOURCE_DIR}/ext/STM32CubeG4)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/ext/cmake-stm32/cmake)
find_package(STM32Cube REQUIRED)

add_library(
  board_stm32g4 OBJECT
  ${CMAKE_CURRENT_SOURCE_DIR}/board.c ${CMAKE_CURRENT_SOURCE_DIR}/board_gps.c
  ${CMAKE_CURRENT_SOURCE_DIR}/platform.c ${CMAKE_CURRENT_SOURCE_DIR}/syscalls.c)
target_include_directories(board_stm32g4
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../include)
target_include_directories(board_stm32g4 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# configure STM32Cube
generate_stm32cube(
  stm32g431
  LL_DRIVERS
  dma
  exti
  gpio
  pwr
  rcc
  rtc
  tim
  usart
  utils)

target_link_libraries(
  board_stm32g4
  PRIVATE stm32::stm32g431_cmsis
          stm32::stm32g431_ll_dma
          stm32::stm32g431_ll_exti
          stm32::stm32g431_ll_gpio
          stm32::stm32g431_ll_pwr
          stm32::stm32g431_ll_rcc
          stm32::stm32g431_ll_tim
          stm32::stm32g431_ll_usart
          stm32::stm32g431_ll_utils
          freertos_kernel
          "-T${CMAKE_CURRENT_SOURCE_DIR}/stm32g431kbtx.ld")

# configure FreeRTOS
add_subdirectory(freertos)

set(FREERTOS_HEAP
    "4"
    CACHE STRING "" FORCE)
set(FREERTOS_PORT
    "GCC_ARM_CM4F"
    CACHE STRING "" FORCE)
add_subdirectory(${PROJECT_SOURCE_DIR}/ext/FreeRTOS
                 ${CMAKE_CURRENT_BINARY_DIR}/ext/FreeRTOS)

target_link_libraries(board_stm32g4 PUBLIC freertos_config freertos_kernel)

# export alias
add_transitive_object_library(board::stm32g4 board_stm32g4)
