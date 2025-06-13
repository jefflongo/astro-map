if((CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64") AND (CMAKE_SYSTEM_NAME STREQUAL
                                                   "Linux"))
  add_subdirectory(linux)
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "cortex-m4")
  add_subdirectory(stm32g4)
else()
  message(FATAL_ERROR "Could not find a board for ${CMAKE_SYSTEM_PROCESSOR}")
endif()
