add_library(trinamic INTERFACE)

target_sources(trinamic INTERFACE
 ${CMAKE_CURRENT_LIST_DIR}/common.c
 ${CMAKE_CURRENT_LIST_DIR}/tmc_interface.c
 ${CMAKE_CURRENT_LIST_DIR}/tmc2130.c
 ${CMAKE_CURRENT_LIST_DIR}/tmc2130hal.c
 ${CMAKE_CURRENT_LIST_DIR}/tmc2209.c
 ${CMAKE_CURRENT_LIST_DIR}/tmc2209hal.c
 ${CMAKE_CURRENT_LIST_DIR}/tmc5160.c
 ${CMAKE_CURRENT_LIST_DIR}/tmc5160hal.c
)

target_include_directories(trinamic INTERFACE ${CMAKE_CURRENT_LIST_DIR})
