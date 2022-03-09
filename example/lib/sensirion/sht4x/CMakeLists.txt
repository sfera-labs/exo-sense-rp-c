add_library(sht4x INTERFACE)
target_sources(sht4x INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/sht4x.c
        ${CMAKE_CURRENT_LIST_DIR}/sensirion_common.c
)
target_include_directories(sht4x INTERFACE ${CMAKE_CURRENT_LIST_DIR})