add_library(dreamdrive64_shared INTERFACE)

target_sources(dreamdrive64_shared INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/pc64_rand.c
)

target_include_directories(dreamdrive64_shared INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
