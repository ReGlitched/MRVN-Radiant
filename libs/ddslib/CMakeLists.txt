add_library(ddslib STATIC
            ddslib.c
            ../ddslib.h
           )
target_include_directories(ddslib PRIVATE ${CMAKE_SOURCE_DIR}/libs)