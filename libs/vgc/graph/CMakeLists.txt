vgc_add_library(graph
    VGC_DEPENDENCIES
        dom
        geometry

    HEADER_FILES
        api.h
        cell.h
        graph.h

    CPP_FILES
        cell.cpp
        graph.cpp
)
