vgc_add_library(workspace
    VGC_DEPENDENCIES
        dom
        graphics
        topology

    HEADER_FILES
        api.h
        element.h
        workspace.h

    CPP_FILES
        element.cpp
        workspace.cpp
)
