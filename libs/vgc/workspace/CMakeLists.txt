vgc_add_library(workspace
    VGC_DEPENDENCIES
        dom
        graphics
        topology

    HEADER_FILES
        api.h
        element.h
        logcategories.h
        workspace.h

    CPP_FILES
        element.cpp
        logcategories.cpp
        workspace.cpp
)
