vgc_add_library(workspace
    VGC_DEPENDENCIES
        dom
        graphics
        topology

    HEADER_FILES
        api.h
        edge.h
        face.h
        graphicelement.h
        layer.h
        renderer.h
        text.h
        workspace.h

    CPP_FILES
        edge.cpp
        face.cpp
        graphicelement.cpp
        layer.cpp
        renderer.cpp
        text.cpp
        workspace.cpp
)
