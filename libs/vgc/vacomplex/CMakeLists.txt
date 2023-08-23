vgc_add_library(vacomplex
    VGC_DEPENDENCIES
        geometry

    CPP_HEADER_FILES
        api.h
        cell.h
        cellgeometry.h
        complex.h
        dataobject.h
        exceptions.h
        inbetweenedge.h
        inbetweenface.h
        inbetweenvertex.h
        keycycle.h
        keyedge.h
        keyedgegeometry.h
        keyface.h
        keyhalfedge.h
        keyvertex.h
        logcategories.h
        operations.h
        transform.h

        detail/operationsimpl.h

    CPP_SOURCE_FILES
        cell.cpp
        cellgeometry.cpp
        complex.cpp
        exceptions.cpp
        inbetweenedge.cpp
        inbetweenface.cpp
        inbetweenvertex.cpp
        keycycle.cpp
        keyedge.cpp
        keyedgegeometry.cpp
        keyface.cpp
        keyhalfedge.cpp
        keyvertex.cpp
        logcategories.cpp
        operations.cpp
        transform.cpp

        detail/operationsimpl.cpp
)
