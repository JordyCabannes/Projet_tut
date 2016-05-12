#include_directories(../RegLC)
#include_directories(../utils)

SET(LOCAL_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/viewpaths.h
    )
    
SET(SWIG_SOURCES 
    ${SWIG_SOURCES}
    ${LOCAL_SOURCES}
    PARENT_SCOPE
    )
    
SET(SOURCES 
    ${SOURCES}
    ${LOCAL_SOURCES}
    PARENT_SCOPE
    )
    
SET(lib_indir geroli ../geroli/src .)

SET(ALGO_INC_DIRS
    ${ALGO_INC_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${lib_indir}
    PARENT_SCOPE
    )

