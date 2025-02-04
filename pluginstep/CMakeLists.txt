set(STEP_HEADERS
    # TODO: find a better way to handle this...
    # NOTE: We need to explicitly add this header in order
    # for Qt to AUTOMOC it...
    ${ctlibplugin_SOURCE_DIR}/pluginentryinterface.h
    st_abstractvisitorgrid4d.h 
    st_beam4d.h 
    st_grid4dwootraversalalgorithm.h 
    st_grid4dwootraversalalgorithm.hpp 
    st_houghspace.h 
    st_houghspace.hpp 
    st_openactivecontours.h 
    st_openactivecontours.hpp 
    st_pluginentry.h 
    st_steppluginmanager.h 
    st_visitorgrid4dfastfilter.h 
    st_visitorgrid4dfastfilter.hpp 
    st_visitorgrid4dincrement.h 
    st_visitorgrid4dincrement.hpp 
    st_visitorgrid4dsetvalue.h 
    st_visitorgrid4dsetvalue.hpp 
    step/st_create_hough_space.h 
    step/st_step_filter_hough_space.h 
    step/st_step_filter_hough_space_by_value.h 
    step/st_stepextractcurvesfromhoughspace.h 
    st_standardhoughspacedrawmanager.h 
    st_standardhoughspacedrawmanager.hpp

)

set(STEP_SOURCES
    st_abstractvisitorgrid4d.cpp 
    st_beam4d.cpp 
    st_grid4dwootraversalalgorithm.cpp 
    st_houghspace.cpp 
    st_openactivecontours.cpp 
    st_pluginentry.cpp 
    st_steppluginmanager.cpp 
    st_visitorgrid4dfastfilter.cpp 
    st_visitorgrid4dincrement.cpp 
    st_visitorgrid4dsetvalue.cpp 
    step/st_create_hough_space.cpp 
    step/st_step_filter_hough_space.cpp 
    step/st_step_filter_hough_space_by_value.cpp 
    step/st_stepextractcurvesfromhoughspace.cpp
)

add_library(plug_step SHARED
    ${STEP_HEADERS}
    ${STEP_SOURCES}
)

target_include_directories(plug_step PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/
)

target_compile_definitions(plug_step PRIVATE USE_OPENCV)
target_link_libraries(plug_step
    ctlibcommon
    ctlibplugin
    ctlibclouds
    ctlibstep
    ctlibstepaddon
    ctlibio
    ctlibfilters
    ctlibaction
    ctlibstdactions
    ctlibstructure
    ctlibmath
    ctliblas
    muparser::muparser
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
)
