COMPUTREE += ctlibplugin ctlibclouds ctlibstep ctlibstepaddon ctlibio ctlibfilters ctlibaction ctlibstdactions ctlibmath ctliblas

MUST_USE_OPENCV = 1

include(../../computreev6/config/plugin_shared.pri)

TARGET = plug_stepv6

QT += concurrent

HEADERS += \
    $$CT_PREFIX_LIB/ctlibplugin/pluginentryinterface.h\
    ./st_abstractvisitorgrid4d.h \
    ./st_beam4d.h \
    ./st_grid4dwootraversalalgorithm.h \
    ./st_grid4dwootraversalalgorithm.hpp \
    ./st_houghspace.h \
    ./st_houghspace.hpp \
    ./st_openactivecontours.h \
    ./st_openactivecontours.hpp \
    ./st_pluginentry.h \
    ./st_steppluginmanager.h \
    ./st_visitorgrid4dfastfilter.h \
    ./st_visitorgrid4dfastfilter.hpp \
    ./st_visitorgrid4dincrement.h \
    ./st_visitorgrid4dincrement.hpp \
    ./st_visitorgrid4dsetvalue.h \
    ./st_visitorgrid4dsetvalue.hpp \
    ./step/st_create_hough_space.h \
    ./step/st_step_filter_hough_space.h \
    ./step/st_step_filter_hough_space_by_value.h \
    ./step/st_stepextractcurvesfromhoughspace.h \
    ./st_standardhoughspacedrawmanager.h \
    ./st_standardhoughspacedrawmanager.hpp

SOURCES += \
    ./st_abstractvisitorgrid4d.cpp \
    ./st_beam4d.cpp \
    ./st_grid4dwootraversalalgorithm.cpp \
    ./st_houghspace.cpp \
    ./st_openactivecontours.cpp \
    ./st_pluginentry.cpp \
    ./st_steppluginmanager.cpp \
    ./st_visitorgrid4dfastfilter.cpp \
    ./st_visitorgrid4dincrement.cpp \
    ./st_visitorgrid4dsetvalue.cpp \
    ./step/st_create_hough_space.cpp \
    ./step/st_step_filter_hough_space.cpp \
    ./step/st_step_filter_hough_space_by_value.cpp \
    ./step/st_stepextractcurvesfromhoughspace.cpp
