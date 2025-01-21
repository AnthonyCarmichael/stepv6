#include "st_steppluginmanager.h"
#include "step/st_create_hough_space.h"
#include "step/st_step_filter_hough_space.h"
#include "step/st_step_filter_hough_space_by_value.h"
#include "step/st_stepextractcurvesfromhoughspace.h"

ST_StepPluginManager::ST_StepPluginManager() : CT_AbstractStepPlugin()
{
}

ST_StepPluginManager::~ST_StepPluginManager()
{
}

QString ST_StepPluginManager::getPluginRISCitation() const
{
    return "TY  - COMP\n"
           "TI  - Plugin Step v6\n"
           "AU  - Piboule, Alexandre\n"
           "AU  - Ravaglia, Joris\n"
           "AU  - Krebs, Michael\n"
           "PB  - \n"
           "PY  - \n"
           "UR  - No url yet\n"
           "ER  - \n";
}


bool ST_StepPluginManager::loadGenericsStep()
{
    addNewPointsStep<ST_StepCreateHoughSpace>(CT_StepsMenu::LP_Points);
    addNewPointsStep<ST_Step_Filter_Hough_space>(CT_StepsMenu::LP_Points);
    addNewPointsStep<ST_Step_Filter_Hough_Space_By_Value>(CT_StepsMenu::LP_Points);
    addNewPointsStep<ST_StepExtractCurvesFromHoughSpace>(CT_StepsMenu::LP_Points);
    return true;
}

bool ST_StepPluginManager::loadOpenFileStep()
{
    return true;
}

bool ST_StepPluginManager::loadCanBeAddedFirstStep()
{
    return true;
}

bool ST_StepPluginManager::loadActions()
{
    return true;
}

bool ST_StepPluginManager::loadExporters()
{
    return true;
}

bool ST_StepPluginManager::loadReaders()
{
    return true;
}

