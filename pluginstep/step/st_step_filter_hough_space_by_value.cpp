#include "st_step_filter_hough_space_by_value.h"

#include <ct_log/ct_logmanager.h>

ST_Step_Filter_Hough_Space_By_Value::ST_Step_Filter_Hough_Space_By_Value() : SuperClass()
{
    _thresh = 10;
}

QString ST_Step_Filter_Hough_Space_By_Value::description() const
{
    return tr("2 (bis) - Filtre un espace de Hough selon un seuil fixe");
}

// Step detailled description
QString ST_Step_Filter_Hough_Space_By_Value::getStepDetailledDescription() const
{
    return tr("Seuil fixe");
}

CT_VirtualAbstractStep* ST_Step_Filter_Hough_Space_By_Value::createNewInstance() const
{
    return new ST_Step_Filter_Hough_Space_By_Value();
}

//////////////////// PROTECTED METHODS //////////////////

void ST_Step_Filter_Hough_Space_By_Value::declareInputModels(CT_StepInModelStructureManager& manager)
{
    manager.addResult(_inResult, tr("Scène(s)"));
    manager.setZeroOrMoreRootGroup(_inResult, _inZeroOrMoreRootGroup);
    manager.addGroup(_inZeroOrMoreRootGroup, _inGroup);
    manager.addItem(_inGroup, _in_hough_space,  tr("Input Hough space"));
}

void ST_Step_Filter_Hough_Space_By_Value::declareOutputModels(CT_StepOutModelStructureManager& manager)
{
    manager.addResultCopy(_inResult);
    manager.addItem(_inGroup, _out_hough_space, tr("Filtered Hough space with fixed threshold"));
}

void ST_Step_Filter_Hough_Space_By_Value::fillPostInputConfigurationDialog(CT_StepConfigurableDialog* postInputConfigDialog)
{
    postInputConfigDialog->addInt(tr("Fixed threshold"),
                                  tr(""),
                                  0,
                                  std::numeric_limits<HoughSpaceValueType>::max(),
                                  _thresh,
                                  tr("Seuil fixe"));
}

void ST_Step_Filter_Hough_Space_By_Value::compute()
{
    setProgress(0);

    for (CT_StandardItemGroup* group : _inGroup.iterateOutputs(_inResult))
    {
        if( isStopped() )
        {
            return;
        }

        ConstHoughSpacePtr in_hough_space = group->singularItem(_in_hough_space);

        HoughSpacePtr filtered_hough_space = in_hough_space->get_filtered_hs_using_fixed_threshold(_thresh, this);

        PS_LOG->addInfoMessage(LogInterface::error, tr("Min value %1").arg(filtered_hough_space->dataMin()));
        PS_LOG->addInfoMessage(LogInterface::error, tr("Max value %1").arg(filtered_hough_space->dataMax()));

        group->addSingularItem(_out_hough_space, filtered_hough_space);
    }

    setProgress(100);
}
