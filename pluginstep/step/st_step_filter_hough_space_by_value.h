#ifndef ST_STEP_FILTER_HOUGH_SPACE_BY_VALUE_H
#define ST_STEP_FILTER_HOUGH_SPACE_BY_VALUE_H
#include "st_houghspace.h"

#include "ct_step/abstract/ct_abstractstep.h"

class ST_Step_Filter_Hough_Space_By_Value : public CT_AbstractStep
{
    Q_OBJECT
    using SuperClass = CT_AbstractStep;

    using HoughSpaceValueType       = int;
    using HoughSpace                = ST_HoughSpace<HoughSpaceValueType>;
    using HoughSpacePtr             = HoughSpace*;
    using HoughSpacePtrConst        = HoughSpacePtr const;
    using ConstHoughSpace           = const HoughSpace;
    using ConstHoughSpacePtr        = ConstHoughSpace*;
    using ConstHoughSpacePtrConst   = ConstHoughSpacePtr const;

public:
    ST_Step_Filter_Hough_Space_By_Value();

    QString description() const override;
    QString getStepDetailledDescription() const;
    CT_VirtualAbstractStep* createNewInstance() const override;

protected:
    void declareInputModels(CT_StepInModelStructureManager& manager) final;
    void declareOutputModels(CT_StepOutModelStructureManager& manager) final;
    void fillPostInputConfigurationDialog(CT_StepConfigurableDialog* postInputConfigDialog) final;
    void compute() final;

protected:
    CT_HandleInResultGroupCopy<>            _inResult;
    CT_HandleInStdZeroOrMoreGroup           _inZeroOrMoreRootGroup;
    CT_HandleInStdGroup<>                   _inGroup;
    CT_HandleInSingularItem<HoughSpace>     _in_hough_space;
    CT_HandleOutSingularItem<HoughSpace>    _out_hough_space;

    HoughSpaceValueType                     _thresh;
};

#endif // ST_STEP_FILTER_HOUGH_SPACE_BY_VALUE_H
