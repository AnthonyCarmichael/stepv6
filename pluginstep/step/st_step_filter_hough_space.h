#ifndef ST_STEP_FILTER_HOUGH_SPACE_H
#define ST_STEP_FILTER_HOUGH_SPACE_H

#include "st_houghspace.h"

#include "ct_step/abstract/ct_abstractstep.h"

class ST_Step_Filter_Hough_space : public CT_AbstractStep
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

    using Vec3d = Eigen::Vector3d;
    using Vec3f = Eigen::Vector3f;
    using Vec4d = Eigen::Vector4d;

public:
    ST_Step_Filter_Hough_space();
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

    double                                  _ratio_thresh;
};

#endif // ST_STEP_FILTER_HOUGH_SPACE_H
