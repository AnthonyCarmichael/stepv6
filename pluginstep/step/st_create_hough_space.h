#ifndef ST_CREATE_HOUGH_SPACE_H
#define ST_CREATE_HOUGH_SPACE_H

#include "st_houghspace.h"

#include "ct_step/abstract/ct_abstractstep.h"
#include "ct_itemdrawable/ct_pointsattributesnormal.h"
#include "ct_itemdrawable/abstract/ct_abstractitemdrawablewithpointcloud.h"

class ST_StepCreateHoughSpace : public CT_AbstractStep
{
    Q_OBJECT
    using SuperClass = CT_AbstractStep;

    using HoughSpaceValueType   = int;
    using HoughSpace            = ST_HoughSpace<HoughSpaceValueType>;

    using Vec3d                 = Eigen::Vector3d;
    using Vec3f                 = Eigen::Vector3f;
    using Vec4d                 = Eigen::Vector4d;

public:
    ST_StepCreateHoughSpace();
    QString description() const override;
    QString getStepDetailledDescription() const;
    CT_VirtualAbstractStep* createNewInstance() const override;

protected:
    void declareInputModels(CT_StepInModelStructureManager& manager) final;
    void declareOutputModels(CT_StepOutModelStructureManager& manager) final;
    void fillPostInputConfigurationDialog(CT_StepConfigurableDialog* postInputConfigDialog) final;
    void compute() final;

protected:
    CT_HandleInResultGroupCopy<>                                    _inResult;
    CT_HandleInStdZeroOrMoreGroup                                   _inZeroOrMoreRootGroup;
    CT_HandleInStdGroup<>                                           _inGroup;
    CT_HandleInSingularItem<CT_AbstractItemDrawableWithPointCloud>  _in_point_cloud;
    CT_HandleInSingularItem<CT_PointsAttributesNormal>              _in_normal_cloud;
    CT_HandleOutSingularItem<HoughSpace>                            _outHoughSpace;

    double _hs_space_resolution;
    double _hs_radius_resolution;
    double _hs_r_min;
    double _hs_r_max;
};

#endif // ST_CREATE_HOUGH_SPACE_H
