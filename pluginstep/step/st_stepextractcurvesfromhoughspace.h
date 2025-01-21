#ifndef ST_STEPEXTRACTCURVESFROMHOUGHSPACE_H
#define ST_STEPEXTRACTCURVESFROMHOUGHSPACE_H

#include "st_houghspace.h"
#include "st_openactivecontours.h"

#include "ct_step/abstract/ct_abstractstep.h"
#include "ct_itemdrawable/ct_circle.h"

class ST_StepExtractCurvesFromHoughSpace : public CT_AbstractStep
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

    using RepulseImage              = ST_HoughSpace<int>;
    using RepulseImagePtr           = RepulseImage*;
    using RepulseImagePtrConst      = RepulseImagePtr const;

    using Vec3d = Eigen::Vector3d;
    using Vec4d = Eigen::Vector4d;
    using Vec4i = Eigen::Vector4i;

    using Circle                    = CT_Circle;
    using CirclePtr                 = Circle*;

    using Snake     = ST_OpenActiveContours<HoughSpaceValueType>;
    using SnakePtr  = Snake*;

public:
    ST_StepExtractCurvesFromHoughSpace();
    QString description() const override;
    QString getStepDetailledDescription() const;
    CT_VirtualAbstractStep* createNewInstance() const override;

protected:
    void declareInputModels(CT_StepInModelStructureManager& manager) final;
    void declareOutputModels(CT_StepOutModelStructureManager& manager) final;
    void fillPostInputConfigurationDialog(CT_StepConfigurableDialog* postInputConfigDialog) final;
    void compute() final;

protected:
    CT_HandleInResultGroupCopy<>        _inResult;
    CT_HandleInStdZeroOrMoreGroup       _inZeroOrMoreRootGroup;
    CT_HandleInStdGroup<>               _inGroup;
    CT_HandleInSingularItem<HoughSpace> _hough_space;

    CT_HandleOutStdGroup                _outGroupOfSnakes;
    CT_HandleOutStdGroup                _outGroupSingleSnake;
    CT_HandleOutStdGroup                _outGroupSingleCircle;
    CT_HandleOutSingularItem<Circle>    _outCircle;

private:
    // Step parameters
    int       _nIterMaxOptim;                       /*!<  */
    double    _treeHeightMaximum;                   /*!<  */
    double    _growCoeff;                           /*!<  */
    double    _timeStep;                            /*!<  */
    double    _alpha;                               /*! */
    double    _beta;                                /*! */
    double    _gama;                                /*! */
    int       _minValueForHoughSpaceMaxima;         /*!<  */
    int       _houghSpaceMaximaNeighbourhoodSize;   /*!<  */
    double    _angleConeRecherche;                  /*!<  */
    int       _tailleConeRecherche;                 /*!<  */
    double    _tailleConeRechercheCm;               /*!<  */
    double    _seuilSigmaL1;                        /*!<  */
    double    _seuilSigmaL4;                        /*!<  */
    double    _threshGradMove;                      /*!<  */
    double    _threshGradLength;                    /*!<  */
    double    _longueurMin;                         /*!<  */
    double    _minHeightForMaximumSearch;           /*!<  */
    double    _maxHeightForMaximumSearch;           /*!<  */
    int       _minValueForMaximumSearch;            /*!<  */
    double    _movementThresh;                      /*!<  */
    bool      _forkSearchActive;                    /*!<  */
    int       _nForkLevels;                         /*!<  */
    int       _nSnakesMax;                          /*!<  */
};

#endif // ST_STEPEXTRACTCURVESFROMHOUGHSPACE_H
