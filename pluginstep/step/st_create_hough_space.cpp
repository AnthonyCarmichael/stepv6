#include "st_create_hough_space.h"

#include <ct_itemdrawable/ct_beam.h>
#include <ct_log/ct_logmanager.h>
#include "st_grid4dwootraversalalgorithm.h"
#include "st_visitorgrid4dincrement.h"

ST_StepCreateHoughSpace::ST_StepCreateHoughSpace() : SuperClass()
{
    _hs_space_resolution = 0.2f;
    _hs_radius_resolution = 0.1f;
    _hs_r_min = 0.8;
    _hs_r_max = 1.2;
}

QString ST_StepCreateHoughSpace::description() const
{
    return tr("1 - Créer un espace de Hough à partir d'un nuage de points");
}

// Step detailled description
QString ST_StepCreateHoughSpace::getStepDetailledDescription() const
{
    return tr("Créer et remplir une grille 4D contenant les valeurs de la transformée de Hough "
              "pour chaque point du nuage. Cette étape à besoin d'avoir un nuage de point avec "
              "des normales. Si votre nuage de points ne possède pas de normales vous pouvez les "
              "obtenir à l'aide de l'étape d'estimation des normales contenue dans le "
              "plugin \"Toolkit\" par exemple. Chaque cellule de la grille représente un cercle "
              "potentiel en 3D et la transformée de Hough associe à chacun de ces cercles le nombre "
              "de points qui lui appartient. Des valeurs élevées vont se formées dans les cellules "
              "où le rayon du cercle s'ajuste bien par rapport à un amat de points. "
              "L'algorithme considère que la direction de la normale du point n'est pas "
              "correctement calculée c'est pourquoi le parcours de la grille est effectué dans la "
              "direction de la normale et dans le sens opposé.");
}

CT_VirtualAbstractStep* ST_StepCreateHoughSpace::createNewInstance() const
{
    return new ST_StepCreateHoughSpace();
}

//////////////////// PROTECTED METHODS //////////////////

void ST_StepCreateHoughSpace::declareInputModels(CT_StepInModelStructureManager& manager)
{
    manager.addResult(_inResult, tr("Scène(s)"));
    manager.setZeroOrMoreRootGroup(_inResult, _inZeroOrMoreRootGroup);
    manager.addGroup(_inZeroOrMoreRootGroup, _inGroup);
    manager.addItem(_inGroup, _in_point_cloud, tr("Point cloud"));
    manager.addItem(_inGroup, _in_normal_cloud, tr("Normal cloud"));
}

void ST_StepCreateHoughSpace::declareOutputModels(CT_StepOutModelStructureManager& manager)
{
    manager.addResultCopy(_inResult);
    manager.addItem(_inGroup, _outHoughSpace, tr("Computed Hough space"));
}

void ST_StepCreateHoughSpace::fillPostInputConfigurationDialog(CT_StepConfigurableDialog* postInputConfigDialog)
{
    postInputConfigDialog->addDouble(tr("Spatial resolution"),
                                     tr("[m]"),
                                     0.0001,
                                     2.0,
                                     4,
                                     _hs_space_resolution,
                                     1.0,
                                     tr("Hough space: spatial resolution (should be at least twice the radius resolution)")
                                     );

    postInputConfigDialog->addDouble(tr("Radius resolution"),
                                     tr("[m]"),
                                     0.0001,
                                     2.0,
                                     4,
                                     _hs_radius_resolution,
                                     1.0,
                                     tr("Hough space: radius resolution")
                                     );

    postInputConfigDialog->addDouble(tr("Radius min."),
                                     tr("[m]"),
                                     0.0001,
                                     2.0,
                                     4,
                                     _hs_r_min,
                                     1.0,
                                     tr("Hough space: radius min")
                                     );

    postInputConfigDialog->addDouble(tr("Radius max."),
                                     tr("[m]"),
                                     0.0001,
                                     2.0,
                                     4,
                                     _hs_r_max,
                                     1.0,
                                     tr("Hough space: radius max")
                                     );
}

void ST_StepCreateHoughSpace::compute()
{
    using PointCloudConst       = const CT_AbstractItemDrawableWithPointCloud;
    using PointCloudConstPtr    = PointCloudConst*;
    using NormalCloudConst      = const CT_PointsAttributesNormal;
    using NormalCloudConstPtr   = NormalCloudConst*;

    setProgress(0);

    for (CT_StandardItemGroup* group : _inGroup.iterateOutputs(_inResult))
    {
        if( isStopped() )
        {
            return;
        }

        // -----------------------------------------------------------------------------------------------------------------
        // Get point cloud and normal cloud from computree previous step
        PointCloudConstPtr  inPointCloud = group->singularItem(_in_point_cloud);
        NormalCloudConstPtr inNormalCloud = group->singularItem(_in_normal_cloud);

        // -----------------------------------------------------------------------------------------------------------------
        // Initialize empty Hough space according to the bounding box of the input point cloud
        Vec3d bbox_bot;
        Vec3d bbox_top;
        inPointCloud->boundingBox(bbox_bot, bbox_top);

        HoughSpace* outHoughSpace = HoughSpace::createHoughSpaceFromCloud(inPointCloud, inNormalCloud,
                                                                          _hs_space_resolution, _hs_radius_resolution,
                                                                          _hs_r_min, _hs_r_max,
                                                                          this);

        PS_LOG->addInfoMessage(LogInterface::error, tr("Min value %1").arg(outHoughSpace->dataMin()));
        PS_LOG->addInfoMessage(LogInterface::error, tr("Max value %1").arg(outHoughSpace->dataMax()));
        PS_LOG->addInfoMessage(LogInterface::error, tr("Non zero cells %1").arg(outHoughSpace->countNonZeroCells()));

        // -----------------------------------------------------------------------------------------------------------------
        // Add computed Hough space to the step's output(s)
        group->addSingularItem(_outHoughSpace, outHoughSpace);
    }

    setProgress(100);
}
