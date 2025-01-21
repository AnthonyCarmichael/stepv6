#include "st_step_filter_hough_space.h"

#include <ct_log/ct_logmanager.h>

ST_Step_Filter_Hough_space::ST_Step_Filter_Hough_space() : SuperClass()
{
    _ratio_thresh = 2.0;
}

QString ST_Step_Filter_Hough_space::description() const
{
    return tr("2 - Filtre un espace de Hough");
}

// Step detailled description
QString ST_Step_Filter_Hough_space::getStepDetailledDescription() const
{
    return tr("Si l'algorithme qui crée l'espace de Hough suppose que les directions des normales "
              "des points ne sont pas forcément dans la bonne direction, il génère des cercles dans les deux directions. "
              "C'est pourquoi, si nous prenons l'exemple d'un tronc, il existe des valeurs élevées au centre du "
              "tronc mais aussi tout autour à une certaine distance. Ce filtre à pour but de supprimer les valeurs "
              "élevées en dehors du tronc. Pour chaque point du nuage l'algorithme va cumuler les valeurs dans la direction "
              "de la normale et cumuler dans une autre variable les valeurs dans le sens opposé de la normale. Le ratio "
              "calculé est max(cumul1, cumul2)/min(cumul1, cumul2). Si ce ratio est inférieur au ratio minimum les deux "
              "valeurs sont supprimer de la grille, sinon la valeur la moins élevée est supprimée.");
}

CT_VirtualAbstractStep* ST_Step_Filter_Hough_space::createNewInstance() const
{
    return new ST_Step_Filter_Hough_space();
}

//////////////////// PROTECTED METHODS //////////////////

void ST_Step_Filter_Hough_space::declareInputModels(CT_StepInModelStructureManager& manager)
{
    manager.addResult(_inResult, tr("Scène(s)"));
    manager.setZeroOrMoreRootGroup(_inResult, _inZeroOrMoreRootGroup);
    manager.addGroup(_inZeroOrMoreRootGroup, _inGroup);
    manager.addItem(_inGroup, _in_hough_space,  tr("Hough space"));
}

void ST_Step_Filter_Hough_space::declareOutputModels(CT_StepOutModelStructureManager& manager)
{
    manager.addResultCopy(_inResult);
    manager.addItem(_inGroup, _out_hough_space, tr("Filtered Hough space"));
}

void ST_Step_Filter_Hough_space::fillPostInputConfigurationDialog(CT_StepConfigurableDialog* postInputConfigDialog)
{
    postInputConfigDialog->addDouble(tr("Ratio threshold"),
                                     tr(""),
                                     0.0001,
                                     100.0,
                                     4,
                                     _ratio_thresh,
                                     1.0,
                                     tr("Si le ratio calculé est inférieur à celui-ci "
                                        "toutes les valeurs du point sont supprimées, sinon "
                                        "les valeurs ayant eu le moins de votes cumulés sont "
                                        "supprimées. Voir la description de l'étape pour plus "
                                        "d'explication.")
                                     );
}

void ST_Step_Filter_Hough_space::compute()
{
    setProgress(0);

    for (CT_StandardItemGroup* group : _inGroup.iterateOutputs(_inResult))
    {
        if( isStopped() )
        {
            return;
        }

        ConstHoughSpacePtr in_hough_space = group->singularItem(_in_hough_space);

        HoughSpacePtr filtered_hough_space = in_hough_space->get_filtered_hs_using_fast_filter(_ratio_thresh, this);

        PS_LOG->addInfoMessage(LogInterface::error, tr("Min value %1").arg(filtered_hough_space->dataMin()));
        PS_LOG->addInfoMessage(LogInterface::error, tr("Max value %1").arg(filtered_hough_space->dataMax()));

        group->addSingularItem(_out_hough_space, filtered_hough_space);
    }

    setProgress(100);
}
