#include "st_stepextractcurvesfromhoughspace.h"

#include "ct_view/ct_groupbox.h"
#include <ct_log/ct_logmanager.h>

ST_StepExtractCurvesFromHoughSpace::ST_StepExtractCurvesFromHoughSpace() : SuperClass()
{
    _minValueForHoughSpaceMaxima = 10;
    _houghSpaceMaximaNeighbourhoodSize = 1;
    _alpha = 1.5;
    _beta = 1.5;
    _gama = 1.3;
    _minHeightForMaximumSearch = 1.1;
    _maxHeightForMaximumSearch = 2;
    _minValueForMaximumSearch = 5;
    _treeHeightMaximum = 30;
    _growCoeff = 1;
    _angleConeRecherche = 20;
    _tailleConeRecherche = 3;
    _tailleConeRechercheCm = 10;
    _seuilSigmaL1 = 0.75;
    _seuilSigmaL4 = 0.125;
    _nIterMaxOptim = 1000;
    _timeStep = 0.001;
    _longueurMin = 1;
    _threshGradMove = 0.001;
    _forkSearchActive = false;
    _nForkLevels = 1;
    _nSnakesMax = 1;
}

QString ST_StepExtractCurvesFromHoughSpace::description() const
{
    return tr("3 - Créer des courbes caractéristiques dans un espace de Hough");
}

// Step detailled description
QString ST_StepExtractCurvesFromHoughSpace::getStepDetailledDescription() const
{
    return tr("Utilise les valeurs élevées contenues dans un espace de Hough (une "
              "grille 4D) pour faire croitre des courbes caractéristiques. La quatrième "
              "dimension de la grille représentant les rayons des cercles issus "
              "de la transformée de Hough.");
}

CT_VirtualAbstractStep* ST_StepExtractCurvesFromHoughSpace::createNewInstance() const
{
    return new ST_StepExtractCurvesFromHoughSpace();
}

//////////////////// PROTECTED METHODS //////////////////

void ST_StepExtractCurvesFromHoughSpace::declareInputModels(CT_StepInModelStructureManager& manager)
{
    manager.addResult(_inResult, tr("Scène(s)"));
    manager.setZeroOrMoreRootGroup(_inResult, _inZeroOrMoreRootGroup);
    manager.addGroup(_inZeroOrMoreRootGroup, _inGroup);
    manager.addItem(_inGroup, _hough_space, tr("(Filtered) Hough space"));
}

void ST_StepExtractCurvesFromHoughSpace::declareOutputModels(CT_StepOutModelStructureManager& manager)
{
    manager.addResultCopy(_inResult);
    manager.addGroup(_inGroup, _outGroupOfSnakes, tr("Group of snakes"));
    manager.addGroup(_outGroupOfSnakes, _outGroupSingleSnake, tr("Single snake"));
    manager.addGroup(_outGroupSingleSnake, _outGroupSingleCircle, tr("Single circle group"));
    manager.addItem(_outGroupSingleCircle, _outCircle, tr("Cercles"));
}

void ST_StepExtractCurvesFromHoughSpace::fillPostInputConfigurationDialog(CT_StepConfigurableDialog* postInputConfigDialog)
{
    postInputConfigDialog->addInt(tr("Nombre d'arbre recherché"), "", 1, 100000, _nSnakesMax, tr("Renseignez le nombre d'arbre qu'il existe dans la scène"));
    postInputConfigDialog->addDouble(tr("Hauteur maximum de l'arbre"), "[m]", 0, 99999999, 3, _treeHeightMaximum);
    postInputConfigDialog->addDouble(tr("Longueur minimum de la courbe"), "", 0, 100, 2, _longueurMin, 1, tr("Indiquez la longueur minimum d'une courbe. Si une courbe n'a pas la longueur requise elle n'apparaitra pas dans les résultats."));
    postInputConfigDialog->addTitle(tr("<html><b>Initialisation de la recherche</b></html>"));
    postInputConfigDialog->addDouble(tr("Hauteur minimum des graines"), "[m]", 1e-06, 100, 6, _minHeightForMaximumSearch, 1, tr("Définissez la hauteur minimum à partir du sol où la recherche à le droit de débuter"));
    postInputConfigDialog->addDouble(tr("Hauteur maximum des graines"), "[m]", 1e-06, 100, 6, _maxHeightForMaximumSearch, 1, tr("Définissez la hauteur maximum à partir du sol où la recherche à le droit de débuter"));

    CT_GroupBox* gp = postInputConfigDialog->addNewGroupBox(tr("Mode avancée"));
    gp->addTitle(tr("<html><b>Energie des contours actifs</b></html>"));
    gp->addDouble(tr("Alpha"), "", 1e-06, 100, 6, _alpha, 1, tr("Joue sur l'élasticité de la courbe. Plus \"Alpha\" est petit et plus la courbe est élastique."));
    gp->addDouble(tr("Beta"), "", 1e-06, 100, 6, _beta, 1, tr("Joue sur la flexion de la courbe. Si \"Beta\" est grand la courbe sera plus lisse et si il est petit il permet à la courbe de former des vagues."));
    gp->addDouble(tr("Gamma"), "", 1e-06, 100, 6, _gama, 1, tr("Joue sur l'attraction des maxima. Plus \"Gamma\" est fort et plus la courbe est attirée par les forts maxima."));

    gp->addTitle(tr("<html><b>Initialisation de la recherche des maxima</b></html>"));
    gp->addInt(tr("Nombre de points minimum d'un cercle"), "", 1, 100000, _minValueForHoughSpaceMaxima, tr("Indiquez le nombre de points minimum qu'un cercle doit avoir pour être considéré comme maxima par l'algorithme de recherche."));
    gp->addInt(tr("Taille de fenêtre d'analyse des maxima locaux"), "", 1, 100000, _houghSpaceMaximaNeighbourhoodSize, tr("Lors de l'initialisation l'algorithme "
                                                                                                                          "recherche tous les maxima locaux. Si deux "
                                                                                                                          "maxima sont contenus dans la fenêtre d'analyse "
                                                                                                                          "le plus grand des deux (ou si ils sont égaux : le premier trouvé) "
                                                                                                                          "est gardé en mémoire. La valeur indiquée représente le nombre "
                                                                                                                          "de cellules. Plus cette valeur est grande et plus les maxima seront "
                                                                                                                          "rassemblés."));

    gp->addTitle(tr("<html><b>Croissance des contours actifs</b></html>"));
    gp->addDouble(tr("Vitesse de croissance"), "", 1e-06, 100, 6, _growCoeff, 1, tr("Choix entre vitesse et précision. Augmenter la valeur si vous souhaitez que le traitement aille plus vite cependant il sera moins précis."));
    gp->addDouble(tr("Angle du cone de recherche"), tr("[°]"), 5, 80, 4, _angleConeRecherche, 1);
    gp->addDouble(tr("Longueur du cone de recherche"), "[cm]", 0.0001, 99999, 4, _tailleConeRechercheCm, 1);
    gp->addDouble(tr("Seuil de fiabilité"), "", 0, 1, 4, _seuilSigmaL1, 1, tr("Représente la fiabilité de la croissance entre 0 et 1. Si le calcul de la fiabilité est inférieur à cette valeur "
                                                                              "l'algorithme arrête la croissance de la courbe courante. La fiabilité est par exemple réduite lors d'un départ "
                                                                              "de branchaison."));

    gp->addTitle(tr("<html><b>Optimisation</b></html>"));
    gp->addInt(tr("Nombre de pas d'optimisation maximum"), "", 0, 99999999, _nIterMaxOptim, tr("Indiquez une valeur forte pour que l'algorithme est assez de temps pour optimiser "
                                                                                               "la forme de la courbe. Evitez une valeur trop forte afin d'arrêtez l'optimisation "
                                                                                               "dans un temps raisonable."));
    gp->addDouble(tr("Vitesse d'optimisation"), "", 1e-06, 100, 6, _timeStep, 1, tr("Choix entre vitesse et précision. Augmenter la valeur si vous souhaitez que le traitement aille plus vite cependant il sera moins précis."));
    gp->addDouble(tr("Seuil de mouvement"), "", 0.0000001, 1, 7, _threshGradMove, 1, tr("Représente la différence entre deux optimisations de la courbe. Si cette différence est inférieur au seuil indiqué l'algorithme arrête l'optimisation de la courbe courante."));

    gp->addTitle("<html><b>Recherche de fourches</b></html>");
    gp->addBool("", "", tr("Active"), _forkSearchActive, tr("Active ou non la recherche de fourches"));
    gp->addInt(tr("Nombre de niveau de fourches par arbre"), "", 1, 3, _nForkLevels, tr("Représente le nombre de niveau de fourches à rechercher par arbre. "
                                                                                        "Un niveau plus élevé peut rendre le calcul incroyablement long dépendamment du nombre de fourches"
                                                                                        "et des fourches potentielles à différents niveaux."));
}

void ST_StepExtractCurvesFromHoughSpace::compute()
{
    setProgress(0);

    for (CT_StandardItemGroup* group : _inGroup.iterateOutputs(_inResult))
    {
        CT_StandardItemGroup* outGroupOfSnakes = new CT_StandardItemGroup();
        group->addGroup( _outGroupOfSnakes, outGroupOfSnakes );

        if( isStopped() )
        {
            return;
        }

        ConstHoughSpacePtr houhg_space_ptr = group->singularItem(_hough_space);
        _tailleConeRecherche = ceil( ( _tailleConeRechercheCm / 100.0 ) / houhg_space_ptr->xres() );
        const int nIterMaxGrow = qMax( static_cast<int>(_treeHeightMaximum / houhg_space_ptr->zres()), 1);

        // -----------------------------------------------------------------------
        // Get local maximas
        std::vector<Vec4i> local_maximas;
        houhg_space_ptr->get_local_maximas(_houghSpaceMaximaNeighbourhoodSize, local_maximas, true);

        if( local_maximas.empty() )
        {
            PS_LOG->addInfoMessage(LogInterface::error, tr("Pas de maxima locaux dans 'espace de Hough"));
            return;
        }
        else
        {
            PS_LOG->addInfoMessage(LogInterface::info, tr("Trouve %1 maximas locaux").arg( local_maximas.size() ));
        }

        // -----------------------------------------------------------------------
        // create a repulsion image
        RepulseImagePtr repulse_image_ptr = RepulseImage::createGrid4DFromWXYZCoords(houhg_space_ptr->minW(), houhg_space_ptr->minX(), houhg_space_ptr->minY(), houhg_space_ptr->minZ(),
                                                                                     houhg_space_ptr->maxW(), houhg_space_ptr->maxX(), houhg_space_ptr->maxY(), houhg_space_ptr->maxZ(),
                                                                                     houhg_space_ptr->wres(), houhg_space_ptr->xres(), houhg_space_ptr->yres(), houhg_space_ptr->zres(),
                                                                                     std::numeric_limits<int>::max(), 0);

        // -----------------------------------------------------------------------
        // Start a growing snake from local maximas
        std::vector<SnakePtr> snakes;
        for( const Vec4i& local_maxima : local_maximas )
        {
            if( snakes.size() >= _nSnakesMax || isStopped() )
            {
                break;
            }

            if( repulse_image_ptr->valueHough(local_maxima[0], local_maxima[1], local_maxima[2], local_maxima[3]) == 0 )
            {
                // Initialise un contours actif a partir du maxima courant
                SnakePtr curr_snake = new Snake(houhg_space_ptr,
                                                repulse_image_ptr,
                                                local_maxima,
                                                20 );

                // Start growing a new snake
                curr_snake->grow(nIterMaxGrow,
                                 _growCoeff,
                                 _angleConeRecherche,
                                 _tailleConeRecherche,
                                 _seuilSigmaL1);

                // If the snake is long enough
                if( curr_snake->length3D() > _longueurMin )
                {
                    // Resample snake
                    curr_snake->resample(0.1);

                    // Mark new snake as repulsive in repulsive image
                    curr_snake->markRepulsion(1.5);

                    // On relache le contour actif pour optimiser l'energie
                    curr_snake->relax(_nIterMaxOptim,
                                      _alpha, _beta, _gama,
                                      1.0,
                                      _timeStep,
                                      _threshGradMove ); // Equivalent de 1 millimetre par defaut

                    // Add snake to the global set of snakes
                    snakes.push_back(curr_snake);
                }
                else
                {
                    delete curr_snake;
                }
            }
        }

        // Transform snakes to circles and free allocated memory
        for( const SnakePtr snake_ptr : snakes )
        {
            CT_StandardItemGroup* outGroupSingleSnakes = new CT_StandardItemGroup();
            outGroupOfSnakes->addGroup( _outGroupSingleSnake, outGroupSingleSnakes );

            // Add circles to the computree step results
            std::vector< CirclePtr > raw_circles = snake_ptr->get_raw_circles();
            for( const CirclePtr circle : raw_circles )
            {
                CT_StandardItemGroup* outGroupCircle = new CT_StandardItemGroup();
                outGroupSingleSnakes->addGroup( _outGroupSingleCircle, outGroupCircle );
                outGroupCircle->addSingularItem( _outCircle, circle );
            }

            // Free allocated memory
            delete snake_ptr;
        }

        delete repulse_image_ptr;
    }


    PS_LOG->addInfoMessage(LogInterface::info, tr("ST_StepExtractCurvesFromHoughSpace DONE"));

    setProgress(100);
}
