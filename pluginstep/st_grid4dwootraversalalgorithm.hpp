#ifndef ST_GRID4DWOOTRAVERSALALGORITHM_HPP
#define ST_GRID4DWOOTRAVERSALALGORITHM_HPP

#include <st_grid4dwootraversalalgorithm.h>
#include <st_abstractvisitorgrid4d.h>
#include <st_beam4d.h>

// Utilise les constantes float max etc
#include <limits>

#define ST_GRID4DWOOTRAVERSALALGORITHM_EPSILONE_TRAVERSAL 0.001

template< class DataT >
ST_Grid4DWooTraversalAlgorithm<DataT>::ST_Grid4DWooTraversalAlgorithm(ConstHoughSpacePtr hough_space_ptr,
                                                                      bool keepFirst,
                                                                      QList< ST_AbstractVisitorGrid4D<DataT>* >& list)
{
    _visitorList = list;
    _grid = hough_space_ptr;
    hough_space_ptr->getMinCoordinates(_gridBot);
    hough_space_ptr->getMaxCoordinates(_gridTop);
    hough_space_ptr->getResolutions(_gridRes);
    hough_space_ptr->getDimensions(_gridDim);
    _keepFirst = keepFirst;
}

template< class DataT >
ST_Grid4DWooTraversalAlgorithm<DataT>::~ST_Grid4DWooTraversalAlgorithm()
{
}

template< class DataT >
void ST_Grid4DWooTraversalAlgorithm<DataT>::initialise( ST_Beam4D& beam )
{
    // /////////////////////////////////////////////////////////////////////////////////////////////
    // On remet le start dans la grille pour pas que ca plante si il etait aux extremites
    for( size_t coord = 0 ; coord < 4 ; coord++ )
    {
        if( _start[coord] < _gridBot[coord] && _start[coord] > _gridBot[coord] - ST_GRID4DWOOTRAVERSALALGORITHM_EPSILONE_TRAVERSAL )
        {
            _start[coord] = ( _gridBot[coord] + ST_GRID4DWOOTRAVERSALALGORITHM_EPSILONE_TRAVERSAL );
        }

        if ( _start[coord] > _gridTop[coord] && _start[coord] < _gridTop[coord] + ST_GRID4DWOOTRAVERSALALGORITHM_EPSILONE_TRAVERSAL )
        {
            _start[coord] = ( _gridTop[coord] - ST_GRID4DWOOTRAVERSALALGORITHM_EPSILONE_TRAVERSAL );
        }
    }
    // /////////////////////////////////////////////////////////////////////////////////////////////

    // ***********************************************************
    // Working with copies of the beam attributes avoids calling the getters
    // ***********************************************************
    Vec4d dir(beam.getDirection());

    // ***********************************************************
    // Initialising delta distances and the traversal directional sign
    // ***********************************************************
    for( size_t i = 0 ; i < 4 ; i++ )
    {
        if( dir[i] > 0 )
        {
            _stepAxis[i] = 1.0;
        }

        else if( dir[i] < 0 )
        {
            _stepAxis[i] = -1.0;
        }

        else
        {
            _stepAxis[i] = 0;
        }

        if ( dir[i] != 0 )
        {
            _tDel[i] = _gridRes[i] / fabs( dir[i] );
        }
    }

    // ***********************************************************
    // Get the index of the voxel from which the traversal starts (so we can access its coordinates in the grid system)
    // ***********************************************************
    int currentVoxelW, currentVoxelX, currentVoxelY, currentVoxelZ;

    // Le voxel courant est le voxel qui contient le point _start (voxel de depart de l'algorithme)
    _grid->coordAtWXYZ( _start[0], _start[1], _start[2], _start[3],
                       currentVoxelW, currentVoxelX, currentVoxelY, currentVoxelZ );

    _currentVoxel[0] = currentVoxelW;
    _currentVoxel[1] = currentVoxelX;
    _currentVoxel[2] = currentVoxelY;
    _currentVoxel[3] = currentVoxelZ;

    // ***********************************************************
    // Compute the leaving point of the ray inside the first voxel pierced
    // And initialise the tMax values
    // ***********************************************************
    for( size_t i = 0 ; i < 4 ; i++ )
    {
        if ( _stepAxis[i] > 0 )
        {
            _boundary[i] = ( _currentVoxel[i] + 1 ) * _gridRes[i] + _gridBot[i];
        }

        else
        {
            _boundary[i] = _currentVoxel[i] * _gridRes[i] + _gridBot[i];
        }

        if( dir[i] != 0 )
        {
            _tMax[i] = fabs( _boundary[i] - _start[i] ) / dir[i];
        }

        else
        {
            _tMax[i] = std::numeric_limits<double>::max();
        }
    }
}

template< class DataT >
void ST_Grid4DWooTraversalAlgorithm<DataT>::compute(ST_Beam4D& beam)
{
    // ***********************************************************
    // Check for the entering point of the beam into the grid
    // ***********************************************************
    // Au passage on calcule start et end, les points d'entree et de sortie du beam dans la grille
    if ( !beam.intersect(_gridBot, _gridTop, _start, _end) )
    {
        // The beam does not even intersect the grid, the algorithm ends
        return;
    }

    // Initialise les tMax, deltaT etc
    initialise( beam );

    // ***********************************************************
    // If the first voxel has to be pierced too, visit it
    // ***********************************************************
    if ( _keepFirst )
    {
        for ( int i = 0 ; i < _visitorList.size() ; ++i )
        {
            _visitorList.at(i)->visit( static_cast<size_t>( _currentVoxel[0] ),
                                       static_cast<size_t>( _currentVoxel[1] ),
                                       static_cast<size_t>( _currentVoxel[2] ),
                                       static_cast<size_t>( _currentVoxel[3] ),
                                       &beam );
        }
    }

    // ***********************************************************
    // Start the traversal loop until the beam reaches the outside of the grid
    // ***********************************************************
    float minTMax;
    while( true )
    {
        // Finds along which axis to do the next step
        // (it is the axis with the lowest tMax)
        _nextStepAxis = 0;

        minTMax = _tMax[0];
        for( int i = 1 ; i < 4 ; i++ )
        {
            if( _tMax[i] < minTMax )
            {
                _nextStepAxis = i;
                minTMax = _tMax[i];
            }
        }

        // Deplace le voxel courrant selon la direction trouvee precedement
        _currentVoxel[_nextStepAxis] = _currentVoxel[_nextStepAxis] + _stepAxis[_nextStepAxis];

        // Mise a jour du tMax correspondant a la direction trouvee precedement
        _tMax[_nextStepAxis] = _tMax[_nextStepAxis] + _tDel[_nextStepAxis];

        // Check if the voxel to be traversed is in the grid
        if( _currentVoxel[_nextStepAxis] >= 0 && _currentVoxel[_nextStepAxis] < _gridDim[_nextStepAxis] )
        {
            // Traverse the voxel
            for( int i = 0 ; i < _visitorList.size() ; ++i )
            {
                _visitorList.at(i)->visit( static_cast<size_t>( _currentVoxel[0] ),
                                           static_cast<size_t>( _currentVoxel[1] ),
                                           static_cast<size_t>( _currentVoxel[2] ),
                                           static_cast<size_t>( _currentVoxel[3] ),
                                           &beam );
            }
        }

        else
        {
            // Traversal ends
            return;
        }
    }
}

template< class DataT >
void ST_Grid4DWooTraversalAlgorithm<DataT>::computeBBoxOfEnteringAndStopingVoxels(Vec4d &botBBox, Vec4d &topBBox )
{
    float tmp;

    for( size_t coord = 0 ; coord < 4 ; coord++ )
    {
        if( botBBox[coord] > topBBox[coord] )
        {
            tmp = topBBox[coord];
            topBBox[coord] = botBBox[coord];
            botBBox[coord] = tmp;
        }
    }
}

template< class DataT >

void ST_Grid4DWooTraversalAlgorithm<DataT>::compute(ST_Beam4D& beam,
                                                    unsigned int stopW, unsigned int stopX, unsigned int stopY, unsigned int stopZ,
                                                    bool addLastVoxel,
                                                    QVector<Vec4i*>& traversed)
{
    // ***********************************************************
    // Check for the entering point of the beam into the grid
    // ***********************************************************
    // Au passage on calcule start et end, les points d'entree et de sortie du beam dans la grille
    if( !beam.intersect(_gridBot, _gridTop, _start, _end) )
    {
        // The beam does not even intersect the grid, the algorithm ends
        return;
    }

    // Initialise les tMax, deltaT etc
    initialise( beam );

    // Compute the bounding box of the entering and stopping voxel

    // Casting de la coordonnee du dernier pixel limite en Vec4d
    Vec4d  botBBox = _currentVoxel;
    Vec4d  stopVoxel( stopW, stopX, stopY, stopZ );
    Vec4d  topBBox( stopVoxel );

    computeBBoxOfEnteringAndStopingVoxels( botBBox, topBBox );

    // ***********************************************************
    // If the first voxel has to be pierced too, visit it and add it to the list of pierced voxels
    // ***********************************************************
    if( _keepFirst )
    {
        for( int i = 0 ; i < _visitorList.size() ; ++i )
        {
            _visitorList.at(i)->visit( static_cast<size_t>( _currentVoxel[0] ),
                                       static_cast<size_t>( _currentVoxel[1] ),
                                       static_cast<size_t>( _currentVoxel[2] ),
                                       static_cast<size_t>( _currentVoxel[3] ),
                                       &beam );
        }

        // Add the voxel to the list
        traversed.push_back( new Vec4i( _currentVoxel.cast<int>() ) );
    }

    // ***********************************************************
    // Start the traversal loop until the beam reaches the outside of the grid
    // ***********************************************************

    float minTMax;
    while( true )
    {
        // Finds along which axis to do the next step
        // (it is the axis with the lowest tMax)
        _nextStepAxis = 0;
        minTMax = _tMax[0];

        for( int i = 1 ; i < 4 ; i++ )
        {
            if( _tMax[i] < minTMax )
            {
                _nextStepAxis = i;
                minTMax = _tMax[i];
            }
        }

        // Deplace le voxel courrant selon la direction trouvee precedement
        _currentVoxel[_nextStepAxis] = _currentVoxel[_nextStepAxis] + _stepAxis[_nextStepAxis];

        // Mise a jour du tMax correspondant a la direction trouvee precedement
        _tMax[_nextStepAxis] = _tMax[_nextStepAxis] + _tDel[_nextStepAxis];

        // Check if the voxel to be traversed is in the grid
        if( _currentVoxel[_nextStepAxis] >= 0 && _currentVoxel[_nextStepAxis] < _gridDim[_nextStepAxis] )
        {
            // Traverse the voxel
            for( int i = 0 ; i < _visitorList.size() ; ++i )
            {
                _visitorList.at(i)->visit( static_cast<size_t>( _currentVoxel[0] ),
                                           static_cast<size_t>( _currentVoxel[1] ),
                                           static_cast<size_t>( _currentVoxel[2] ),
                                           static_cast<size_t>( _currentVoxel[3] ),
                                           &beam );

            }

            // Check if the voxel to be traversed is the stopping voxel
            // The commented line should be the right one but with numerical instabilities, the following is prefered
            // if ( _currentVoxel == stopVoxel )
            // It is preferable to check if the current voxel is outside the bounding box made by the starting voxel and the stopping voxel
            if ( _currentVoxel == stopVoxel ||
                 _currentVoxel(0) < botBBox(0) || _currentVoxel(0) > topBBox(0) ||
                 _currentVoxel(1) < botBBox(1) || _currentVoxel(1) > topBBox(1) ||
                 _currentVoxel(2) < botBBox(2) || _currentVoxel(2) > topBBox(2) ||
                 _currentVoxel(3) < botBBox(3) || _currentVoxel(3) > topBBox(3) )
            {
                if( addLastVoxel )
                {
                    qDebug() << "Ajout d'un pixel au tableau";

                    // Add the voxel to the list
                    traversed.push_back( new Vec4i( _currentVoxel.cast<int>() ));
                }

                return;
            }

            else
            {
                qDebug() << "Ajout d'un pixel au tableau";

                // Add the voxel to the list
                traversed.push_back( new Vec4i( _currentVoxel.cast<int>() ));
            }
        }

        // On est sorti de la grille on stoppe le raytracing
        else
        {
            return;
        }
    }
}

#endif // ST_GRID4DWOOTRAVERSALALGORITHM_HPP
