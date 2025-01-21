#ifndef ST_GRID4DWOOTRAVERSALALGORITHM_H
#define ST_GRID4DWOOTRAVERSALALGORITHM_H

#include <ct_itemdrawable/abstract/ct_abstractgrid4d.h>
#include "st_abstractvisitorgrid4d.h"
#include <st_beam4d.h>
#include <QList>

template< class DataT >
class ST_HoughSpace;

template< class DataT >
class ST_Grid4DWooTraversalAlgorithm
{
    using Vec4i = Eigen::Vector4i;
    using Vec4d = Eigen::Vector4d;

    using HoughSpace                = ST_HoughSpace<DataT>;
    using HoughSpacePtr             = HoughSpace*;
    using HoughSpacePtrConst        = HoughSpacePtr const;
    using ConstHoughSpace           = const HoughSpace;
    using ConstHoughSpacePtr        = ConstHoughSpace*;
    using ConstHoughSpacePtrConst   = ConstHoughSpacePtr const;

public:
    /*!
     * \brief Initialisation constructor
     * \param grid : Grid to be traversed
     * \param keepFirst :
     * \param list : List of visitors (that will modify the voxels they traverse) used
     */
    ST_Grid4DWooTraversalAlgorithm(ConstHoughSpacePtr hough_space_ptr, bool keepFirst, QList< ST_AbstractVisitorGrid4D<DataT> *>& list);

    /*!
      * \brief Destructor
      */
    ~ST_Grid4DWooTraversalAlgorithm();

protected :
    /*!
     * \brief initialise
     * Initialise l'algorithme (premier voxel traverse, les tMax, les deltaT, ... )
     */
    void initialise( ST_Beam4D& beam );

public :
    /*!
     * \brief compute method of the algorithm
     *
     * \param beam : origin and direction of the traversal
     */
    virtual void compute( ST_Beam4D& beam );

protected :
    /*!
     * \brief computeBBoxOfEnteringAndStopingVoxels
     *
     * Remet les vecteurs dans le bon ordre (toutes les coordonnees de bot sont inferieure aux coordonnees de top)
     *
     * \param botBBox
     * \param topBBox
     */
    void computeBBoxOfEnteringAndStopingVoxels( Vec4d& botBBox, Vec4d& topBBox );

public :
    /*!
     * \brief compute
     *
     * Starts the algorithm
     * Each traversed voxel is visited by each visitor
     * And is then stored in a list
     * This way the list of traversed pixels is available
     *
     * \param beam : origin and direction of the traversal
     * \param stopW : coordinate of the last voxel to be traversed (in case the traversal should not traverse the entire grid but only a bounded part of it)
     * \param stopX : coordinate of the last voxel to be traversed (in case the traversal should not traverse the entire grid but only a bounded part of it)
     * \param stopY : coordinate of the last voxel to be traversed (in case the traversal should not traverse the entire grid but only a bounded part of it)
     * \param stopZ : coordinate of the last voxel to be traversed (in case the traversal should not traverse the entire grid but only a bounded part of it)
     * \param traversed : output - list of the pixels traversed during the algorithm
     */
    virtual void compute(ST_Beam4D& beam,
                         unsigned int stopW, unsigned int stopX, unsigned int stopY, unsigned int stopZ,
                         bool addLastVoxel,
                         QVector<Vec4i*> &traversed );

/* **************************************************************** */
/* Attributs de la classe                                           */
/* **************************************************************** */
protected :
    // //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// //
    // Attributs relatifs a la grille a traverser                                                                                                   //
    // //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// //
    /**/    const CT_AbstractGrid4D*    _grid;          /*!< Grid to traverse */                                                                    //
    /**/    Vec4d                       _gridBot;       /*!< bottom left coordinates of the calibration grid*/                                      //
    /**/    Vec4d                       _gridTop;       /*!< upper right coordinates of the calibration grid*/                                      //
    /**/    Vec4d                       _gridRes;       /*!< Resolution of the calibration grid*/                                                   //
    /**/    Vec4d                       _gridDim;       /*!< Dimension of the calibration grid*/                                                    //
    // //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// //

    // //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// //
    // Attributs relatifs au comportement de l'algorithme lors de la traverse                                                                       //
    // //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// //
    /**/    bool                                        _keepFirst;     /*! Should the cell containing the beam origin be kept*/                    //
    /**/    QList< ST_AbstractVisitorGrid4D<DataT>* >   _visitorList;   /*! List of visitors visiting the cell of the grid */                       //
    // //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// //

    // //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// //
    // Variables d'initialisation de l'algorithme                                                                                                   //
    // //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// //
    /**/    Vec4d   _start;      /*!< Point d'entree du rayon dans la grille */                                                                      //
    /**/    Vec4d   _end;        /*!< Point de sortie du rayon dans la grille */                                                                     //
    /**/    Vec4d   _boundary;   /*!< Point de sortie du rayon dans le premier voxel traverse */                                                     //
    // //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// //

    // //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// //
    // Variables utilisees lors de la traverse a proprement parler                                                                                  //
    // //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// //
    /**/    Vec4d   _tMax;          /*!< "the value of t at which the ray crosses the first voxel boundary (along each direction)"*/                //
    /**/    Vec4d   _tDel;          /*!< "how far along the ray we must move (in units of t)" for each component "of such a movement to equal the width of a voxel"*/
    /**/    Vec4d   _stepAxis;      /*!< indicates for each axis wether the ray goes forward (in the same direction than the base vector => 1) or backward (the opposite direction => -1)*/
    /**/    Vec4d   _currentVoxel;  /*!< Coordonnees du voxel courant */                                                                            //
    /**/    int     _nextStepAxis;  /*!< Wich axis is going to be incremented next*/                                                                //
    // //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// //
};

#include "st_grid4dwootraversalalgorithm.hpp"

#endif // ST_GRID4DWOOTRAVERSALALGORITHM_H
