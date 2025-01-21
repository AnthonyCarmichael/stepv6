#ifndef ST_VISITORGRID4DINCREMENT_H
#define ST_VISITORGRID4DINCREMENT_H

// Inherits from abstract visitor
#include "st_abstractvisitorgrid4d.h"

template< class DataT >
class ST_HoughSpace;

template< class DataT >
class ST_VisitorGrid4DIncrement : public ST_AbstractVisitorGrid4D<DataT>
{
    using SuperClass                = ST_AbstractVisitorGrid4D<DataT>;
    using HoughSpace                = ST_HoughSpace<DataT>;
    using HoughSpacePtr             = HoughSpace*;
    using HoughSpacePtrConst        = HoughSpacePtr const;
    using ConstHoughSpace           = const HoughSpace;
    using ConstHoughSpacePtr        = ConstHoughSpace*;
    using ConstHoughSpacePtrConst   = ConstHoughSpacePtr const;

public:
    /*!
     * \brief ST_VisitorGrid4DIncrement
     *
     * Constructeur
     *
     * \param grid : grille que le visiteur viste
     */
    ST_VisitorGrid4DIncrement(HoughSpacePtrConst hough_space_ptr);

    /*!
      * Destructeur (rien a faire, il ne doit pas liberer l'image qu'il visite!!)
      */
    virtual ~ST_VisitorGrid4DIncrement();

    /*!
     * \brief visit
     *
     * \param levw : coordonnee du pixel a visiter
     * \param levx : coordonnee du pixel a visiter
     * \param levy : coordonnee du pixel a visiter
     * \param levz : coordonnee du pixel a visiter
     * \param beam : rayon qui traverse la grille
     */
    virtual void visit(size_t levw, size_t levx, size_t levy, size_t levz, const ST_Beam4D* const beam);

protected :
    HoughSpacePtrConst _hough_space_ptr;
};

// Inclusion des implementations template

#include "st_visitorgrid4dincrement.hpp"

#endif // ST_VISITORGRID4DINCREMENT_H
