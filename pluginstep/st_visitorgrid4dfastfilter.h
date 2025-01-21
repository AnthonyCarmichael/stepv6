#ifndef ST_VISITORGRID4DFASTFILTER_H
#define ST_VISITORGRID4DFASTFILTER_H

#include "st_abstractvisitorgrid4d.h"
#include "st_beam4d.h"

template< class DataT >
class ST_HoughSpace;

template< class DataT >
class ST_VisitorGrid4DFastFilter : public ST_AbstractVisitorGrid4D<DataT>
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
     * \brief ST_VisitorGrid4DFastFilter
     *
     * Constructeur
     *
     * \param grid : grille que le visiteur viste
     */
    ST_VisitorGrid4DFastFilter(ConstHoughSpacePtrConst hough_space_ptr);

    /*!
      * Destructeur (rien a faire, il ne doit pas liberer l'image qu'il visite!!)
      */
    virtual ~ST_VisitorGrid4DFastFilter();

    /*!
     * \brief visit
     *
     * \param levw : coordonnee du pixel a visiter
     * \param levx : coordonnee du pixel a visiter
     * \param levy : coordonnee du pixel a visiter
     * \param levz : coordonnee du pixel a visiter
     * \param beam : rayon qui traverse la grille
     */
    virtual void visit(size_t levw, size_t levx, size_t levy, size_t levz, const ST_Beam4D* const beam) override;

    int sumOfVisitedVotes() const;

    void setSumOfVisitedVotes(int sumOfVisitedVotes);

protected :
    ConstHoughSpacePtrConst _hough_space_ptr;
    int                     _sumOfVisitedVotes;
};

#include "st_visitorgrid4dfastfilter.hpp"

#endif // ST_VISITORGRID4DFASTFILTER_H
