#ifndef ST_VISITORGRID4DFASTFILTER_HPP
#define ST_VISITORGRID4DFASTFILTER_HPP

#include "st_visitorgrid4dfastfilter.h"

template< class DataT >
ST_VisitorGrid4DFastFilter<DataT>::ST_VisitorGrid4DFastFilter(ConstHoughSpacePtrConst hough_space_ptr) :
    SuperClass(),
    _sumOfVisitedVotes(0),
    _hough_space_ptr(hough_space_ptr)
{
}

template< class DataT >
ST_VisitorGrid4DFastFilter<DataT>::~ST_VisitorGrid4DFastFilter()
{
}

template< class DataT >
void ST_VisitorGrid4DFastFilter<DataT>::visit(size_t levw, size_t levx, size_t levy, size_t levz, const ST_Beam4D* const beam)
{
    Q_UNUSED(beam);
    _sumOfVisitedVotes += _hough_space_ptr->value( static_cast<int>(levw), static_cast<int>(levx), static_cast<int>(levy), static_cast<int>(levz) );
}

template< class DataT >
int ST_VisitorGrid4DFastFilter<DataT>::sumOfVisitedVotes() const
{
    return _sumOfVisitedVotes;
}

template< class DataT >
void ST_VisitorGrid4DFastFilter<DataT>::setSumOfVisitedVotes(int sumOfVisitedVotes)
{
    _sumOfVisitedVotes = sumOfVisitedVotes;
}

#endif // ST_VISITORGRID4DFASTFILTER_HPP
