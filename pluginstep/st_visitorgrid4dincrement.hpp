#ifndef ST_VISITORGRID4DINCREMENT_HPP
#define ST_VISITORGRID4DINCREMENT_HPP

#include "st_visitorgrid4dincrement.h"

template< class DataT >
ST_VisitorGrid4DIncrement<DataT>::ST_VisitorGrid4DIncrement(HoughSpacePtrConst hough_space_ptr) :
    SuperClass(),
    _hough_space_ptr(hough_space_ptr)
{
}

template< class DataT >
ST_VisitorGrid4DIncrement<DataT>::~ST_VisitorGrid4DIncrement()
{
}

template< class DataT >
void ST_VisitorGrid4DIncrement<DataT>::visit(size_t levw, size_t levx, size_t levy, size_t levz, const ST_Beam4D* const beam)
{
    Q_UNUSED(beam);
    _hough_space_ptr->addValue(static_cast<int>(levw), static_cast<int>(levx), static_cast<int>(levy), static_cast<int>(levz), 1);
}

#endif // ST_VISITORGRID4DINCREMENT_HPP
