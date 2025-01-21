#ifndef ST_VISITORGRID4DSETVALUE_HPP
#define ST_VISITORGRID4DSETVALUE_HPP

#include "st_visitorgrid4dsetvalue.h"

template< class DataT >
ST_VisitorGrid4DSetValue<DataT>::ST_VisitorGrid4DSetValue(HoughSpacePtrConst hough_space_ptr, DataT value_to_set) :
    SuperClass(),
    _hough_space_ptr(hough_space_ptr),
    _value_to_set( value_to_set )
{
}

template< class DataT >
ST_VisitorGrid4DSetValue<DataT>::~ST_VisitorGrid4DSetValue()
{
}

template< class DataT >
void ST_VisitorGrid4DSetValue<DataT>::visit(size_t levw, size_t levx, size_t levy, size_t levz, const ST_Beam4D* const beam)
{
    Q_UNUSED(beam);
    _hough_space_ptr->setValue(levw, levx, levy, levz, _value_to_set);
}


#endif // ST_VISITORGRID4DSETVALUE_HPP
