#ifndef ST_ABSTRACTVISITORGRID4D_H
#define ST_ABSTRACTVISITORGRID4D_H

#include "st_beam4d.h"

template< class DataT >
class ST_AbstractVisitorGrid4D
{
public:
    virtual void visit(size_t levw, size_t levx, size_t levy, size_t levz, const ST_Beam4D* const beam) = 0;
};

#endif // ST_ABSTRACTVISITORGRID4D_H
