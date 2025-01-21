#include "st_beam4d.h"

bool ST_Beam4D::intersect ( const Vec4d& bot, const Vec4d& top, Vec4d& nearP, Vec4d& farP ) const
{
    double t0 = 0;
    double t1 = std::numeric_limits<double>::max();

    for( size_t coord = 0 ; coord < 4 ; coord++ )
    {
        if( !updateIntervals(bot(coord), top(coord), _origin(coord), _direction(coord), t0, t1) )
        {
            return false;
        }

        nearP(coord) = (_origin(coord) + _direction(coord)*t0);
        farP(coord) = (_origin(coord) + _direction(coord)*t1);
    }

    return true;

}

bool ST_Beam4D::intersect ( const Vec4d& bot, const Vec4d& top ) const
{
    double t0 = 0;
    double t1 = std::numeric_limits<double>::max();

    for( size_t coord = 0 ; coord < 4 ; coord++ )
    {
        if( !updateIntervals(bot(coord), top(coord), _origin(coord), _direction(coord), t0, t1) )
        {
            return false;
        }
    }

    return true;
}

bool ST_Beam4D::updateIntervals(const double &bot,const double &top,
                                const double &origin, const double &direction,
                                double &t0,double &t1 ) const
{
    // Update interval for bounding box slab
    double invRayDir = 1.0 / direction;
    double tNear = (bot - origin) * invRayDir;
    double tFar  = (top - origin) * invRayDir;

    // Update parametric interval from slab intersection $t$s
    if (tNear > tFar) std::swap(tNear, tFar);

    t0 = tNear > t0 ? tNear : t0;
    t1 = tFar  < t1 ? tFar  : t1;

    if (t0 > t1 && t0 - t1 > EPSILON_INTERSECTION_RAY ) // t0 being always > t1, (t0-t1) is always positive
    {
        return false;
    }

    return true;
}
