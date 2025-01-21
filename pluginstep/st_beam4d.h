#ifndef ST_BEAM4D_H
#define ST_BEAM4D_H

#include <Eigen/Core>

#ifndef EPSILON_INTERSECTION_RAY
    /*! \def    EPSILON_INTERSECTION_RAY
                Redefinition of the zero for the ray-box intersection algorithm */
    #define EPSILON_INTERSECTION_RAY 0.000001    // 10^-6
#endif

class ST_Beam4D
{
    using Vec3d = Eigen::Vector3d;
    using Vec4d = Eigen::Vector4d;

public:
    //********************************************//
    //         Constructors/Destructors           //
    //********************************************//
    /*!
    *  \brief Constructor
    *
    *  Constructor of the class
    *
    *  \param origin : origin of the ray
    *  \param direction : direction of the ray
    */
    inline ST_Beam4D(const Vec4d& origin, const Vec4d& direction ) :
        _origin(origin)
    {
        assert( direction.norm() != 0 );
        _direction = direction.normalized();
    }

    inline ST_Beam4D(const Vec3d& origin_3d, const Vec3d& direction_3d ) :
        _origin(0, origin_3d[0], origin_3d[1], origin_3d[2])
    {
        assert( direction_3d.norm() != 0 );
        Vec3d direction_3d_normalized = direction_3d.normalized();
        _direction = Vec4d( 1.0, direction_3d_normalized[0], direction_3d_normalized[1], direction_3d_normalized[2] );
    }

    /*!
    *  \brief Destructor
    *
    *  Destructor of the class
    *
    */
    inline ~ST_Beam4D()
    {
    }

    //********************************************//
    //                Operators                   //
    //********************************************//
    /*!
    *  \brief Access operator
    *
    *  \param t : time spent by the ray
    *
    *  \return Returns the point of the ray at a given time
    */
    inline Vec4d operator() (double t) const
    {
        return _origin + t * _direction;;
    }

    //********************************************//
    //                  Getters                   //
    //********************************************//
    /*!
    *  \brief Getter of the class
    *
    *  \return Returns the origin of the ray
    */
    inline Vec4d getOrigin () const { return _origin; }

    /*!
    *  \brief Getter of the class
    *
    *  \return Returns the direction of the ray
    */
    inline Vec4d getDirection () const { return _direction; }

    //********************************************//
    //                  Setters                   //
    //********************************************//

    /*!
    *  \brief Setter of the class
    */
    inline void setOrigin ( const Vec4d& origin )
    {
        _origin = origin;
    }

    /*!
    *  \brief Setter of the class
    */
    inline void setDirection ( const Vec4d& direction )
    {
        assert( direction.norm() != 0 );
        _direction = direction.normalized();
    }

    //********************************************//
    //                   Tools                    //
    //********************************************//
    /*!
    *  \brief Calculates the intersection between a grid and a ray
    *
    *  This method uses the algorithm from Williams et al.
    *  ****************************************************************
    *  Williams, A., Barrus, S., & Morley, R. (2005).
    *  An efficient and robust ray-box intersection algorithm.
    *  ACM SIGGRAPH 2005
    *  1-4.
    *  *****************************************************************
    *
    *  \param bot : bottom left corner of the box to intersect
    *  \param top : top right corner of the box to intersect
    *  \param near : output, nearest 4D point of the intersection
    *  \param far : output, farest 4D point intersection
    *
    *  \return Returns false if no intersection was found, true else
    */
    bool intersect ( const Vec4d& bot, const Vec4d& top, Vec4d& nearP, Vec4d& farP ) const;

    /*!
    *  \brief Test the intersection between a grid and a ray
    *
    *  This method uses the algorithm from Williams et al.
    *  ****************************************************************
    *  Williams, A., Barrus, S., & Morley, R. (2005).
    *  An efficient and robust ray-box intersection algorithm.
    *  ACM SIGGRAPH 2005
    *  1-4.
    *  *****************************************************************
    *
    *  \param bot : bottom left corner of the box to intersect
    *  \param top : top right corner of the box to intersect
    *
    *  \return Returns false if no intersection was found, true else
    */
    bool intersect ( const Vec4d& bot, const Vec4d& top ) const;

private :
    /*!
     * \brief Utilitary function for intersect
     */
    bool updateIntervals(const double &bottomCorner,const double &upperCorner,
                         const double &origin, const double &direction,
                         double &t0,double &t1 ) const;

protected :
    /* **************************************************************** */
    /* Attributs de la classe                                           */
    /* **************************************************************** */
    Vec4d _origin;          /*!< Origin of the ray*/
    Vec4d _direction;       /*!< Direction of the ray*/
};

#endif // ST_BEAM4D_H
