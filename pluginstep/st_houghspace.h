#ifndef ST_HOUGHSPACE_H
#define ST_HOUGHSPACE_H

#include "ct_itemdrawable/ct_grid4d_sparse.h"
#include "ct_itemdrawable/ct_pointsattributesnormal.h"
#include "ct_itemdrawable/abstract/ct_abstractitemdrawablewithpointcloud.h"

#include "ct_step/abstract/ct_abstractstep.h"

template< class DataT >
class ST_HoughSpace : public CT_Grid4D_Sparse<DataT>
{
    using SuperClass = CT_Grid4D_Sparse<DataT>;

    using PointCloudConst       = const CT_AbstractItemDrawableWithPointCloud;
    using PointCloudConstPtr    = PointCloudConst*;
    using NormalCloudConst      = const CT_PointsAttributesNormal;
    using NormalCloudConstPtr   = NormalCloudConst*;

    using Vec3i                 = Eigen::Vector3i;
    using Vec3f                 = Eigen::Vector3f;
    using Vec3d                 = Eigen::Vector3d;

    using Vec4i                 = Eigen::Vector4i;
    using Vec4f                 = Eigen::Vector4f;
    using Vec4d                 = Eigen::Vector4d;

    using HoughSpace            = ST_HoughSpace<DataT>;
    using HoughSpacePtr         = HoughSpace*;
    using ConstHoughSpace       = const HoughSpace;
    using ConstHoughSpacePtr    = ConstHoughSpace*;

public:
    //**********************************************//
    //           Constructors/Destructors           //
    //**********************************************//
    /**
      * \brief Default constructor
      *  Each attribute will be set to 0, nullptr or will be cleared
      */
    ST_HoughSpace();
    ST_HoughSpace(const ST_HoughSpace& other);

    /*!
     * \brief Initialisation constructor
     *
     * Grid is created thanks to bottom left point of bounding box (4D) and number of cells along each dimension.
     *
     * \param model Item model for creation
     * \param result Result containing the item
     * \param wmin Minimum W coordinate (bottom left corner)
     * \param xmin Minimum X coordinate (bottom left corner)
     * \param ymin Minimum Y coordinate (bottom left corner)
     * \param zmin Minimum Z coordinate (bottom left corner)
     * \param dimw Number of w levels
     * \param dimx Number of colums
     * \param dimy Number of rows
     * \param dimz Number of zlevels
     * \param resw Length of a cell on w
     * \param resx Length of a cell on x
     * \param resy Length of a cell on y
     * \param resz Length of a cell on z
     * \param na Value used to code NA
     * \param initValue Initialisation value for grid cells
     */
    ST_HoughSpace(double wmin,
                  double xmin,
                  double ymin,
                  double zmin,
                  size_t dimw,
                  size_t dimx,
                  size_t dimy,
                  size_t dimz,
                  double resw,
                  double resx,
                  double resy,
                  double resz,
                  DataT na,
                  DataT initValue);

    /*!
     * \brief Initialisation constructor
     *
     * Grid is created thanks to the bounding box (4D) of the grid
     *
     * \param model Item model for creation
     * \param result Result containing the item
     * \param wmin Minimum W coordinate (bottom left corner)
     * \param xmin Minimum X coordinate (bottom left corner)
     * \param ymin Minimum Y coordinate (bottom left corner)
     * \param zmin Minimum Z coordinate (bottom left corner)
     * \param wmax Maximum W coordinate (top right corner)
     * \param xmax Maximum X coordinate (top right corner)
     * \param ymax Maximum Y coordinate (top right corner)
     * \param zmax Maximum Z coordinate (top right corner)
     * \param resw Length of a cell on w
     * \param resx Length of a cell on x
     * \param resy Length of a cell on y
     * \param resz Length of a cell on z
     * \param na Value used to code NA
     * \param initValue Initialisation value for grid cells
     * \param coordConstructor Not used, only to ensure constructor different signatures
     */
    ST_HoughSpace(double wmin,
                  double xmin,
                  double ymin,
                  double zmin,
                  double wmax,
                  double xmax,
                  double ymax,
                  double zmax,
                  double resw,
                  double resx,
                  double resy,
                  double resz,
                  DataT na,
                  DataT initValue);

    /*!
     * \brief Factory
     *
     * Grid is created thanks to the bounding box (4D) of the grid
     *
     * \param model Item model for creation
     * \param result Result containing the item
     * \param wmin Minimum W coordinate (bottom left corner)
     * \param xmin Minimum X coordinate (bottom left corner)
     * \param ymin Minimum Y coordinate (bottom left corner)
     * \param zmin Minimum Z coordinate (bottom left corner)
     * \param wmax Maximum W coordinate (top right corner)
     * \param xmax Maximum X coordinate (top right corner)
     * \param ymax Maximum Y coordinate (top right corner)
     * \param zmax Maximum Z coordinate (top right corner)
     * \param resw Length of a cell on w
     * \param resx Length of a cell on x
     * \param resy Length of a cell on y
     * \param resz Length of a cell on z
     * \param na Value used to code NA
     * \param initValue Initialisation value for grid cells
     * \param coordConstructor Not used, only to ensure constructor different signatures
     */
    static ST_HoughSpace<DataT>* createGrid4DFromWXYZCoords(double wmin,
                                                            double xmin,
                                                            double ymin,
                                                            double zmin,
                                                            double wmax,
                                                            double xmax,
                                                            double ymax,
                                                            double zmax,
                                                            double resw,
                                                            double resx,
                                                            double resy,
                                                            double resz,
                                                            DataT na,
                                                            DataT initValue);

public:
    inline DataT valueHough(const int levw, const int levx, const int levy, const int levz) const
    {
        int idx[4] = {levw, levx, levy, levz};
        return _data(idx);
    }

    inline bool setValueHough(const int levw, const int levx, const int levy, const int levz, DataT value)
    {
        if( (levw >= _dimw) || (levx >= _dimx) || (levy >= _dimy) || (levz >= _dimz) )
        {
            return false;
        }

        int idx[4] = {levw, levx, levy, levz};
        _data.ref(idx) = value;

        return true;
    }

    inline bool addValueHough(int levw, int levx, int levy, int levz, DataT value )
    {
        if( (levw >= _dimw) || (levx >= _dimx) || (levy >= _dimy) || (levz >= _dimz) )
        {
            return false;
        }

        int idx[4] = {levw, levx, levy, levz};
        _data.ref(idx) += value;

        return true;
    }

public:
    static ST_HoughSpace<DataT>* createHoughSpaceFromCloud(PointCloudConstPtr point_cloud,
                                                           NormalCloudConstPtr normal_cloud,
                                                           double spatial_resolution, double radius_resolution,
                                                           double r_min, double r_max,
                                                           CT_AbstractStep* step_ptr = nullptr);

    /*!
     * \brief Destructor
     */
    ~ST_HoughSpace() override;

    inline Vec4d getResolutionsHough() const { return Vec4(wres(), xres(), yres(), zres()); }

    ST_HoughSpace<DataT>* get_filtered_hs_using_fast_filter(double ratio_thresh,
                                                            CT_AbstractStep* step_ptr = nullptr) const;

    ST_HoughSpace<DataT>* get_filtered_hs_using_fixed_threshold(DataT fixed_threshold,
                                                                CT_AbstractStep* step_ptr = nullptr) const;

    void get_local_maximas(int nei_size,
                           std::vector<Vec4i>& out_local_maximas,
                           bool sort_descending_order) const;

    void get_local_maximas_within_height_range(float zmin, float zmax, int nei_size,
                                               std::vector<Vec4i>& out_local_maximas,
                                               bool sort_descending_order) const;

    void get_local_maximas_in_bbox(const Vec4i& bot, const Vec4i& top, int nei_size,
                                   std::vector<Vec4i>& out_local_maximas,
                                   bool sort_descending_order) const;

    bool is_pixel_local_maxima( const Vec4i& pix, int nei_size ) const;

    inline DataT value(const Vec4i& pix) const
    {
        return CT_Grid4D_Sparse<DataT>::value(pix.w(), pix.x(), pix.y(), pix.z());
    }

    inline DataT value(int levw, int levx, int levy, int levz) const //override
    {
        return CT_Grid4D_Sparse<DataT>::value(levw, levx, levy, levz);
    }

    inline bool is_pixel_in_bbox( const Vec4i& pixel, const Vec4i& bot, const Vec4i& top ) const
    {
        if( pixel.w() < bot.w() ||
            pixel.x() < bot.x() ||
            pixel.y() < bot.y() ||
            pixel.z() < bot.z() ||
            pixel.w() > top.w() ||
            pixel.x() > top.x() ||
            pixel.y() > top.y() ||
            pixel.z() > top.z() )
        {
            return false;
        }

        return true;
    }

    inline double pixelToCartesianW( int levw ) const
    {
        return ( minW() + ( levw + 0.5 ) * wres() );
    }

    inline double pixelToCartesianX( int levx ) const
    {
        return ( minX() + ( levx + 0.5 ) * xres() );
    }

    inline double pixelToCartesianY( int levy ) const
    {
        return ( minY() + ( levy + 0.5 ) * yres() );
    }

    inline double pixelToCartesianZ( int levz ) const
    {
        return ( minZ() + ( levz + 0.5 ) * zres() );
    }

    inline Vec4d pixel_to_cartesian(const Vec4i& pix)  const
    {
        return pixel_to_cartesian(pix[0], pix[1], pix[2], pix[3]);
    }

    inline Vec4d pixel_to_cartesian(int levw, int levx, int levy, int levz) const
    {
        return Vec4d(pixelToCartesianW(levw),
                     pixelToCartesianX(levx),
                     pixelToCartesianY(levy),
                     pixelToCartesianZ(levz));
    }

    inline int cartesianToPixelW( double w ) const
    {
        if( w == _top[0] )
        {
            return _dimw - 1;
        }

        else
        {
            return floor( ( w - _bot[0] ) / _resw );
        }
    }

    inline int cartesianToPixelX( double x ) const
    {
        if( x == _top[1] )
        {
            return _dimx - 1;
        }

        else
        {
            return floor( ( x - _bot[1] ) / _resx );
        }
    }

    inline int cartesianToPixelY( double y ) const
    {
        if( y == _top[2] )
        {
            return _dimy - 1;
        }

        else
        {
            return floor( ( y - _bot[2] ) / _resy );
        }
    }

    inline int cartesianToPixelZ( double z ) const
    {
        if( z == _top[3] )
        {
            return _dimz - 1;
        }

        else
        {
            return floor( ( z - _bot[3] ) / _resz );
        }
    }

    inline Vec4i cartesian_to_pixel(const Vec4d& point)  const
    {
        return cartesian_to_pixel(point[0], point[1], point[2], point[3]);
    }

    inline Vec4i cartesian_to_pixel(double w, double x, double y, double z) const
    {
        return Vec4i(cartesianToPixelW(w),
                     cartesianToPixelX(x),
                     cartesianToPixelY(y),
                     cartesianToPixelZ(z));
    }

    bool isPixelIn( const Vec4i& pix ) const
    {
        return ( pix[0] >= 0 && pix[0] < wdim() &&
                 pix[1] >= 0 && pix[1] < xdim() &&
                 pix[2] >= 0 && pix[2] < ydim() &&
                 pix[3] >= 0 && pix[3] < zdim() );
    }


    inline Vec4i dim() const
    {
        return Vec4i(_dimw, _dimx, _dimy, _dimz);
    }

    inline void getMinMaxInNeighbourhood(const Vec4i& pixel, DataT& outMin, DataT& outMax, int size) const
    {
        outMin =  std::numeric_limits<DataT>::max();
        outMax = -std::numeric_limits<DataT>::max();

        DataT neiValue;
        Vec4i nei;
        for( nei[0] = pixel.w()-size ; nei[0] <= pixel.w()+size ; nei[0]++ )
        {
            for( nei[1] = pixel.x()-size ; nei[1] <= pixel.x()+size ; nei[1]++ )
            {
                for( nei[2] = pixel.y()-size ; nei[2] <= pixel.y()+size ; nei[2]++ )
                {
                    for( nei[3] = pixel.z()-size ; nei[3] <= pixel.z()+size ; nei[3]++ )
                    {
                        if( isPixelIn(nei) )
                        {
                            neiValue = valueHough(nei[0], nei[1], nei[2], nei[3]);

                            if ( neiValue < outMin )
                            {
                                outMin = neiValue;
                            }

                            if ( neiValue > outMax )
                            {
                                outMax = neiValue;
                            }
                        }
                    }
                }
            }
        }
    }

protected:
    // -------------------------------------------------------
    // Attributes
    // -------------------------------------------------------
    using SuperClass::nCells;
    using SuperClass::NA;
    using SuperClass::id;
    using SuperClass::alternativeDrawManager;
    using SuperClass::_top;
    using SuperClass::_bot;
    using SuperClass::_dimw;
    using SuperClass::_dimx;
    using SuperClass::_dimy;
    using SuperClass::_dimz;
    using SuperClass::_resw;
    using SuperClass::_resx;
    using SuperClass::_resy;
    using SuperClass::_resz;
    using SuperClass::_NAdata;
    using SuperClass::_dataMin;
    using SuperClass::_dataMax;
    using SuperClass::_data;

    PointCloudConstPtr  _point_cloud_const_ptr;
    NormalCloudConstPtr _normal_cloud_const_ptr;
};

#include "st_houghspace.hpp"

#endif // ST_HOUGHSPACE_H
