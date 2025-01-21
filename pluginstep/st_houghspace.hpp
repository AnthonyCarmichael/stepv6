#ifndef ST_HOUGHSPACE_HPP
#define ST_HOUGHSPACE_HPP

#include "st_houghspace.h"

#include "st_grid4dwootraversalalgorithm.h"
#include "st_visitorgrid4dincrement.h"
#include "st_visitorgrid4dfastfilter.h"
#include "st_visitorgrid4dsetvalue.h"

#include "ct_log/ct_logmanager.h"

template< class DataT >
ST_HoughSpace<DataT>::ST_HoughSpace() :
    SuperClass(),
    _point_cloud_const_ptr( nullptr ),
    _normal_cloud_const_ptr( nullptr )
{
}

template< class DataT >
ST_HoughSpace<DataT>::ST_HoughSpace(const ST_HoughSpace<DataT>& other) :
    SuperClass( other ),
    _point_cloud_const_ptr( other._point_cloud_const_ptr ),
    _normal_cloud_const_ptr( other._normal_cloud_const_ptr )
{
}

template< class DataT >
ST_HoughSpace<DataT>::ST_HoughSpace(double wmin,
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
                                    DataT initValue) :
    SuperClass(wmin, xmin, ymin, zmin,
               dimw, dimx, dimy, dimz,
               resw, resx, resy, resz,
               na, initValue),
    _point_cloud_const_ptr( nullptr ),
    _normal_cloud_const_ptr( nullptr )
{
}

template< class DataT >
ST_HoughSpace<DataT>::ST_HoughSpace(double wmin,
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
                                    DataT initValue) :
    SuperClass(wmin, xmin, ymin, zmin,
               wmax, xmax, ymax, zmax,
               resw, resx, resy, resz,
               na, initValue),
    _point_cloud_const_ptr( nullptr ),
    _normal_cloud_const_ptr( nullptr )
{
}

template< class DataT >
ST_HoughSpace<DataT>* ST_HoughSpace<DataT>::createGrid4DFromWXYZCoords(double wmin,
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
                                                                       DataT initValue)
{
    size_t dimw = ceil((wmax - wmin)/resw);
    size_t dimx = ceil((xmax - xmin)/resx);
    size_t dimy = ceil((ymax - ymin)/resy);
    size_t dimz = ceil((zmax - zmin)/resz);

    // to ensure a point exactly on a maximum limit of the grid will be included in the grid
    while (wmax >= (wmin + dimw * resw))
    {
        dimw++;
    }

    while (xmax >= (xmin + dimx * resx))
    {
        dimx++;
    }

    while (ymax >= (ymin + dimy * resy))
    {
        dimy++;
    }

    while (zmax >= (zmin + dimz * resz))
    {
        dimz++;
    }

    return new ST_HoughSpace<DataT>(wmin, xmin, ymin, zmin,
                                    dimw, dimx, dimy, dimz,
                                    resw, resx, resy, resz,
                                    na, initValue);
}

template< class DataT >
ST_HoughSpace<DataT>* ST_HoughSpace<DataT>::createHoughSpaceFromCloud(PointCloudConstPtr point_cloud,
                                                                      NormalCloudConstPtr normal_cloud,
                                                                      double spatial_resolution, double radius_resolution,
                                                                      double r_min, double r_max,
                                                                      CT_AbstractStep* step_ptr)
{
    // -----------------------------------------------------------------------------------------------------------------
    // Initialize empty Hough space according to the bounding box of the input point cloud
    Vec3d bbox_bot;
    Vec3d bbox_top;
    point_cloud->boundingBox(bbox_bot, bbox_top);

    HoughSpace* rslt_hough_space = HoughSpace::createGrid4DFromWXYZCoords(r_min, bbox_bot[0] - r_max, bbox_bot[1] - r_max, bbox_bot[2] - r_max,
                                                                          r_max, bbox_top[0] + r_max, bbox_top[1] + r_max, bbox_top[2] + r_max,
                                                                          radius_resolution, spatial_resolution, spatial_resolution, spatial_resolution,
                                                                          std::numeric_limits<DataT>::max(), 0);

    rslt_hough_space->_point_cloud_const_ptr = point_cloud;
    rslt_hough_space->_normal_cloud_const_ptr = normal_cloud;

    // On declare tout ce qui est necessaire pour faire le raytracing 4d
    ST_VisitorGrid4DIncrement<DataT>* incrementVisitor = new ST_VisitorGrid4DIncrement<DataT>(rslt_hough_space);
    QList< ST_AbstractVisitorGrid4D<DataT> * > visitorList;
    visitorList.push_back( incrementVisitor );

    // On declare un algorithme de raytracing 4D
    ST_Grid4DWooTraversalAlgorithm<DataT> traversal_algo_accumulate( rslt_hough_space, true, visitorList );

    // -----------------------------------------------------------------------------------------------------------------
    // Loop through all points and normals of the input point cloud and start raytracing inside Hough space
    size_t i_point = 0;
    size_t n_points = point_cloud->pointCloudIndex()->size();
    CT_PointIterator itPoint(point_cloud->pointCloudIndex());
    for( CT_PointIterator itPoint(point_cloud->pointCloudIndex()) ; itPoint.hasNext() ; i_point++ )
    {
        if( step_ptr != nullptr )
        {
            if( i_point % 100 == 0 )
            {
                // step_ptr->setProgress( static_cast<float>(i_point) * 100.0f / static_cast<float>(n_points) );
            }

            if( step_ptr->isStopped() )
            {
                return rslt_hough_space;
            }
        }

        const CT_Point&  currentPoint       = itPoint.next().currentPoint();
        const CT_Normal& currentCTNormal    = normal_cloud->constNormalAt(i_point);
        const Vec3d      currentNormal      = currentCTNormal.head(3).cast<double>();

        float normalLenght = currentNormal.norm();

        if( normalLenght != 0.0 )
        {
            ST_Beam4D beam_01( currentPoint, currentNormal );
            ST_Beam4D beam_02( currentPoint, -currentNormal );

            traversal_algo_accumulate.compute(beam_01);
            traversal_algo_accumulate.compute(beam_02);
        }
    }

    delete incrementVisitor;
    rslt_hough_space->computeMinMax();

    return rslt_hough_space;
}

template< class DataT >
ST_HoughSpace<DataT>::~ST_HoughSpace()
{
    _data.release();
}

template< class DataT >
ST_HoughSpace<DataT>* ST_HoughSpace<DataT>::get_filtered_hs_using_fast_filter(double ratio_thresh,
                                                                              CT_AbstractStep* step_ptr) const
{
    ST_HoughSpace<DataT>* filtered_hs = new ST_HoughSpace<DataT>( *this );

    // On declare tout ce qui est necessaire pour faire le raytracing 4d
    ST_VisitorGrid4DFastFilter<DataT>* filter_visitor = new ST_VisitorGrid4DFastFilter<DataT>( this );
    QList< ST_AbstractVisitorGrid4D<DataT>* > filter_visitors_list;
    filter_visitors_list.push_back( filter_visitor );

    ST_VisitorGrid4DSetValue<DataT>* set_value_visitor = new ST_VisitorGrid4DSetValue<DataT>(filtered_hs, static_cast<DataT>(0) );
    QList< ST_AbstractVisitorGrid4D<DataT>* > set_value_visitors_list;
    set_value_visitors_list.push_back( set_value_visitor );

    // On declare un algorithme de raytracing 4D
    ST_Grid4DWooTraversalAlgorithm<DataT> traversal_algo_accumulate( this, true, filter_visitors_list );
    ST_Grid4DWooTraversalAlgorithm<DataT> traversal_algo_set_zero( filtered_hs, false, set_value_visitors_list );

    // -----------------------------------------------------------------------------------------------------------------
    // Loop through all points and normals of the input point cloud and start raytracing inside Hough space
    size_t i_point = 0;
    size_t n_points = _point_cloud_const_ptr->pointCloudIndex()->size();
    CT_PointIterator itPoint(_point_cloud_const_ptr->pointCloudIndex());
    for( CT_PointIterator itPoint(_point_cloud_const_ptr->pointCloudIndex()) ; itPoint.hasNext() ; i_point++ )
    {
        if( step_ptr != nullptr )
        {
            if( i_point % 100 == 0 )
            {
                // step_ptr->setProgress( static_cast<float>(i_point) * 100.0f / static_cast<float>(n_points) );
            }

            if( step_ptr->isStopped() )
            {
                return filtered_hs;
            }
        }

        const CT_Point&  currentPoint       = itPoint.next().currentPoint();
        const CT_Normal& currentCTNormal    = _normal_cloud_const_ptr->constNormalAt(i_point);
        const Vec3d      currentNormal      = currentCTNormal.head(3).cast<double>();

        float normalLenght = currentNormal.norm();

        if( normalLenght != 0.0 )
        {
            ST_Beam4D beam_01( currentPoint, currentNormal );
            ST_Beam4D beam_02( currentPoint, -currentNormal );

            filter_visitor->setSumOfVisitedVotes( 0 );
            traversal_algo_accumulate.compute(beam_01);
            int n_votes_beam_01 = filter_visitor->sumOfVisitedVotes();

            filter_visitor->setSumOfVisitedVotes( 0 );
            traversal_algo_accumulate.compute(beam_02);
            int n_votes_beam_02 = filter_visitor->sumOfVisitedVotes();

            bool  beam_01_is_max = n_votes_beam_01 > n_votes_beam_02;
            float n_votes_max    = static_cast<float>( std::max( n_votes_beam_01, n_votes_beam_02 ) );
            float n_votes_min    = static_cast<float>( std::min( n_votes_beam_01, n_votes_beam_02 ) );
            float ratio          = n_votes_max / n_votes_min;

            if( ratio < ratio_thresh )
            {
                // Filtrer les deux directions
                traversal_algo_set_zero.compute(beam_01);
                traversal_algo_set_zero.compute(beam_02);
            }
            else if( beam_01_is_max )
            {
                // Filtre direction de beam 02
                traversal_algo_set_zero.compute(beam_02);
            }
            else
            {
                // Filtre direction de beam 01
                traversal_algo_set_zero.compute(beam_01);
            }
        }
    }

    delete filter_visitor;
    delete set_value_visitor;

    filtered_hs->computeMinMax();

    return filtered_hs;
}

template<class DataT>
ST_HoughSpace<DataT>* ST_HoughSpace<DataT>::get_filtered_hs_using_fixed_threshold(DataT fixed_threshold, CT_AbstractStep *step_ptr) const
{
    ST_HoughSpace<DataT>* filtered_hs = new ST_HoughSpace<DataT>( *this );

    cv::SparseMatConstIterator_<DataT> pixel_it = filtered_hs->_data.begin();
    cv::SparseMatConstIterator_<DataT> pixel_it_end = filtered_hs->_data.end();

    for( ; pixel_it != pixel_it_end ; ++pixel_it )
    {
        const cv::SparseMat::Node* curr_pixel_node = pixel_it.node();
        DataT curr_val = pixel_it.template value<DataT>();
        if( curr_val < fixed_threshold )
        {
            filtered_hs->setValue(curr_pixel_node->idx[0],
                                  curr_pixel_node->idx[1],
                                  curr_pixel_node->idx[2],
                                  curr_pixel_node->idx[3],
                                  0);
        }
    }

    return filtered_hs;
}

template< class DataT >
void ST_HoughSpace<DataT>::get_local_maximas(int nei_size,
                                             std::vector<Vec4i>& out_local_maximas,
                                             bool sort_descending_order) const
{
    out_local_maximas.clear();

    Vec4i bot(0,0,0,0);
    Vec4i top( _dimw, _dimx, _dimy, _dimz );

    get_local_maximas_in_bbox( bot, top, nei_size, out_local_maximas, sort_descending_order );
}

template< class DataT >
void ST_HoughSpace<DataT>::get_local_maximas_within_height_range(float zmin, float zmax, int nei_size,
                                                                 std::vector<Vec4i>& out_local_maximas,
                                                                 bool sort_descending_order) const
{
    out_local_maximas.clear();

    // Calcul du niveau z correspondant a minz a partir du sol (bot de l'espace de Hough)
    int minLevZ;
    if( !levZ( minZ() + zmin , minLevZ) )
    {
        PS_LOG->addErrorMessage(LogInterface::error, QString("La grille ne contient pas la hauteur min %1").arg(zmin));
        return;
    }

    // Calcul du niveau z correspondant a maxz
    int maxLevZ;
    if( !levZ( minZ() + zmax , maxLevZ  ) )
    {
        maxLevZ = zdim();
    }

    // On limite la bbox de recherche par ces deux valeurs
    Vec4i bot( 0, 0, 0, minLevZ );
    Vec4i top( wdim(), xdim(), ydim(), maxLevZ );

    get_local_maximas_in_bbox(bot, top, nei_size, out_local_maximas, sort_descending_order);
}

template< class DataT >
void ST_HoughSpace<DataT>::get_local_maximas_in_bbox(const Vec4i& bot, const Vec4i& top, int nei_size,
                                                     std::vector<Vec4i>& out_local_maximas,
                                                     bool sort_descending_order) const
{
    out_local_maximas.clear();

    // On met a jour la bbox pour etre sur de ne pas aller hors de l'espace de Hough (perte de temps dans le parcours car beaucoup de tests potentiels de cellules hors de l'espace)
    Vec4i top_bbox = top;
    Vec4i bot_bbox = bot;
    Vec4i hs_dim   = dim();

    for( int axe = 0 ; axe < 4 ; axe++ )
    {
        if( bot_bbox[axe] < 0 )
        {
            bot_bbox[axe] = 0;
        }

        if( bot_bbox[axe] >= hs_dim[axe] )
        {
            bot_bbox[axe] = hs_dim[axe] - 1;
        }

        if( top_bbox[axe] < 0 )
        {
            top_bbox[axe] = 0;
        }

        if( top_bbox[axe] >= hs_dim[axe] )
        {
            top_bbox[axe] = hs_dim[axe] - 1;
        }
    }

    cv::SparseMatConstIterator_<DataT> pixel_it = _data.begin();
    cv::SparseMatConstIterator_<DataT> pixel_it_end = _data.end();

    for( ; pixel_it != pixel_it_end ; ++pixel_it )
    {
        const cv::SparseMat::Node* curr_pixel_node = pixel_it.node();
        DataT curr_val = pixel_it.template value<DataT>();
        if( curr_val > 0 )
        {
            Vec4i pix( curr_pixel_node->idx[0], curr_pixel_node->idx[1], curr_pixel_node->idx[2], curr_pixel_node->idx[3] );

            if( is_pixel_in_bbox(pix, bot_bbox, top_bbox) )
            {
                if( is_pixel_local_maxima( pix, nei_size ) )
                {
                    out_local_maximas.push_back( pix );
                }
            }
        }
    }

    if( sort_descending_order )
    {
        std::sort(out_local_maximas.begin(), out_local_maximas.end(),
                  [&out_local_maximas, this](const Vec4i& pix1, const Vec4i& pix2) -> bool { return value(pix1[0], pix1[1], pix1[2], pix1[3]) > value(pix2[0], pix2[1], pix2[2], pix2[3]); } );
    }
}

template< class DataT >
bool ST_HoughSpace<DataT>::is_pixel_local_maxima( const Vec4i& pix, int nei_size ) const
{
    DataT curr_value = value( pix );
    Vec4i nei;
    Vec4i dimensions = dim();
    Vec4i size ( nei_size, nei_size, nei_size, nei_size );
    Vec4i bot = pix - size;
    Vec4i top = pix + size;

    // On met a jour la bbox pour etre sur de ne pas aller hors de l'espace de Hough (perte de temps dans le parcours car beaucoup de tests potentiels de cellules hors de l'espace)
    for( int axe = 0 ; axe < 4 ; axe++ )
    {
        if( bot[axe] < 0 )
        {
            bot[axe] = 0;
        }

        if( bot[axe] >= dimensions[axe] )
        {
            bot[axe] = dimensions[axe] - 1;
        }

        if( top[axe] < 0 )
        {
            top[axe] = 0;
        }

        if( top[axe] >= dimensions[axe] )
        {
            top[axe] = dimensions[axe] - 1;
        }
    }

    for ( nei.w() = bot.w() ; nei.w() <= top.w() ; nei.w()++ )
    {
        for ( nei.x() = bot.x(); nei.x() <= top.x() ; nei.x()++ )
        {
            for ( nei.y() = bot.y() ; nei.y() <= top.y() ; nei.y()++ )
            {
                for ( nei.z() = bot.z() ; nei.z() <= top.z(); nei.z()++ )
                {
                    if ( nei != pix )
                    {
                        if ( value(nei) > curr_value )
                        {
                            return false;
                        }
                    }
                }
            }
        }
    }

    return true;
}

#endif // ST_HOUGHSPACE_HPP
