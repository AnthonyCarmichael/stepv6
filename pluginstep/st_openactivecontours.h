#ifndef ST_OPENACTIVECONTOURS_H
#define ST_OPENACTIVECONTOURS_H

#include "ct_itemdrawable/ct_circle.h"
#include <ct_log/ct_logmanager.h>
#include "st_houghspace.h"

template< class DataT >
class ST_OpenActiveContours
{
    using Vec3i                     = Eigen::Vector3i;
    using Vec3f                     = Eigen::Vector3f;
    using Vec3d                     = Eigen::Vector3d;

    using Vec4i                     = Eigen::Vector4i;
    using Vec4f                     = Eigen::Vector4f;
    using Vec4d                     = Eigen::Vector4d;

    using RepulseImage              = ST_HoughSpace<int>;
    using RepulseImagePtr           = RepulseImage*;
    using RepulseImagePtrConst      = RepulseImagePtr const;
    using ConstRepulseImage         = const RepulseImage;
    using ConstRepulseImagePtr      = ConstRepulseImage*;
    using ConstRepulseImagePtrConst = ConstRepulseImagePtr;

    using HoughSpaceValueType       = DataT;
    using HoughSpace                = ST_HoughSpace<HoughSpaceValueType>;
    using HoughSpacePtr             = HoughSpace*;
    using HoughSpacePtrConst        = HoughSpacePtr const;
    using ConstHoughSpace           = const HoughSpace;
    using ConstHoughSpacePtr        = ConstHoughSpace*;
    using ConstHoughSpacePtrConst   = ConstHoughSpacePtr const;

    using Circle                    = CT_Circle;
    using CirclePtr                 = Circle*;

    using DirScorePair              = std::pair< Vec4d, DataT>;

public:
    ST_OpenActiveContours(ConstHoughSpacePtrConst hough_space,
                          RepulseImagePtrConst repulse_image,
                          const Vec4i& start_pixel,
                          int search_cone_size);

    void get_dir_and_score_inside_bbox(const Vec4i& center_pixel,
                                       const Vec4d& reference_point,
                                       int search_bbox_size_spatial,
                                       int search_bbox_size_radius,
                                       int n_max_dir_to_keep,
                                       DataT min_value_to_consider,
                                       bool sort_crescent_order,
                                       std::vector< DirScorePair >& out_dir_and_scores);

    std::vector< CirclePtr > get_raw_circles() const;

    void append_raw_circles_to_vector( std::vector< CirclePtr >& in_out_circle_vector ) const;

    inline int get_n_points() const { return _points.rows(); }

    void markRepulsion( double repulseFactor );

    void resample( double sampleRes );

    double length3D() const;

    void grow(int nIterMax,
              double growCoeff,
              double coneAngleMaxDegres,
              double coneSizeMaxPixels,
              double seuilSigmaL1);

    void updatePoints();

    void getGrowingDirections(double coneAngleDegres,
                              int coneSizePixels,
                              double seuilSigmaL1,
                              double&outSigmaL1Head,
                              double&outSigmaL1Back,
                              bool& outHasGrownHead,
                              bool& outHasGrownBack,
                              Vec4d& outGrowDirHead,
                              Vec4d& outGrowDirBack ) const;

    void get_dir_and_score_inside_cone(const Vec4i& center_pixel,
                                       const Vec4d& reference_point,
                                       const Vec4d& cone_direction,
                                       double cone_angle_deg,
                                       int search_bbox_size_spatial,
                                       int search_bbox_size_radius,
                                       int n_max_dir_to_keep,
                                       DataT min_value_to_consider,
                                       bool sort_crescent_order,
                                       std::vector< DirScorePair >& out_dir_and_scores) const;

    Vec4d getGradientEnergyGrowProportionalResolution(const Vec4d& snakeExtremityPoint,
                                                      const Vec4i& snakeExtremityPix,
                                                      const Vec4d &snakeTangentAtExtremity,
                                                      double coneAngleDegres,
                                                      int coneSizePixels,
                                                      double& out_sigma_l1,
                                                      bool& outHasGrown ) const;

    void getGeometricMatrix(double alpha, double beta, Eigen::MatrixXd& outGeometricMatrix) const;
    void getAPlusRhoIInverse(double alpha, double beta, double timeStep, Eigen::MatrixXd& outAPlusRhoIInverse ) const;
    void updatePointsAndGetAverageAndStDevMovement(const Eigen::MatrixXd &newPoints,
                                                   double& outAverageMovemet,
                                                   double& outStdDevMovement );

    void relax(int nIterMax,
               double alpha, double beta, double gama,
               double globalWeight,
               double timeStep,
               double threshAverageMovement4D );

    Eigen::MatrixXd* getGradientEnergyOrthogonalImageMatrixMultipliedByTangentNorm( double globalWeight ) const;
    Eigen::MatrixXd* getGradientEnergyOrthogonalImageMatrix( double globalWeight ) const;
    Vec4d gradientEnergyImageInterpolatedAtPoint(int indexPoint, double globalWeight) const;
    double partialDerivativeEnergyImageAtPixelInImage(const Vec4i &p, double globalWeight, int coordToDerive) const;
    double partialDerivativeEnergyImageAtPixelOutsideImage(const Vec4i& p, double globalWeight, int coordToDerive) const;

    inline double partialDerivativeEnergyImageAtPixel(const Vec4i& p, double globalWeight, int coordToDerive) const
    {
        double partialDerivate;
        // Si p est dans l'image, calculer sa derivee partielle est toujours possible :
        // p est dans le centre de l'image,on fait une difference finie centree
        // p est un bord de l'image, on peut faire une difference finie a droite/gauche
        if( _hough_space->isPixelIn(p) )
        {
            partialDerivate = partialDerivativeEnergyImageAtPixelInImage( p, globalWeight, coordToDerive );
        }

        // Si p est en dehors de l'image il faut calculer la derivee partielle autrement
        // Pour le coup je prend le mirroir
        // i.e. si p est a gauche de l'image on prend l'oppose de la derivee partielle en image(1)
        //      si p est a droite de l'image on prend l'oppose de la derivee partielle en image( dim-1 )
        else
        {
            partialDerivate = partialDerivativeEnergyImageAtPixelOutsideImage( p, globalWeight, coordToDerive );
        }

        return partialDerivate;
    }

    double energyImageLocalAtPixel(const Vec4i& p) const;

    inline double energyImageGlobalAtPixel(const Vec4i& p) const
    {
        return static_cast<double>( _hough_space->dataMin() - _hough_space->valueHough(p[0], p[1], p[2], p[3]) ) / static_cast<double>( _hough_space->dataMax() - _hough_space->dataMin() );
    }

    inline double energyImageAtPixel(const Vec4i &p, double globalWeight) const
    {
        return ( globalWeight * energyImageGlobalAtPixel(p) ) + ( ( 1.0 - globalWeight ) * energyImageLocalAtPixel(p) );
    }

    Vec4d directionContoursAtPoint(int indexPoint) const;

    Eigen::MatrixXd* getSecondDifferentialOnTangentDividedByTangentNormMultiplyedByImageEnergy( double globalWeight ) const;

    Vec4d getTangentAtPoint( int i, bool normalize ) const;
    inline Vec4d getTangentAtHead(bool normalize) const { return getTangentAtPoint(0, normalize); }
    inline Vec4d getTangentAtTail(bool normalize) const { return getTangentAtPoint( get_n_points() - 1, normalize ); }

    inline Vec4d getSecondDifferentialAtPoint( int indexPoint ) const
    {
        if( indexPoint == 0 || indexPoint == get_n_points()-1 )
        {
            return Vec4d(0,0,0,0);
        }

        Vec4d prevPoint = _points.row( indexPoint - 1 );
        Vec4d currPoint = _points.row( indexPoint );
        Vec4d nextPoint = _points.row( indexPoint + 1 );

        return ( prevPoint - (2*currPoint) + nextPoint );
    }


private :
    ConstHoughSpacePtrConst     _hough_space;   /*!< Un contours actif se deplace sur une image */
    RepulseImagePtrConst        _repulse_image; /*!< Un contours actif ne peut pas se deplacer vers des pixels qui sont marques comme repulsifs */
    Eigen::MatrixXd             _points;        /*!< Points du contours places dans une matrice n lignes, 4 colonnes */
};

template< class DataT >
void uncenteredPCA(const std::vector< std::pair< Eigen::Vector4d, DataT> >& dir_score_pairs,
                   Eigen::Vector4d& out_v1, Eigen::Vector4d& out_v2, Eigen::Vector4d& out_v3, Eigen::Vector4d& out_v4,
                   double& out_l1, double& out_l2, double& out_l3, double& out_l4);

namespace StepTools
{
    template< typename DataT >
    inline DataT linearInterpolation( DataT xa, DataT xb, DataT ya, DataT yb, DataT valToInterpolate )
    {
        if( xa == xb )
        {
            return ya;
        }

        double a = (ya-yb)/((double)(xa-xb));
        return ( ( a * valToInterpolate ) + ( ya - (a*xa) ) );
    }
}

#include "st_openactivecontours.hpp"

#endif // ST_OPENACTIVECONTOURS_H
