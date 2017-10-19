//
// Created by buyi on 17-10-19.
//

#ifndef DSDTM_RARSAC_BASE_H
#define DSDTM_RARSAC_BASE_H

#include "Camera.h"
#include "Frame.h"

namespace DSDTM
{

class Rarsac_base
{
public:
    Rarsac_base(Frame *_frame);
    ~Rarsac_base();

    //! Calculate the F matrix using Rarsac algorithm and delete outliers
    void Compute_Fmat(std::vector<cv::Point2f> Cur_pts, std::vector<cv::Point2f> Prev_pts);

    //! Get the bin index of the feature
    static int Get_GridIndex(cv::Point2f _pt);

private:

    //! Set the bin of _pt be occupied
    void Set_GridOccupied(cv::Point2f _pt);

    //! Compute the Sampson distance
    double Sampson_Distance(cv::Point2f PointA, cv::Point2f PointB, Eigen::Matrix3d F);




private:
    static int          mGridSize_Col;
    static int          mGridSize_Row;
    Frame               *mFrame;
    std::vector<bool>   mvGrid_occupancy;
    std::vector<float>  mvGrid_probability;
    double              mdeta;
    int                 mMaxIteators;
};
}// namspace DSDTM
#endif //DSDTM_RARSAC_BASE_H
