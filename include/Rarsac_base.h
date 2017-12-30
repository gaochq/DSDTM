//
// Created by buyi on 17-10-19.
//

#ifndef DSDTM_RARSAC_BASE_H
#define DSDTM_RARSAC_BASE_H

#include "Camera.h"
#include "Frame.h"

namespace DSDTM
{
class Frame;


class Rarsac_base
{
public:
    Rarsac_base();
    Rarsac_base(Frame *_frame, std::vector<cv::Point2f> &_pts1, std::vector<cv::Point2f> &_pts2);
    ~Rarsac_base();

    //! Calculate the F matrix using Rarsac algorithm and delete outliers
    std::vector<uchar> RejectFundamental();

    //! Get the bin index of the feature
    int Get_GridIndex(cv::Point2f _pt);

protected:

    //! Get features from mvBinFeatures
    bool Get_Features(int Index, std::pair<cv::Point2f, cv::Point2f> &_featurePair);

    //! Set the bin of _pt be occupied
    void Set_GridOccupied(cv::Point2f _pt);

    //! Compute the Sampson distance
    double Sampson_Distance(cv::Point2f PointA, cv::Point2f PointB, cv::Mat _F);

    //! Get inliers from the Sampson Distance
    void Get_Inliers(const cv::Mat _F, std::vector<uchar> &_status);

    //! Compare two mvBinIdexProba from max to min
    static bool CompareBin(const std::pair<int, double> &a, const std::pair<int, double> &b)
    {
        return a.second > b.second;
    }

    //! Get inliners refer to Fundamental matrix
    float CheckFundamental(const cv::Mat &F21, std::vector<uchar> &_status, float sigma);

    //! Normalize corners to calculate faundamental matrix
    cv::Mat Normalize_Points(std::vector <cv::Point2f> Points, std::vector <cv::Point2f> &Norm_Points);

    //! Solve fundamental matrix
    cv::Mat ComputeF21(const std::vector<cv::Point2f> &vP1, const std::vector<cv::Point2f> &vP2);


protected:
    int                     mGridSize_Col;
    int                     mGridSize_Row;
    Frame                   *mFrame;

    int                     mMaxIteators;
    double                  mdOutlierThreshold;
    double                  mdTerminThreshold;

    std::vector<uchar>       mvStatus;
    int                     mHalf_GridWidth;
    int                     mHalf_GridHeight;
    std::vector<bool>       mvGrid_occupancy;
    std::vector<double>     mvGrid_probability;
    std::vector<double>     mvBin_probability;

    std::vector<cv::Point2f>            mvCur_pts;         // Features in current image
    std::vector<cv::Point2f>            mvPrev_pts;        // Features in previous image
    std::vector< std::vector<int> >     mvBinFeatures;

    std::vector< std::pair<int, double> > mvBinIndexProba;      // Bins ranked according to the probability

    std::vector<Eigen::Vector2d> mvGridBinPose;      // center position of Bins
};
}// namspace DSDTM
#endif //DSDTM_RARSAC_BASE_H
