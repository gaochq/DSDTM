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
    Rarsac_base(Frame *_frame, std::vector<cv::Point2f> &_pts1, std::vector<cv::Point2f> &_pts2);
    ~Rarsac_base();

    //! Calculate the F matrix using Rarsac algorithm and delete outliers
    std::vector<bool> RejectFundamental();

    //! Get the bin index of the feature
    int Get_GridIndex(cv::Point2f _pt);

private:

    //! Get features from mvBinFeatures
    bool Get_Features(int Index, std::pair<cv::Point2f, cv::Point2f> _featurePair);

    //! Set the bin of _pt be occupied
    void Set_GridOccupied(cv::Point2f _pt);

    //! Compute the Sampson distance
    double Sampson_Distance(cv::Point2f PointA, cv::Point2f PointB, cv::Mat _F);

    //! Get inliers from the Sampson Distance
    void Get_Inliers(const cv::Mat _F, std::vector<bool> _status);

    //! Compare two mvBinIdexProba from max to min
    static bool CompareBin(const std::pair<int, double> &a, const std::pair<int, double> &b)
    {
        return a.second > b.second;
    }

    void ComputeFundamental();



private:
    int                 mGridSize_Col;
    int                 mGridSize_Row;
    Frame               *mFrame;
    std::vector<bool>   mvGrid_occupancy;
    std::vector<double> mvGrid_probability;
    std::vector<double> mvBin_probability;
    double              mdeta;
    int                 mMaxIteators;
    std::vector<bool>   mvStatus;
    double              mdOutlierThreshold;
    int                 mHalf_GridWidth;
    int                 mHalf_GridHeight;

    std::vector<cv::Point2f> mvCur_pts;         // Features in current image
    std::vector<cv::Point2f> mvPrev_pts;        // Features in previous image
    std::vector< std::vector<int> > mvBinFeatures;

    std::vector< std::pair<int, double> > mvBinIndexProba;      // Bins ranked according to the probability
};
}// namspace DSDTM
#endif //DSDTM_RARSAC_BASE_H
