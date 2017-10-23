//
// Created by buyi on 17-10-16.
//

#ifndef DSDTM_FEATURE_DETECTION_H
#define DSDTM_FEATURE_DETECTION_H

#include "fast/fast.h"

#include "Camera.h"
#include "Feature.h"
#include "Frame.h"

namespace DSDTM
{

class Frame;

struct Corner
{
    int x;
    int y;
    int level;
    float score;
    float angle;
    Corner(int _x, int _y ,float _score, int level, float _angle):
            x(_x), y(_y), score(_score), angle(_angle)
    {}
};
typedef std::vector<Corner> Corners;

class Feature_detector
{
public:
    Feature_detector(const int Image_height, const int Image_width);
    ~Feature_detector();

    //! Get the cell index from feature coordinate
    int Get_CellIndex(int x, int y, int level);

    //! Set the cell be occupied
    void Set_CellIndexOccupy(cv::Point2f &px);

    //! Set the existing features grrid to be occupied
    void Set_ExistingFeatures(const mFeatures& features);

    //! Set the existing features grrid to be occupied
    void Set_ExistingFeatures(const std::vector<cv::Point2f> &features);

    //! Extract fast corners from curren frame
    void detect(Frame* frame, const double detection_threshold);

    //! Reset the states of gridcell
    void ResetGrid();

    //! Get the Shi-Tomasi Score
    float shiTomasiScore(const cv::Mat& img, int u, int v);

public:
    typedef std::shared_ptr<Feature_detector> Feature_detectorPtr;
    const int mImg_height;
    const int mImg_width;

    int mCell_size;
    int mPyr_levels;
    int mGrid_rows;
    int mGrid_cols;
    std::vector<bool> mvGrid_occupy;
};

}// namespace DSDTM



#endif //DSDTM_FEATURE_DETECTION_H
