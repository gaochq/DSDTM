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
    Corner(int _x, int _y ,float _score, int _level, float _angle):
            x(_x), y(_y), score(_score), level(_level), angle(_angle)
    {}
    bool operator < (const Corner& _c1) const
    {
        return (_c1.score < score);
    }
};
typedef std::vector<Corner> Corners;

class Feature_detector
{
public:
    Feature_detector();
    ~Feature_detector();

    //! Get the cell index from feature coordinate
    int Get_CellIndex(int x, int y, int level);

    //! Set the cell be occupied
    void Set_CellIndexOccupy(const cv::Point2f px);

    //! Set the existing features grrid to be occupied
    void Set_ExistingFeatures(const Features& features);

    //! Set the existing features grrid to be occupied
    void Set_ExistingFeatures(const std::vector<cv::Point2f>& features);

    //! Extract fast corners from curren frame
    void detect(Frame* frame, const double detection_threshold, const bool tFirst=true);

    //! Reset the states of gridcell
    void ResetGrid();

    //! Get the Shi-Tomasi Score
    float shiTomasiScore(const cv::Mat& img, int u, int v);

public:
    typedef std::shared_ptr<Feature_detector> Feature_detectorPtr;
    int mImg_height;
    int mImg_width;

    int mCell_size;
    int mPyr_levels;
    int mGrid_rows;
    int mGrid_cols;
    int mMax_fts;
    std::vector<bool> mvGrid_occupy;
};

}// namespace DSDTM



#endif //DSDTM_FEATURE_DETECTION_H
