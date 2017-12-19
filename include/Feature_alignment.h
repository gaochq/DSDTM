//
// Created by buyi on 17-12-15.
//

#ifndef DSDTM_FEATURE_ALIGNMENT_H
#define DSDTM_FEATURE_ALIGNMENT_H

#include "Camera.h"



namespace DSDTM
{
class Camera;
class MapPoint;
class Frame;
class Feature;

static const int mHalf_PatchSize = 4;

class Feature_Alignment
{
public:
    struct Candidate
    {
        MapPoint            *mMpPoint;
        Eigen::Vector2d     mPx;
        Candidate(MapPoint *pt, Eigen::Vector2d px):
                mMpPoint(pt), mPx(px)
        {}
    };
    typedef std::list<Candidate>    Cell;
    typedef std::vector<Cell*>      CandidateGrid;
    struct Grid
    {
        CandidateGrid       mCells;
        std::vector<int>    mCellOrder;
        int mCell_size;
        int mGrid_Cols;
        int mGrid_Rows;
    };

public:

    Feature_Alignment(Camera *camera);
    ~Feature_Alignment();

    //! Init the reproject grid
    void GridInitalization();

    //! Reset Grid
    void ResetGrid();

    //! Add the best feature into cell
    bool ReprojectCell(Frame tFrame, Cell *tCell);

    //! project mappoint into Grid
    bool ReprojectPoint(Frame tFrame, MapPoint *tMPoint);

    //! Find mappoint match for current frame through feature alignment
    void SearchLocalPoints(Frame tFrame);

    //! Compare cells refer to observe times
    static bool CellComparator(Candidate &c1, Candidate &c2);

    //! Find the match of MapPoint using Featureliment
    bool FindMatchDirect(MapPoint *tMpPoint, const Frame tFrame, Eigen::Vector2d &tPt);

    //! Get the affine matrix
    Eigen::Matrix2d SolveAffineMatrix(KeyFrame *tReferKframe, const Frame &tCurFrame, Feature tReferFeature, MapPoint *tMpPoint);

    //! Get the best search level refer to the determinate of affine matrix
    int GetBestSearchLevel(Eigen::Matrix2d tAffineMat, int tMaxLevel);

    //! Affine transformation to get the reference patch
    void WarpAffine(const Eigen::Matrix2d tA_c2r, const cv::Mat &tImg_ref, Feature tRefFeature, const int tSearchLevel, uchar *tPatchLarger);


private:

    Camera  *mCam;

    Grid    mGrid;
    int     mMax_pts;
    int     mPyr_levels;


    uchar   mPatch[2*mHalf_PatchSize*2*mHalf_PatchSize];
    uchar   mPatch_WithBoarder[(2*mHalf_PatchSize + 2)*(2*mHalf_PatchSize + 2)];

};


}// namespace DSDTM







#endif //DSDTM_FEATURE_ALIGNMENT_H
