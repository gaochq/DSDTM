//
// Created by buyi on 17-12-15.
//

#ifndef DSDTM_FEATURE_ALIGNMENT_H
#define DSDTM_FEATURE_ALIGNMENT_H

#include "Camera.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"


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

    //! Get the final patch form patch with boarder
    void GetPatchNoBoarder();

    //! Use inverse compositional align method
    static bool Align2D(const cv::Mat &tCurImg, uchar *tPatch_WithBoarder, uchar *tPatch,  int MaxIters, Eigen::Vector2d &tCurPx);


private:

    Camera  *mCam;

    Grid    mGrid;
    int     mMax_pts;
    int     mPyr_levels;


    uchar   mPatch[2*mHalf_PatchSize*2*mHalf_PatchSize];
    uchar   mPatch_WithBoarder[(2*mHalf_PatchSize + 2)*(2*mHalf_PatchSize + 2)];

};

/*
class FeatureAlign2D:public ceres::SizedCostFunction<1, 3>
{

public:
    FeatureAlign2D(const int index, const uchar *tRefPatch, const uchar *tRefdx, const uchar *tRefdy, const cv::Mat tCurImg):
         tIndex(index), itRef(tRefPatch), itRefdx(tRefdx), itRefdy(tRefdy), mCurImg(tCurImg)
    {}

    virtual bool Evaluate(double const* const* parameters, double *residuals, double **jacobians) const
    {
        int y = tIndex/8;
        int x = tIndex%8;

        double u = parameters[0][0];
        double v = parameters[0][1];
        double mean_diff = parameters[0][2];

        int u_r = floor(u);
        int v_r = floor(v);
        if(u_r < mHalf_PatchSize || v_r < mHalf_PatchSize || u_r > mCurImg.cols - mHalf_PatchSize ||
           v_r > mCurImg.rows - mHalf_PatchSize || isnan(u) || isnan(v))
            return false;

        float subpix_x = u - u_r;
        float subpix_y = v - v_r;
        float wTL = (1.0 - subpix_x)*(1.0 - subpix_y);
        float wTR = subpix_x*(1 - subpix_y);
        float wBL = (1.0 - subpix_x)*subpix_y;
        float wBR = subpix_x*subpix_y;


    }

protected:
    int tIndex;

    uchar *itRef;
    uchar *itRefdx;
    uchar *itRefdy;
    cv::Mat mCurImg;

};
*/

}// namespace DSDTM







#endif //DSDTM_FEATURE_ALIGNMENT_H
