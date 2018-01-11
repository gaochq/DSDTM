//
// Created by buyi on 17-12-15.
//

#ifndef DSDTM_FEATURE_ALIGNMENT_H
#define DSDTM_FEATURE_ALIGNMENT_H

#include "Camera.h"
#include "Keyframe.h"
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

    Feature_Alignment(CameraPtr camera);
    ~Feature_Alignment();

    //! Init the reproject grid
    void GridInitalization();

    //! Reset Grid
    void ResetGrid();

    //! Add the best feature into cell
    bool ReprojectCell(FramePtr tFrame, Cell *tCell);

    //! project mappoint into Grid
    bool ReprojectPoint(FramePtr tFrame, MapPoint *tMPoint);

    //! Find mappoint match for current frame through feature alignment
    void SearchLocalPoints(FramePtr tFrame);

    //! Compare cells refer to observe times
    static bool CellComparator(Candidate &c1, Candidate &c2);

    //! Find the match of MapPoint using Featurealignment
    bool FindMatchDirect(const MapPoint *tMpPoint, const FramePtr tFrame, Eigen::Vector2d &tPt, int &tLevel);

    //! Get the affine matrix
    Eigen::Matrix2d SolveAffineMatrix(KeyFrame *tReferKframe, const FramePtr tCurFrame, Feature *tReferFeature, const MapPoint *tMpPoint);

    //! Get the best search level refer to the determinate of affine matrix
    int GetBestSearchLevel(Eigen::Matrix2d tAffineMat, int tMaxLevel);

    //! Affine transformation to get the reference patch
    void WarpAffine(const Eigen::Matrix2d tA_c2r, const cv::Mat &tImg_ref, Feature *tRefFeature, const int tSearchLevel, uchar *tPatchLarger);

    //! Get the final patch form patch with boarder
    void GetPatchNoBoarder();

    //! Use inverse compositional align method
    static bool Align2DCeres(const cv::Mat &tCurImg, uchar *tPatch_WithBoarder, uchar *tPatch,  int MaxIters, Eigen::Vector2d &tCurPx);
    static bool Align2DGaussNewton(const cv::Mat &tCurImg, uchar *tPatch_WithBoarder, uchar *tPatch,  int MaxIters, Eigen::Vector2d &tCurPx);


private:

    CameraPtr  mCam;

    Grid    mGrid;
    int     mMax_pts;
    int     mPyr_levels;


    uchar   mPatch[2*mHalf_PatchSize*2*mHalf_PatchSize];
    uchar   mPatch_WithBoarder[(2*mHalf_PatchSize + 2)*(2*mHalf_PatchSize + 2)];

};


class FeatureAlign2DProblem:public ceres::SizedCostFunction<4*mHalf_PatchSize*mHalf_PatchSize, 3>
{
public:
    FeatureAlign2DProblem(uchar *tRefPatch, const cv::Mat *tCurImg, double *tJacobianPatch):
            mRefPatch(tRefPatch), mCurImg(tCurImg), mJacobianPatch(tJacobianPatch),
            mPatchArea(4*mHalf_PatchSize*mHalf_PatchSize)
    {

    }

    virtual bool Evaluate(double const* const* parameters, double *residuals, double **jacobians) const
    {
        double u = parameters[0][0];
        double v = parameters[0][1];
        double tMeandiff = parameters[0][2];

        int u_i = floor(u);
        int v_i = floor(v);

        if(u_i < mHalf_PatchSize || v_i < mHalf_PatchSize || u_i > mCurImg->cols - mHalf_PatchSize ||
                v_i > mCurImg->rows - mHalf_PatchSize)
        {
            Eigen::Map<Eigen::Matrix<double, 4*mHalf_PatchSize*mHalf_PatchSize, 1>> mResiduals(residuals);

            mResiduals.setZero();

            if (jacobians != NULL && jacobians[0] != NULL)
            {
                Eigen::Map<Eigen::Matrix<double, 4*mHalf_PatchSize*mHalf_PatchSize, 3, Eigen::RowMajor>> mJacobians(jacobians[0]);
                mJacobians.setZero();
            }

            return true;
        }

        const double subpix_x = u - u_i;
        const double subpix_y = v - v_i;
        const double tC_tl = (1.0 - subpix_x)*(1.0 - subpix_y);
        const double tC_tr = subpix_x*(1.0 - subpix_y);
        const double tC_bl = (1.0 - subpix_x)*subpix_y;
        const double tC_br = subpix_x*subpix_y;

        int mStep = mCurImg->step.p[0];
        int tNum = 0;
        for (int i = 0; i < 2*mHalf_PatchSize; ++i)
        {
            uchar *it = (uchar*)mCurImg->data + (v_i + i - mHalf_PatchSize)*mStep + u_i - mHalf_PatchSize;
            for (int j = 0; j < 2*mHalf_PatchSize; ++j, ++it, ++tNum)
            {
                double tSearchPx = tC_tl*it[0] + tC_tr*it[1] + tC_bl*it[mStep] + tC_br*it[mStep+1];
                residuals[tNum] = tSearchPx - *(mRefPatch+tNum) + tMeandiff;
            }
        }

        if (jacobians != NULL && jacobians[0] != NULL)
        {
            Eigen::Map<Eigen::Matrix<double, 4*mHalf_PatchSize*mHalf_PatchSize, 3, Eigen::RowMajor>> mJacobians(jacobians[0]);

            Eigen::Map<Eigen::Matrix<double, 4*mHalf_PatchSize*mHalf_PatchSize, 3, Eigen::RowMajor>> tJacobians(mJacobianPatch);
            mJacobians = tJacobians;
        }

        return true;
    }


protected:

    const int   mPatchArea;

    uchar       *mRefPatch;
    double      *mJacobianPatch;

    const cv::Mat     *mCurImg;

};


}// namespace DSDTM







#endif //DSDTM_FEATURE_ALIGNMENT_H
