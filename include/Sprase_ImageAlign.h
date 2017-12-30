//
// Created by buyi on 17-12-21.
//

#ifndef DSDTM_SPRASE_IMAGEALIGN_H
#define DSDTM_SPRASE_IMAGEALIGN_H

#include "Camera.h"
#include "Frame.h"
#include "Feature_alignment.h"
#include "ceres/ceres.h"
#include "Optimizer.h"


namespace DSDTM
{

class Frame;

class Sprase_ImgAlign
{
public:
    Sprase_ImgAlign(int tMaxLevel, int tMinLevel, int tMaxIterators);
    ~Sprase_ImgAlign();

    //! Reset
    void Reset();

    int Run(FramePtr tCurFrame, FramePtr tRefFrame);

    //! Compute the jacobian matrix and  reference patch
    void GetJocabianMat(int tLevel);

    //! Compute the jacobian about pixel position to camera transform
    Eigen::Matrix<double, 2,6> GetJocabianBA(Eigen::Vector3d tPoint);

    //! Solve transform with ceres
    void DirectSolver(Sophus::SE3 &tT_c2r, int tLevel);

protected:
    int     mnMaxLevel;
    int     mnMinLevel;
    int     mnMaxIterators;
    int     mnMinfts;


    const int       mBoarder;
    const int       mPatchArea;

    FramePtr    mCurFrame;
    FramePtr    mRefFrame;

    Sophus::SE3     mT_c2r;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> mRefPatch;
    Eigen::Matrix<double, Eigen::Dynamic, 6, Eigen::RowMajor> mJocabianPatch;
    Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::ColMajor> mRefNormals;

    std::vector<bool> mVisible;
};

class DirectSE3_Problem: public ceres::SizedCostFunction<mHalf_PatchSize*mHalf_PatchSize, 6>
{
public:
    DirectSE3_Problem(Eigen::Vector3d tPoint, double *tRefPatch, double *tJocabianPatch,
                      const cv::Mat *tCurImg, const float tScale, Camera *tCamera):
            mMapPoint(tPoint), mRefPatch(tRefPatch), mJacobianPatch(tJocabianPatch),
            mCurImg(tCurImg), mScale(tScale), mCamera(tCamera), mnboarder(mHalf_PatchSize-1)
    {
    }

    virtual bool Evaluate(double const* const* parameters, double *residuals, double **jacobians) const
    {
        // parameter: translation--rotation
        Eigen::Matrix<double, 6, 1> tT_c2rArray;
        tT_c2rArray<< parameters[0][0], parameters[0][1], parameters[0][2],
                      parameters[0][3], parameters[0][4], parameters[0][5];

        Sophus::SE3 tT_c2r(Sophus::SO3::exp(tT_c2rArray.tail<3>()), tT_c2rArray.head<3>());

        Eigen::Vector3d tCurPoint = tT_c2r*mMapPoint;
        Eigen::Vector2d tCurPix = mCamera->Camera2Pixel(tCurPoint)*mScale;

        const double u = tCurPix(0);
        const double v = tCurPix(1);
        const int u_i = floor(u);
        const int v_i = floor(v);
        if(u_i < 0 || v_i < 0 || u_i - mnboarder < 0 || v_i - mnboarder < 0 || u_i + mnboarder >= mCurImg->cols || v_i + mnboarder >= mCurImg->rows)
        {
            Eigen::Map<Eigen::Matrix<double, mHalf_PatchSize*mHalf_PatchSize, 1>> mResiduals(residuals);
            mResiduals.setZero();

            if(jacobians != NULL && jacobians[0] != NULL)
            {
                Eigen::Map<Eigen::Matrix<double, mHalf_PatchSize * mHalf_PatchSize, 6, Eigen::RowMajor>> mJacobians(jacobians[0]);
                mJacobians.setZero();
            }

            return true;
        }

        const double tSubPixU = u - u_i;
        const double tSubPixV = v - v_i;
        const double tC_tl = (1.0 - tSubPixU)*(1.0 - tSubPixV);
        const double tC_tr = tSubPixU*(1.0 - tSubPixV);
        const double tC_bl = (1.0 - tSubPixU)*tSubPixV;
        const double tC_br = tSubPixU*tSubPixV;

        int tStep = mCurImg->step.p[0];
        int tNum = 0;

        for (int i = 0; i < mHalf_PatchSize; ++i)
        {
            uchar *it = mCurImg->data + (v_i + i - 2)*mCurImg->cols + u_i - 2;
            for (int j = 0; j < mHalf_PatchSize; ++j, ++it, ++tNum)
            {
                double tCurPx = tC_tl*it[0] + tC_tr*it[1] + tC_bl*it[tStep] + tC_br*it[tStep+1];
                residuals[tNum] = *(mRefPatch+tNum) - tCurPx;
            }
        }

        if(jacobians!=NULL)
        {
            if (jacobians[0] != NULL)
            {
                Eigen::Map<Eigen::Matrix<double, mHalf_PatchSize * mHalf_PatchSize, 6, Eigen::RowMajor>> mJacobians(jacobians[0]);
                Eigen::Matrix<double, mHalf_PatchSize * mHalf_PatchSize, 6, Eigen::RowMajor> tJacobians(mJacobianPatch);

                mJacobians = tJacobians;
            }
        }

        return true;
    }


protected:
    Eigen::Vector3d     mMapPoint;

    Camera              *mCamera;
    const cv::Mat       *mCurImg;
    const float         mScale;

    double              *mRefPatch;
    double              *mJacobianPatch;

    const int           mnboarder;
};



}// namespace DSDTM

#endif //DSDTM_SPRASE_IMAGEALIGN_H
