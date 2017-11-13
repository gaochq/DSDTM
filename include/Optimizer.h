//
// Created by buyi on 17-11-9.
//

#ifndef DSDTM_OPTIMIZER_H
#define DSDTM_OPTIMIZER_H

#include "Camera.h"
#include "Frame.h"
#include "ceres/ceres.h"

namespace DSDTM
{

class Frame;

class Optimizer
{
public:
    Optimizer();
    ~Optimizer();

    static void PoseSolver(Frame &tRefFrame, Frame &tCurFrame);
};


class PoseGraph_Problem: public ceres::SizedCostFunction<2, 7>
{
public:
    PoseGraph_Problem(const Eigen::Vector3d tMapPoint, const Eigen::Vector2d tObservation,
                      const Eigen::Matrix3d tIntrinsic):
            mMapPoint(tMapPoint), mObservation(tObservation), mIntrinsic(tIntrinsic)
    {

    }

    virtual bool Evaluate(double const *parameters, double residuals, double **jacobians) const
    {
        Eigen::Vector3d mTranslation(parameters[0], parameters[1], parameters[2]);
        Eigen::Quaternion mRotation(parameters[3], parameters[4], parameters[5], parameters[6]);

        //! column major
        Eigen::Map<Eigen::Vector2d> mResidual(residuals);
        Eigen::Vector3d tCamPoint = mRotation*mMapPoint + mTranslation;
        mResidual = mObservation - 1.0/tCamPoint(2)*mIntrinsic.block(0, 0, 1, 1)*tCamPoint.block(0, 0, 1, 0);

        if(jacobians)
        {
            double x = mMapPoint(0);
            double y = mMapPoint(1);
            double z_inv = 1.0/mMapPoint(2);
            double z_invsquare = z_inv*z_inv;

            if(jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> mJacobians1(jacobians[0]);

                mJacobians1(0, 0) = fx*z_inv;
                mJacobians1(0, 1) = 0;
                mJacobians1(0, 2) = -fx*x*z_invsquare;
                mJacobians1(0, 3) = -fx*x*y*z_invsquare;
                mJacobians1(0, 4) = fx + fx*x*x/z_invsquare;
                mJacobians1(0, 5) = -fx*y/z_inv;

                mJacobians1 = -1.0*mJacobians1;
            }

            if(jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> mJacobians2(jacobians[1]);

                mJacobians1(1, 0) = 0;
                mJacobians1(1, 1) = fy*z_inv;
                mJacobians1(1, 2) = -fy*y*z_invsquare;
                mJacobians1(1, 3) = -fy - fy*y*y/z_invsquare;
                mJacobians1(1, 4) = fy*x*y*z_invsquare;
                mJacobians1(1, 5) = fy*y/z_inv;

                mJacobians2 = -1.0*mJacobians2;
            }
        }

        return true;
    }


protected:
    Eigen::Vector3d     mMapPoint;
    Eigen::Vector2d     mObservation;
    Eigen::Matrix3d     mSqrt_Info;
    Eigen::Matrix3d     mIntrinsic;

    double              mCam_fx;
    double              mCam_fy;
    double              mCam_cx;
    double              mCam_cy;


};

} //namespace DSDTM


#endif //DSDTM_OPTIMIZER_H
