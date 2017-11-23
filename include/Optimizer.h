//
// Created by buyi on 17-11-9.
//

#ifndef DSDTM_OPTIMIZER_H
#define DSDTM_OPTIMIZER_H

#include "Camera.h"
#include "Frame.h"

#include "ceres/ceres.h"
#include "ceres/rotation.h"

namespace DSDTM
{

class Frame;
class PoseGraph_Problem;

class Optimizer
{
public:
    Optimizer();
    ~Optimizer();

    static void PoseOptimization(Frame &tCurFrame, int tIterations = 10);
    static double *se3ToDouble(Eigen::Matrix<double, 6, 1> tso3);
    static std::vector<double> GetReprojectReidual(const ceres::Problem &problem);

};


//! 参数块的个数和后面的雅克比矩阵的维数要对应
//! The number of parameter blocks should be equal to jacobian
class PoseSolver_Problem: public ceres::SizedCostFunction<2, 6>
{
public:
    PoseSolver_Problem(const Eigen::Vector3d &tMapPoint, const Eigen::Vector2d &tObservation,
                      const Eigen::Matrix4d &tIntrinsic):
            mMapPoint(tMapPoint), mObservation(tObservation), mIntrinsic(tIntrinsic)
    {
        mfx = mIntrinsic(0, 0);
        mfy = mIntrinsic(1, 1);
        mcx = mIntrinsic(0, 2);
        mcy = mIntrinsic(1, 2);
    }

    virtual bool Evaluate(double const* const* parameters, double* residuals, double **jacobians) const
    {
        //! Convert se3 into SE3
        Eigen::Matrix<double, 6, 1> tTransform;
        tTransform << parameters[0][0], parameters[0][1], parameters[0][2],
                      parameters[0][3], parameters[0][4], parameters[0][5];
        Sophus::SE3 mPose = Sophus::SE3::exp(tTransform);

        //! column major, in camera coordinate
        Eigen::Map<Eigen::Vector2d> mResidual(residuals);
        Eigen::Vector3d tCamPoint = mPose*mMapPoint;

        //! in normalize plane, distortion correction
        double tx = tCamPoint(0)/tCamPoint(2);
        double ty = tCamPoint(1)/tCamPoint(2);
        double tk1 = mIntrinsic(0, 3);
        double tk2 = mIntrinsic(1, 3);

        double tr2 = tx*tx + ty*ty;
        double tDistortion = 1.0 + tr2*(tk1 + tk2*tr2);

        Eigen::Vector2d tPixel;
        tPixel << mfx*tx*tDistortion + mcx,
                mfy*ty*tDistortion + mcy;

        mResidual = mObservation - tPixel;

        if(mResidual(0)>10 || mResidual(1)>10)
            std::cout<< "error" <<std::endl;

        //std::cout<< "num" <<std::endl;
        //std::cout << mResidual <<std::endl<<std::endl;

        if(jacobians!=NULL)
        {
            double x = tCamPoint(0);
            double y = tCamPoint(1);
            double z_inv = 1.0/tCamPoint(2);
            double z_invsquare = z_inv*z_inv;

            //! There is only one parametre block, so is the jacobian matrix
            if(jacobians[0]!=NULL)
            {
                Eigen::Map< Eigen::Matrix<double, 2, 6, Eigen::RowMajor> > mJacobians1(jacobians[0]);

                mJacobians1(0, 0) = mfx*z_inv;
                mJacobians1(0, 1) = 0;
                mJacobians1(0, 2) = -mfx*x*z_invsquare;
                mJacobians1(0, 3) = -mfx*x*y*z_invsquare;
                mJacobians1(0, 4) = mfx + mfx*x*x/z_invsquare;
                mJacobians1(0, 5) = -mfx*y/z_inv;

                mJacobians1(1, 0) = 0;
                mJacobians1(1, 1) = mfy*z_inv;
                mJacobians1(1, 2) = -mfy*y*z_invsquare;
                mJacobians1(1, 3) = -mfy - mfy*y*y/z_invsquare;
                mJacobians1(1, 4) = mfy*x*y*z_invsquare;
                mJacobians1(1, 5) = mfy*x/z_inv;

                mJacobians1 = -1.0*mJacobians1;
            }
        }

        return true;
    }

protected:
    Eigen::Vector3d     mMapPoint;
    Eigen::Vector2d     mObservation;
    Eigen::Matrix3d     mSqrt_Info;
    Eigen::Matrix<double, 4, 4, Eigen::RowMajor>     mIntrinsic;


    double              mfx;
    double              mfy;
    double              mcx;
    double              mcy;
};

//! 参数块的个数和后面的雅克比矩阵的维数要对应
class TwoViewBA_Problem: public ceres::SizedCostFunction<2, 6, 3>
{
public:
    TwoViewBA_Problem(const Eigen::Vector2d &tObservation, const Eigen::Matrix4d &tIntrinsic):
            mObservation(tObservation), mIntrinsic(tIntrinsic)
    {
        mfx = mIntrinsic(0, 0);
        mfy = mIntrinsic(1, 1);
        mcx = mIntrinsic(0, 2);
        mcy = mIntrinsic(1, 2);
    }

    virtual bool Evaluate(double const* const* parameters, double* residuals, double **jacobians) const
    {
        //! Camera pose: Convert se3 into SE3
        Eigen::Matrix<double, 6, 1> tTransform;
        tTransform << parameters[0][0], parameters[0][1], parameters[0][2],
                parameters[0][3], parameters[0][4], parameters[0][5];
        Sophus::SE3 mPose = Sophus::SE3::exp(tTransform);

        //! MapPoint
        Eigen::Vector3d mMapPoint;
        mMapPoint << parameters[1][0], parameters[1][1], parameters[1][2];

        //! column major, in camera coordinate
        Eigen::Map<Eigen::Vector2d> mResidual(residuals);
        Eigen::Vector3d tCamPoint = mPose*mMapPoint;

        //! in normalize plane, distortion correction
        double tx = tCamPoint(0)/tCamPoint(2);
        double ty = tCamPoint(1)/tCamPoint(2);
        double tk1 = mIntrinsic(0, 3);
        double tk2 = mIntrinsic(1, 3);

        double tr2 = tx*tx + ty*ty;
        //double tDistortion = 1.0 + tr2*(tk1 + tk2*tr2);
        double tDistortion = 1.0;

        Eigen::Vector2d tPixel;
        tPixel << mfx*tx*tDistortion + mcx,
                mfy*ty*tDistortion + mcy;

        mResidual = mObservation - tPixel;

        if(mResidual(0)>10 || mResidual(1)>10)
            std::cout<< "error" <<std::endl;

        //std::cout<< "num" <<std::endl;
        //std::cout << mResidual <<std::endl<<std::endl;

        if(jacobians!=NULL)
        {
            double x = tCamPoint(0);
            double y = tCamPoint(1);
            double z_inv = 1.0/tCamPoint(2);
            double z_invsquare = z_inv*z_inv;

            if(jacobians[0]!=NULL)
            {
                Eigen::Map< Eigen::Matrix<double, 2, 6, Eigen::RowMajor> > mJacobians1(jacobians[0]);

                mJacobians1(0, 0) = mfx*z_inv;
                mJacobians1(0, 1) = 0;
                mJacobians1(0, 2) = -mfx*x*z_invsquare;
                mJacobians1(0, 3) = -mfx*x*y*z_invsquare;
                mJacobians1(0, 4) = mfx + mfx*x*x/z_invsquare;
                mJacobians1(0, 5) = -mfx*y/z_inv;

                mJacobians1(1, 0) = 0;
                mJacobians1(1, 1) = mfy*z_inv;
                mJacobians1(1, 2) = -mfy*y*z_invsquare;
                mJacobians1(1, 3) = -mfy - mfy*y*y/z_invsquare;
                mJacobians1(1, 4) = mfy*x*y*z_invsquare;
                mJacobians1(1, 5) = mfy*x/z_inv;

                mJacobians1 = -1.0*mJacobians1;
            }

            if(jacobians!=NULL && jacobians[1]!=NULL)
            {
                Eigen::Map< Eigen::Matrix<double, 2, 3, Eigen::RowMajor> > mJacobians2(jacobians[1]);

                Eigen::Matrix<double, 2, 3, Eigen::RowMajor> mJacob_tmp;
                mJacob_tmp << mfx*z_inv, 0, -mfx*x*z_invsquare,
                            0, mfy*z_inv, -mfy*y*z_invsquare;

                mJacobians2 = -mJacob_tmp*mPose.so3().matrix();
            }

        }

        return true;
    }

protected:
    Eigen::Vector2d     mObservation;
    Eigen::Matrix3d     mSqrt_Info;
    Eigen::Matrix<double, 4, 4, Eigen::RowMajor>     mIntrinsic;


    double              mfx;
    double              mfy;
    double              mcx;
    double              mcy;
};


} //namespace DSDTM


#endif //DSDTM_OPTIMIZER_H
