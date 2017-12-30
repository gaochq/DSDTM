//
// Created by buyi on 17-11-9.
//

#ifndef DSDTM_OPTIMIZER_H
#define DSDTM_OPTIMIZER_H

#include "Camera.h"
#include "Frame.h"

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <sophus/se3.h>


namespace DSDTM
{

class Frame;
class PoseGraph_Problem;

class Optimizer
{
public:
    Optimizer();
    ~Optimizer();

    static void PoseOptimization(Frame &tCurFrame, int tIterations = 100);
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
        //! Convert se3 into SO3
        Eigen::Matrix<double, 6, 1> tTransform;
        tTransform << parameters[0][0], parameters[0][1], parameters[0][2],
                      parameters[0][3], parameters[0][4], parameters[0][5];
        Sophus::SE3 mPose = Sophus::SE3::exp(tTransform);

        //! column major, in camera coordinate
        Eigen::Map<Eigen::Vector2d> mResidual(residuals);
        Eigen::Vector3d tCamPoint = mPose*mMapPoint;

        //! Don't need undistortion
        Eigen::Vector2d tPixel;
        tPixel << mfx*tCamPoint(0)/tCamPoint(2) + mcx,
                mfy*tCamPoint(1)/tCamPoint(2) + mcy;

        mResidual = mObservation - tPixel;

        if(mResidual(0)>10 || mResidual(1)>10)
            std::cout<< "error" <<std::endl;

        //std::cout<< "num" <<std::endl;
        //std::cout << mResidual <<std::endl<<std::endl;
        Eigen::Matrix<double, 2, 3, Eigen::RowMajor> J_cam;


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

                mJacobians1(0, 0) = -mfx*x*y*z_invsquare;
                mJacobians1(0, 1) = mfx + mfx*x*x/z_invsquare;
                mJacobians1(0, 2) = -mfx*y/z_inv;
                mJacobians1(0, 3) = mfx*z_inv;
                mJacobians1(0, 4) = 0;
                mJacobians1(0, 5) = -mfx*x*z_invsquare;

                mJacobians1(1, 0) = -mfy - mfy*y*y/z_invsquare;
                mJacobians1(1, 1) = mfy*x*y*z_invsquare;
                mJacobians1(1, 2) = mfy*x/z_inv;
                mJacobians1(1, 3) = 0;
                mJacobians1(1, 4) = mfy*z_inv;
                mJacobians1(1, 5) = -mfy*y*z_invsquare;

                mJacobians1 = -1.0*mJacobians1;
            }

            if(jacobians[1]!=NULL)
            {
                std::cout << "11" << std::endl;
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
        //! Camera pose: Convert se3 into SO3
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

        Eigen::Vector2d tPixel;
        tPixel << mfx*tCamPoint(0)/tCamPoint(2) + mcx,
                mfy*tCamPoint(1)/tCamPoint(2) + mcy;

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

                mJacobians1(0, 0) = -mfx*x*y*z_invsquare;
                mJacobians1(0, 1) = mfx + mfx*x*x/z_invsquare;
                mJacobians1(0, 2) = -mfx*y/z_inv;
                mJacobians1(0, 3) = mfx*z_inv;
                mJacobians1(0, 4) = 0;
                mJacobians1(0, 5) = -mfx*x*z_invsquare;

                mJacobians1(1, 0) = -mfy - mfy*y*y/z_invsquare;
                mJacobians1(1, 1) = mfy*x*y*z_invsquare;
                mJacobians1(1, 2) = mfy*x/z_inv;
                mJacobians1(1, 3) = 0;
                mJacobians1(1, 4) = mfy*z_inv;
                mJacobians1(1, 5) = -mfy*y*z_invsquare;

                mJacobians1 = -1.0*mJacobians1;
            }

            if(jacobians!=NULL && jacobians[1]!=NULL)
            {
                Eigen::Map< Eigen::Matrix<double, 2, 3, Eigen::RowMajor> > mJacobians2(jacobians[1]);

                Eigen::Matrix<double, 2, 3, Eigen::RowMajor> mJacob_tmp;
                mJacob_tmp << mfx*z_inv, 0, -mfx*x*z_invsquare,
                            0, mfy*z_inv, -mfy*y*z_invsquare;

                mJacobians2 = -1.0*mJacob_tmp*mPose.so3().matrix();
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


class PoseLocalParameterization : public ceres::LocalParameterization
{

    virtual bool Plus(const double *T_raw, const double *delta_raw, double *T_plus_delta_raw) const
    {
        Eigen::Map<const Sophus::Vector6d> tOld(T_raw);
        Eigen::Map<const Sophus::Vector6d> tDelta(delta_raw);
        Eigen::Map<Sophus::Vector6d> tNew(T_plus_delta_raw);

        Sophus::SE3 tT_Old(Sophus::SO3::exp(tOld.tail<3>()), tOld.head<3>());
        Sophus::SE3 tT_Delta(Sophus::SO3::exp(tDelta.tail<3>()), tDelta.head<3>());
        Sophus::SE3 tT_New = tT_Old*tT_Delta;

        tNew.block(0, 0, 3, 1) = tT_New.translation();
        tNew.block(3, 0, 3, 1) = tT_New.so3().log();

        return true;
    }

    virtual bool ComputeJacobian(const double *T_raw, double *jacobian_raw) const
    {
        Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor> > J(jacobian_raw);
        J.setIdentity();

        return true;
    }

    virtual int GlobalSize() const { return 6; }

    virtual int LocalSize() const { return 6; }

};



} //namespace DSDTM


#endif //DSDTM_OPTIMIZER_H
