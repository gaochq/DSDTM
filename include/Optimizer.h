//
// Created by buyi on 17-11-9.
//

#ifndef DSDTM_OPTIMIZER_H
#define DSDTM_OPTIMIZER_H

#include "Camera.h"
#include "Frame.h"
#include <Map.h>

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

    //! Two View BA after FeatureAlignment
    static void PoseOptimization(FramePtr tCurFrame, int tIterations = 100);

    //! Local BA int LcoalMaper
    static void LocalBundleAdjustment(KeyFrame *tKF, Map *tMap);

    //! Convert SE3 into double array
    static double *se3ToDouble(Eigen::Matrix<double, 6, 1> tso3);

    //! Get the Residual of the problem
    static std::vector<double> GetReprojectReidual(const ceres::Problem &problem);

    //! Get the jacobian of SE3([t|R])
    static void jacobian_xyz2uv(const Eigen::Vector3d& xyz_in_f, Eigen::Matrix<double,2,6>& J);
};


//! 参数块的个数和后面的雅克比矩阵的维数要对应
//! The number of parameter blocks should be equal to jacobian
class BAPoseOnly_Problem: public ceres::SizedCostFunction<2, 6>
{
public:
    BAPoseOnly_Problem(const Eigen::Vector3d &tMapPoint, const Feature* tFeature, const CameraPtr tCamera):
            mMapPoint(tMapPoint), mFeature(tFeature), mCamera(tCamera)
    {
        mfx = mCamera->mfx;
        mfy = mCamera->mfy;
        mcx = mCamera->mcx;
        mcy = mCamera->mcy;
    }

    virtual bool Evaluate(double const* const* parameters, double* residuals, double **jacobians) const
    {
        //! Convert se3 into SO3
        Eigen::Matrix<double, 6, 1> tTransform;
        tTransform << parameters[0][0], parameters[0][1], parameters[0][2],
                      parameters[0][3], parameters[0][4], parameters[0][5];
        Sophus::SE3 mPose(Sophus::SO3::exp(tTransform.tail<3>()), tTransform.head<3>());

        //! column major, in camera coordinate
        Eigen::Map<Eigen::Vector2d> mResidual(residuals);
        Eigen::Vector3d tCamPoint = mPose*mMapPoint;

        //! Don't need undistortion
        Eigen::Vector2d tPrediction;
        tPrediction << tCamPoint(0)/tCamPoint(2), tCamPoint(1)/tCamPoint(2);

        Eigen::Vector2d tObservation;
        tObservation << mFeature->mNormal(0)/mFeature->mNormal(2), mFeature->mNormal(1)/mFeature->mNormal(2);

        mResidual = (tObservation - tPrediction)/(1<<mFeature->mlevel);


        if(mResidual(0)>10 || mResidual(1)>10)
            std::cout<< " optimizer error" <<std::endl;

        if(jacobians!=NULL)
        {
            double x = tCamPoint(0);
            double y = tCamPoint(1);
            double z_inv = 1.0/tCamPoint(2);
            double z_inv2 = z_inv*z_inv;

            if(jacobians[0]!=NULL)
            {
                Eigen::Map< Eigen::Matrix<double, 2, 6, Eigen::RowMajor> > mJacobians1(jacobians[0]);

                mJacobians1(0, 0) = -z_inv;
                mJacobians1(0, 1) = 0.0;
                mJacobians1(0, 2) = x*z_inv2;
                mJacobians1(0, 3) = y*mJacobians1(0,2);
                mJacobians1(0, 4) = -(1.0 + x*mJacobians1(0,2));
                mJacobians1(0, 5) = y*z_inv;

                mJacobians1(1, 0) = 0.0;
                mJacobians1(1, 1) = -z_inv;
                mJacobians1(1, 2) = y*z_inv2;
                mJacobians1(1, 3) = 1.0 + y*mJacobians1(1,2);
                mJacobians1(1, 4) = -x*mJacobians1(1,2);
                mJacobians1(1, 5) = -x*z_inv;
            }
        }

        return true;
    }

protected:
    Eigen::Vector3d     mMapPoint;
    Eigen::Matrix3d     mSqrt_Info;

    const Feature     *mFeature;
    const CameraPtr   mCamera;

    double              mfx;
    double              mfy;
    double              mcx;
    double              mcy;
};

//! 参数块的个数和后面的雅克比矩阵的维数要对应
class FullBA_Problem: public ceres::SizedCostFunction<2, 6, 3>
{
public:
    FullBA_Problem(const Feature* tFeature, const CameraPtr tCamera):
                    mFeature(tFeature), mCamera(tCamera)
    {
    }

    virtual bool Evaluate(double const* const* parameters, double* residuals, double **jacobians) const
    {
        //! Camera pose: Convert se3 into SO3
        Eigen::Matrix<double, 6, 1> tTransform;
        tTransform << parameters[0][0], parameters[0][1], parameters[0][2],
                    parameters[0][3], parameters[0][4], parameters[0][5];
        Sophus::SE3 mPose(Sophus::SO3::exp(tTransform.tail<3>()), tTransform.head<3>());

        //! MapPoint
        Eigen::Vector3d mMapPoint;
        mMapPoint << parameters[1][0], parameters[1][1], parameters[1][2];

        Eigen::Map<Eigen::Vector2d> mResidual(residuals);
        Eigen::Vector3d tCamPoint = mPose*mMapPoint;

        //! Don't need undistortion
        Eigen::Vector2d tPrediction;
        tPrediction << tCamPoint(0)/tCamPoint(2), tCamPoint(1)/tCamPoint(2);

        Eigen::Vector2d tObservation;
        tObservation << mFeature->mNormal(0)/mFeature->mNormal(2), mFeature->mNormal(1)/mFeature->mNormal(2);

        //mResidual = tObservation - tPrediction;
         mResidual = (tObservation - tPrediction)/(1<<mFeature->mlevel);

        if(mResidual(0)>10 || mResidual(1)>10)
            std::cout<< "error" <<std::endl;

        if(jacobians!=NULL)
        {
            double x = tCamPoint(0);
            double y = tCamPoint(1);
            double z_inv = 1.0/tCamPoint(2);
            double z_inv2 = z_inv*z_inv;

            if(jacobians[0]!=NULL)
            {
                Eigen::Map< Eigen::Matrix<double, 2, 6, Eigen::RowMajor> > mJacobians1(jacobians[0]);

                mJacobians1(0, 0) = -z_inv;
                mJacobians1(0, 1) = 0.0;
                mJacobians1(0, 2) = x*z_inv2;
                mJacobians1(0, 3) = y*mJacobians1(0,2);
                mJacobians1(0, 4) = -(1.0 + x*mJacobians1(0,2));
                mJacobians1(0, 5) = y*z_inv;

                mJacobians1(1, 0) = 0.0;
                mJacobians1(1, 1) = -z_inv;
                mJacobians1(1, 2) = y*z_inv2;
                mJacobians1(1, 3) = 1.0 + y*mJacobians1(1,2);
                mJacobians1(1, 4) = -x*mJacobians1(1,2);
                mJacobians1(1, 5) = -x*z_inv;
            }

            if(jacobians!=NULL && jacobians[1]!=NULL)
            {
                Eigen::Map< Eigen::Matrix<double, 2, 3, Eigen::RowMajor> > mJacobians2(jacobians[1]);

                Eigen::Matrix<double, 2, 3, Eigen::RowMajor> mJacob_tmp;
                mJacob_tmp << z_inv, 0, -x*z_inv2,
                            0, z_inv, -y*z_inv2;

                mJacobians2 = -1.0*mJacob_tmp*mPose.rotation_matrix();
            }

        }

        return true;
    }

protected:
    Eigen::Vector2d     mObservation;
    Eigen::Matrix3d     mSqrt_Info;
    Eigen::Matrix<double, 4, 4, Eigen::RowMajor>     mIntrinsic;

    const Feature     *mFeature;
    const CameraPtr   mCamera;
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
        Sophus::SE3 tT_New = tT_Delta*tT_Old;

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
