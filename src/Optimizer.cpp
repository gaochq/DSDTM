//
// Created by buyi on 17-11-9.
//

#include "Optimizer.h"


namespace DSDTM
{
    Optimizer::Optimizer()
    {

    }

    Optimizer::~Optimizer()
    {

    }

    void Optimizer::PoseSolver(Frame &tCurFrame, int tIterations)
    {
        double start = static_cast<double>(cvGetTickCount());
        Eigen::Matrix3d tIntrinsic;
        tIntrinsic = tCurFrame.mCamera->Return_Intrinsic();

        ceres::Problem problem;
        ceres::LossFunction *loss_function;
        loss_function = new ceres::CauchyLoss(1.0);
        double *mTransform = se3ToDouble(tCurFrame.Get_Pose().log());

        std::map<long int, MapPoint*> tObservations = tCurFrame.Get_Observations();
        for (auto iter = tObservations.begin(); iter!=tObservations.end(); ++iter)
        {
            Eigen::Vector3d tPoint(iter->second->Get_Pose());
            int tIndex = iter->first;
            Eigen::Vector2d tObserves(tCurFrame.mvFeatures[tIndex].mpx.x,
                                      tCurFrame.mvFeatures[tIndex].mpx.y);

            PoseGraph_Problem* p = new PoseGraph_Problem(tPoint, tObserves, tIntrinsic);
            problem.AddResidualBlock(p, loss_function, mTransform);
        }

        ceres::Solver::Options options;
        //options.linear_solver_type = ceres::DENSE_SCHUR;
#ifndef NDEBUG
        options.minimizer_progress_to_stdout =true;
#endif
        options.trust_region_strategy_type = ceres::DOGLEG;
        options.max_num_iterations = tIterations;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);



        double time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();

        DLOG(INFO) <<"Cost "<< time << " us";
        /*
        for (int j = 0; j < 6; ++j)
        {
            std::cout << mTransform[j]<< std::endl;
        }
         */
        Eigen::Map< Eigen::Matrix<double, 6, 1> > tFinalPose(mTransform);
        tCurFrame.Set_Pose(Sophus::SE3::exp(tFinalPose));

        DLOG(INFO)<< "Residual: "<<std::sqrt(summary.final_cost / summary.num_residuals);
        DLOG(INFO)<< summary.FullReport() << std::endl;
    }

    double *Optimizer::se3ToDouble(Eigen::Matrix<double, 6, 1> tse3)
    {
        double *so3_tmp = new double[6];

        so3_tmp[0] = tse3(0, 0);
        so3_tmp[1] = tse3(1, 0);
        so3_tmp[2] = tse3(2, 0);
        so3_tmp[3] = tse3(3, 0);
        so3_tmp[4] = tse3(4, 0);
        so3_tmp[5] = tse3(5, 0);

        return so3_tmp;
    }

}