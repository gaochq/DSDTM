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

    void Optimizer::PoseSolver(Frame &tRefFrame, Frame &tCurFrame, int tIterations)
    {
        Eigen::Matrix3d tIntrinsic;
        tIntrinsic = tCurFrame.mCamera->Return_Intrinsic();

        ceres::Problem problem;
        ceres::LossFunction *loss_function;
        double *mTransform;
        mTransform = se3ToDouble(tCurFrame.mT_c2w.log());

        loss_function = new ceres::CauchyLoss(1.0);
        for (int i = 0; i < tRefFrame.mvMapPoints.size(); ++i)
        {
            Eigen::Vector3d tPoint(tRefFrame.mvMapPoints[i]->mPose);
            int tIndex = tRefFrame.mvMapPoints[i]->mlID;
            Eigen::Vector2d tObserves(tCurFrame.mvFeatures[tIndex].mpx.x,
                                      tCurFrame.mvFeatures[tIndex].mpx.y);
            Eigen::Vector2d tReference(tRefFrame.mvFeatures[tIndex].mpx.x,
                                       tRefFrame.mvFeatures[tIndex].mpx.y);

            PoseGraph_Problem* p = new PoseGraph_Problem(tPoint, tObserves, tIntrinsic, tReference);
            problem.AddResidualBlock(p, loss_function, mTransform);
        }

        ceres::Solver::Options options;
        //options.linear_solver_type = ceres::DENSE_SCHUR;
        options.minimizer_progress_to_stdout =true;
        options.trust_region_strategy_type = ceres::DOGLEG;
        //options.max_num_iterations = tIterations;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        for (int j = 0; j < 6; ++j)
        {
            std::cout << mTransform[j]<< std::endl;
        }

        std::cout << summary.FullReport() << std::endl;
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