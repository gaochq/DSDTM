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

    std::map<size_t, MapPoint*> tObservations = tCurFrame.Get_Observations();
    for (auto iter = tObservations.begin(); iter!=tObservations.end(); ++iter)
    {
        Eigen::Vector3d tPoint(iter->second->Get_Pose());
        int tIndex = iter->first;
        Eigen::Vector2d tObserves(tCurFrame.mvFeatures[tIndex].mpx.x,
                                  tCurFrame.mvFeatures[tIndex].mpx.y);

        //DLOG(INFO)<< "--" << tCurFrame.mvFeatures[tIndex].mlId << "--" << tObserves << "--" << iter->second;

        PoseGraph_Problem* p = new PoseGraph_Problem(tPoint, tObserves, tIntrinsic);
        problem.AddResidualBlock(p, loss_function, mTransform);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
#ifndef NDEBUG
    options.minimizer_progress_to_stdout =true;
#endif
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = tIterations;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    Eigen::Map< Eigen::Matrix<double, 6, 1> > tFinalPose(mTransform);
    tCurFrame.Set_Pose(Sophus::SE3::exp(tFinalPose));

    std::vector<double> tvdResidual = GetReprojectReidual(problem);

    DLOG(INFO)<< summary.FullReport() << std::endl;
    DLOG(INFO)<< "Residual: "<<std::sqrt(summary.final_cost / summary.num_residuals);
    DLOG(INFO)<< "Residual Sum: "<<std::accumulate(tvdResidual.begin(), tvdResidual.end(), 0.0);
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

std::vector<double> Optimizer::GetReprojectReidual(const ceres::Problem &problem)
{
    std::vector<double> tResult;
    std::vector<ceres::ResidualBlockId> tIds;

    problem.GetResidualBlocks(&tIds);

    for (auto &id:tIds)
    {

        std::vector<double*> tParamsBlocks;
        Eigen::Vector2d tResidual;

        problem.GetParameterBlocksForResidualBlock(id, &tParamsBlocks);
        auto cost = problem.GetCostFunctionForResidualBlock(id);

        cost->Evaluate(tParamsBlocks.data(), tResidual.data(), nullptr);
        tResult.push_back(tResidual.norm());
    }

    return tResult;
}

}