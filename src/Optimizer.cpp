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

void Optimizer::PoseOptimization(FramePtr tCurFrame, int tIterations)
{
    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> tIntrinsic;
    tIntrinsic = tCurFrame->mCamera->Return_Intrinsic();

    ceres::Problem problem;

    ceres::LossFunction *loss_function;
    loss_function = new ceres::CauchyLoss(1.0);

    Eigen::Matrix<double, 6, 1> tT_c2rArray;
    tT_c2rArray.block(0, 0, 3, 1) = tCurFrame->Get_Pose().translation();
    tT_c2rArray.block(3, 0, 3, 1) = tCurFrame->Get_Pose().so3().log();

    ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
    problem.AddParameterBlock(tT_c2rArray.data(), SIZE_POSE, local_parameterization);

    size_t tNum = 0;
    for (auto iter = tCurFrame->mvFeatures.begin(); iter!=tCurFrame->mvFeatures.end(); ++iter, ++tNum)
    {
        Eigen::Vector3d tPoint((*iter)->mPoint);

        if(!((*iter)->mbInitial))
            continue;

        Eigen::Vector2d tObserves((*iter)->mpx.x,(*iter)->mpx.y);

        // only optimize camera pose
        PoseSolver_Problem* p = new PoseSolver_Problem(tPoint, tObserves, tIntrinsic);
        problem.AddResidualBlock(p, NULL, tT_c2rArray.data());
    }

    ceres::Solver::Options options;
    //options.linear_solver_type = ceres::SPARSE_SCHUR;

#ifndef NDEBUG
    options.minimizer_progress_to_stdout = true;
#endif

    //options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = 100;


    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    tCurFrame->Set_Pose(Sophus::SE3(Sophus::SO3::exp(tT_c2rArray.tail<3>()), tT_c2rArray.head<3>()));

    std::vector<double> tvdResidual = GetReprojectReidual(problem);
    std::cout << summary.FullReport() << std::endl;
    std::cout<< "Residual: "<<std::sqrt(summary.initial_cost / summary.num_residuals)<<"----"<<std::sqrt(summary.final_cost / summary.num_residuals)<<std::endl;

    std::cout <<"Residual Sum: "<<std::accumulate(tvdResidual.begin(), tvdResidual.end(), 0.0) << std::endl;
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