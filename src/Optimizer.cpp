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

void Optimizer::PoseOptimization(Frame &tCurFrame, int tIterations)
{
    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> tIntrinsic;
    tIntrinsic = tCurFrame.mCamera->Return_Intrinsic();

    ceres::Problem problem;

    ceres::LossFunction *loss_function;
    loss_function = new ceres::CauchyLoss(1.0);

    double *mTransform = se3ToDouble(tCurFrame.Get_Pose().log());

    std::map<size_t, MapPoint*> tObservations = tCurFrame.Get_Observations();
    size_t N = tObservations.size();
    double tPointsets[N][3];
    size_t tNum = 0;
    for (auto iter = tObservations.begin(); iter!=tObservations.end(); ++iter)
    {
        Eigen::Vector3d tPoint(iter->second->Get_Pose());
        int tIndex = iter->first;
        Eigen::Vector2d tObserves(tCurFrame.mvFeaturesUn[tIndex].x,
                                  tCurFrame.mvFeaturesUn[tIndex].y);

        //DLOG(INFO)<< "--" << tCurFrame.mvFeatures[tIndex].mlId << "--" << tObserves << "--" << iter->second;

        // only optimize camera pose
        PoseSolver_Problem* p = new PoseSolver_Problem(tPoint, tObserves, tIntrinsic);
        problem.AddResidualBlock(p, loss_function, mTransform);

        /*
        tPointsets[tNum][0] = tPoint(0);
        tPointsets[tNum][1] = tPoint(1);
        tPointsets[tNum][2] = tPoint(2);

        TwoViewBA_Problem* p = new TwoViewBA_Problem(tObserves, tIntrinsic);
        problem.AddResidualBlock(p, loss_function, mTransform, tPointsets[tNum]);
        tNum++;
        */
    }

    ceres::Solver::Options options;
    //options.linear_solver_type = ceres::DENSE_SCHUR;
#ifndef NDEBUG
    options.minimizer_progress_to_stdout =true;
#endif
    //options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = tIterations;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    Eigen::Map< Eigen::Matrix<double, 6, 1> > tFinalPose(mTransform);
    tCurFrame.Set_Pose(Sophus::SE3::exp(tFinalPose));

    tNum = 0;
    std::vector<double> tvdResidual = GetReprojectReidual(problem);
    /*
    for (auto iter = tObservations.begin(); iter!=tObservations.end(); ++iter)
    {
        //Eigen::Vector3d a = Eigen::Map<Eigen::Matrix<double, 3, 1>>(tPointsets[tNum]);
        tNum++;
        iter->second->Set_Pose(Eigen::Map<Eigen::Matrix<double, 3, 1>>(tPointsets[tNum]));

        if(tvdResidual[tNum]>1.0)
            iter->second->SetBadFlag();
    }
     */

    DLOG(INFO)<< summary.FullReport() << std::endl;
    std::cout<< "Residual: "<<std::sqrt(summary.final_cost / summary.num_residuals)<<std::endl;
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