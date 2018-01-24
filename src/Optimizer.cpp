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

    size_t N = tCurFrame->mvFeatures.size();
    double tPointsets[N][3];
    size_t tNum = 0;
    std::map<int, MapPoint*> tvMpts;
    std::map<int, Eigen::Vector3d> tvMptPoses;
    for (auto iter = tCurFrame->mvFeatures.begin(); iter!=tCurFrame->mvFeatures.end(); ++iter, ++tNum)
    {
        if(!(*iter)->Mpt)
            continue;

        if((*iter)->Mpt->IsBad())
            continue;

        if(!((*iter)->mbInitial))
            continue;

        Eigen::Vector3d tPoint((*iter)->Mpt->Get_Pose());

        //BAPoseOnly_Problem *p = new BAPoseOnly_Problem(tPoint, (*iter), tCurFrame->mCamera);
        //problem.AddResidualBlock(p, NULL, tT_c2rArray.data());

        /*
        tPointsets[tNum][0] = tPoint(0);
        tPointsets[tNum][1] = tPoint(1);
        tPointsets[tNum][2] = tPoint(2);
         */
        tvMpts[tNum] = (*iter)->Mpt;
        tvMptPoses[tNum] = tPoint;
        problem.AddParameterBlock(tvMptPoses[tNum].data(), 3);
        problem.SetParameterBlockConstant(tvMptPoses[tNum].data());

        FullBA_Problem* p = new FullBA_Problem(*iter, tCurFrame->mCamera);
        problem.AddResidualBlock(p, loss_function, tT_c2rArray.data(), tvMptPoses[tNum].data());
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.minimizer_progress_to_stdout = true;
    //options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = 100;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    //std::cout<< "Residual: "<<std::sqrt(summary.initial_cost / summary.num_residuals)<<"----"<<std::sqrt(summary.final_cost / summary.num_residuals)<<std::endl;
    tCurFrame->Set_Pose(Sophus::SE3(Sophus::SO3::exp(tT_c2rArray.tail<3>()), tT_c2rArray.head<3>()));

    /*
    tNum = 0;
    for (auto &it : tvMpts)
    {
        it.second->Set_Pose(tvMptPoses[tNum]);
        tNum++;
    }
    */

    //std::vector<double> tvdResidual = GetReprojectReidual(problem);
    //std::cout << summary.FullReport() << std::endl;

    //std::cout <<"Residual Sum: "<<std::accumulate(tvdResidual.begin(), tvdResidual.end(), 0.0) << std::endl;
    //std::cout << std::endl;
}

void Optimizer::LocalBundleAdjustment(KeyFrame *tKFrame, Map *tMap)
{
    double tReprojectThresh = Config::Get<float>("Optimization.LocalBAthreshhold");
    tReprojectThresh = tReprojectThresh/tKFrame->mCamera->mf;
    double tOutlineThres = tReprojectThresh*tReprojectThresh;

    std::vector<KeyFrame*> tvLocalKeyFrmaes;
    tvLocalKeyFrmaes.push_back(tKFrame);

    //! Get Covibility keyframes and all the mappoints they observe
    std::vector<std::pair<int, KeyFrame*>> tvCovKeyFrames  = tKFrame->GetCovKFrames();
    for(auto &iter : tvCovKeyFrames)
    {
        KeyFrame *tKf = iter.second;
        //TODO check the quality of this CovKeyframe

        tKf->mlLocalBAKfId = tKFrame->mlId;
        tvLocalKeyFrmaes.push_back(iter.second);
    }

    std::vector<MapPoint*> tvLocalMapPoints;
    for (const auto &iterKF : tvLocalKeyFrmaes)
    {
        std::vector<MapPoint*> tvMps = iterKF->mvMapPoints;
        for (auto &iterMp : tvMps)
        {
            if(!iterMp)
                continue;

            if(iterMp->IsBad())
                continue;

            if(iterMp->mlLocalBAKFId!=tKFrame->mlId)
            {
                iterMp->mlLocalBAKFId = tKFrame->mlId;
                tvLocalMapPoints.push_back(iterMp);
            }
        }
    }

    //! Get Keyframes observe the lcoalMappoints but not belong to tvLocalKeyFrames
    std::vector<KeyFrame*> tvFixedLocalFrames;
    for (const auto &itMp : tvLocalMapPoints)
    {
        std::map<KeyFrame*, size_t > tmObservations = itMp->Get_Observations();
        for (auto &itKf : tmObservations)
        {
            KeyFrame *tKf = itKf.first;

            if(tKf->mlLocalBAKfId!=tKFrame->mlId && tKf->mlFixedLocalBAKfId!=tKFrame->mlId)
            {
                tKf->mlFixedLocalBAKfId = tKFrame->mlId;
                tvFixedLocalFrames.push_back(tKf);
            }
        }
    }

    //! Construct ceres problem
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    loss_function = new ceres::HuberLoss(tReprojectThresh);
    //loss_function = new ceres::CauchyLoss(1.0);

    //! Add Keyframe pose
    std::map<unsigned long, Eigen::Matrix<double, 6, 1>> tKFPoseSets;
    std::map<unsigned long, KeyFrame*> tSolveKFSets;
    for (const auto &itKF : tvLocalKeyFrmaes)
    {
        Eigen::Matrix<double, 6, 1> tT_c2w;
        Sophus::SE3 tPose = itKF->Get_Pose();
        tT_c2w.block(0, 0, 3, 1) = itKF->Get_Pose().translation();
        tT_c2w.block(3, 0, 3, 1) = itKF->Get_Pose().so3().log();

        tKFPoseSets[itKF->mlId] = tT_c2w;
        tSolveKFSets[itKF->mlId] = itKF;

        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(tKFPoseSets[itKF->mlId].data(), SIZE_POSE, local_parameterization);

        if(itKF->mlId == 0)
            problem.SetParameterBlockConstant(tKFPoseSets[itKF->mlId].data());
    }

    for (const auto &itKF : tvFixedLocalFrames)
    {
        Eigen::Matrix<double, 6, 1> tT_c2w;
        Sophus::SE3 tPose = itKF->Get_Pose();
        tT_c2w.block(0, 0, 3, 1) = itKF->Get_Pose().translation();
        tT_c2w.block(3, 0, 3, 1) = itKF->Get_Pose().so3().log();

        tKFPoseSets[itKF->mlId] = tT_c2w;
        tSolveKFSets[itKF->mlId] = itKF;

        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(tKFPoseSets[itKF->mlId].data(), SIZE_POSE, local_parameterization);
        problem.SetParameterBlockConstant(tKFPoseSets[itKF->mlId].data());
    }

    //! Add MapPoint and residual
    std::map<size_t, Eigen::Vector3d> tMpPoseSets;
    std::map<size_t, MapPoint*> tSolveMpSets;
    for (const auto &itMp : tvLocalMapPoints)
    {
        Eigen::Vector3d tPose = itMp->Get_Pose();

        tMpPoseSets[itMp->mlID] = tPose;
        tSolveMpSets[itMp->mlID] = itMp;

        problem.AddParameterBlock(tMpPoseSets[itMp->mlID].data(), 3);

        std::map<KeyFrame*, size_t> tMpObserves = itMp->Get_Observations();
        for (const auto &itKf : tMpObserves)
        {
            KeyFrame *tKf = itKf.first;

            if(!tKFPoseSets.count(tKf->mlId))
                continue;

            FullBA_Problem *p = new FullBA_Problem(tKf->mvFeatures[itKf.second], tKf->mCamera);
            problem.AddResidualBlock(p, loss_function, tKFPoseSets[tKf->mlId].data(), tMpPoseSets[itMp->mlID].data());
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.minimizer_progress_to_stdout = true;
    options.num_threads = 4;
    options.num_linear_solver_threads = 4;
    options.max_num_iterations = 10;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    //! Update the pose of Kf and Mp
    for (const auto &itKfPose : tKFPoseSets)
    {
        Sophus::SE3 tT_c2w = Sophus::SE3(Sophus::SO3::exp(itKfPose.second.tail<3>()), itKfPose.second.head<3>());

        tSolveKFSets[itKfPose.first]->Set_Pose(tT_c2w);
    }

    for (const auto &itMpPose : tMpPoseSets)
    {
        //TODO check the quality of the MapPoint
        tSolveMpSets[itMpPose.first]->Set_Pose(itMpPose.second);
    }

    //! Pick out outlines

    int tOutlineNum = 0;
    std::vector<double> tResidualSets;
    std::vector<std::pair<MapPoint*, KeyFrame*>> tObs;
    for (auto &itMp : tvLocalMapPoints)
    {
        Eigen::Vector3d tPose = itMp->Get_Pose();

        std::map<KeyFrame*, size_t> tMpObserves = itMp->Get_Observations();
        for(auto &itKf : tMpObserves)
        {
            double tRes = utils::ReprojectionError(itKf.first->mvFeatures[itKf.second]->mNormal, itKf.first->Get_Pose(), tPose);

            if(tRes > tOutlineThres)
            {
                itMp->Erase_Observation(itKf.first);
                itKf.first->Erase_MapPointMatch(itMp);

                tOutlineNum++;
            }

            tResidualSets.push_back(tRes);
            tObs.push_back(std::make_pair(itMp, itKf.first));
        }
    }

    std::cout <<"outlines: " << tOutlineNum <<std::endl;
    //std::cout << summary.FullReport() << std::endl;
    //std::cout << "Residual: "<<std::sqrt(summary.initial_cost / summary.num_residuals)<<"----"<<std::sqrt(summary.final_cost / summary.num_residuals)<<std::endl;

    //std::vector<double> tvdResidual = GetReprojectReidual(problem);
    //std::cout << "Residual Sum: "<< std::accumulate(tvdResidual.begin(), tvdResidual.end(), 0.0) << std::endl;

    //std::cout << std::endl;
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



void Optimizer::jacobian_xyz2uv(const Eigen::Vector3d &xyz_in_f, Eigen::Matrix<double, 2, 6> &J)
{
    const double x = xyz_in_f[0];
    const double y = xyz_in_f[1];
    const double z_inv = 1./xyz_in_f[2];
    const double z_inv_2 = z_inv*z_inv;

    J(0,0) = -z_inv;              // -1/z
    J(0,1) = 0.0;                 // 0
    J(0,2) = x*z_inv_2;           // x/z^2
    J(0,3) = y*J(0,2);            // x*y/z^2
    J(0,4) = -(1.0 + x*J(0,2));   // -(1.0 + x^2/z^2)
    J(0,5) = y*z_inv;             // y/z

    J(1,0) = 0.0;                 // 0
    J(1,1) = -z_inv;              // -1/z
    J(1,2) = y*z_inv_2;           // y/z^2
    J(1,3) = 1.0 + y*J(1,2);      // 1.0 + y^2/z^2
    J(1,4) = -J(0,3);             // -x*y/z^2
    J(1,5) = -x*z_inv;            // x/z
}
}