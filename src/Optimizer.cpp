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
    //problem.SetParameterBlockConstant(tT_c2rArray.data());

    size_t N = tCurFrame->mvFeatures.size();
    double tPointsets[N][3];
    size_t tNum = 0;
    for (auto iter = tCurFrame->mvFeatures.begin(); iter!=tCurFrame->mvFeatures.end(); ++iter, ++tNum)
    {
        Eigen::Vector3d tPoint((*iter)->mPoint);

        if(!((*iter)->mbInitial))
            continue;

        Eigen::Vector2d tObserves((*iter)->mpx.x,(*iter)->mpx.y);

        /*
        tPointsets[tNum][0] = tPoint(0);
        tPointsets[tNum][1] = tPoint(1);
        tPointsets[tNum][2] = tPoint(2);

        // only optimize camera pose
        TwoViewBA_Problem* p = new TwoViewBA_Problem((*iter), tCurFrame->mCamera);
        problem.AddResidualBlock(p, NULL, tT_c2rArray.data(), tPointsets[tNum]);
        */
        PoseSolver_Problem *p = new PoseSolver_Problem(tPoint, (*iter), tCurFrame->mCamera);
        problem.AddResidualBlock(p, NULL, tT_c2rArray.data());
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
//    options.minimizer_progress_to_stdout = true;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = 100;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    //std::cout<< "Residual: "<<std::sqrt(summary.initial_cost / summary.num_residuals)<<"----"<<std::sqrt(summary.final_cost / summary.num_residuals)<<std::endl;
    tCurFrame->Set_Pose(Sophus::SE3(Sophus::SO3::exp(tT_c2rArray.tail<3>()), tT_c2rArray.head<3>()));

    /*
    tNum = 0;
    for (auto iter = tCurFrame->mvMapPoints.begin(); iter!=tCurFrame->mvMapPoints.end();++iter)
    {
        (*iter)->Set_Pose(Eigen::Map<Eigen::Matrix<double, 3, 1>>(tPointsets[tNum]));
    }
     */

//    std::vector<double> tvdResidual = GetReprojectReidual(problem);
//    std::cout << summary.FullReport() << std::endl;

//    std::cout <<"Residual Sum: "<<std::accumulate(tvdResidual.begin(), tvdResidual.end(), 0.0) << std::endl;
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

void Optimizer::optimizeGaussNewton(const double reproj_thresh, const size_t n_iter, FramePtr &frame, double& error_init,
                                    double& error_final)
{
    //! Step1: 初始化
    // init
    size_t num_obs;
    double chi2(0.0);
    std::vector<double> chi2_vec_init, chi2_vec_final;

    Sophus::SE3 T_old(frame->Get_Pose());
    Eigen::Matrix<double, 6, 6> A;
    Eigen::Matrix<double, 6, 1> b;

    // compute the scale of the error for robust estimation
    //! Step2：计算重投影误差
    std::vector<float> errors;
    errors.reserve(frame->mvFeatures.size());
    chi2_vec_init.reserve(num_obs);
    chi2_vec_final.reserve(num_obs);

    for(auto it=frame->mvFeatures.begin(); it!=frame->mvFeatures.end(); ++it)
    {
        if(!((*it)->mbInitial))
            continue;

        //! 将3D点变换到当前帧坐标系下
        Eigen::Vector3d tPose = frame->Get_Pose() * (*it)->mPoint;
        Eigen::Vector2d e = Eigen::Vector2d((*it)->mNormal(0)/(*it)->mNormal(2),(*it)->mNormal(1)/(*it)->mNormal(2)) -
                            Eigen::Vector2d(tPose(0)/tPose(2), tPose(1)/tPose(2));
        //! 这种尺度变换的原理是什么呢
        e *= 1.0 / (1<<(*it)->mlevel);

        errors.push_back(e.norm());
    }
    if(errors.empty())
        return;

    num_obs = errors.size();
    chi2_vec_init.reserve(num_obs);
    chi2_vec_final.reserve(num_obs);

    //! Step3: 迭代优化当前帧位姿
    for(size_t iter=0; iter<n_iter; iter++)
    {
        b.setZero();
        A.setZero();
        double new_chi2(0.0);

        //! Step3.1：计算残差
        // compute residual
        for(auto it=frame->mvFeatures.begin(); it!=frame->mvFeatures.end(); ++it)
        {
            if(!((*it)->mbInitial))
                continue;
            Eigen::Matrix<double,2,6> J;

            //！ 当前帧坐标系下的3D点
            Eigen::Vector3d xyz_f(frame->Get_Pose() * (*it)->mPoint);

            //! 计算 de/dδT
            jacobian_xyz2uv(xyz_f, J);

            //! 相机归一化平面上的误差
            Eigen::Vector3d tPose = frame->Get_Pose() * (*it)->mPoint;
            Eigen::Vector2d e = Eigen::Vector2d((*it)->mNormal(0)/(*it)->mNormal(2),(*it)->mNormal(1)/(*it)->mNormal(2)) -
                                Eigen::Vector2d(tPose(0)/tPose(2), tPose(1)/tPose(2));
            double sqrt_inv_cov = 1.0 / (1<<(*it)->mlevel);
            e *= sqrt_inv_cov;

            if(iter == 0)
                chi2_vec_init.push_back(e.squaredNorm()); // just for debug
            J *= sqrt_inv_cov;



            //! 计算高斯牛顿的增量方程，AdT = b
            A.noalias() += J.transpose()*J;
            b.noalias() -= J.transpose()*e;
            new_chi2 += e.squaredNorm();
        }

        //! 求解更新的位姿
        // solve linear system
        //! 求解的表达是李代数的形式
        const Eigen::Matrix<double, 6, 1> dT(A.ldlt().solve(b));

        // check if error increased
        if((iter > 0 && new_chi2 > chi2) || (bool) std::isnan((double)dT[0]))
        {
            /*
            if(true)
                std::cout << "it " << iter
                          << "\t FAILURE \t new_chi2 = " << new_chi2 << std::endl;
            frame->Set_Pose(T_old); // roll-back
             */
            break;
        }

        // update the model
        Sophus::SE3 T_new = Sophus::SE3::exp(dT)*frame->Get_Pose();
        T_old = frame->Get_Pose();
        frame->Set_Pose(T_new);
        chi2 = new_chi2;
        /*
        if(true)
            std::cout << "it " << iter
                      << "\t Success \t new_chi2 = " << new_chi2
                      << "\t norm(dT) = " << dT.cwiseAbs().maxCoeff() << std::endl;
        */
        // stop when converged
        if(dT.cwiseAbs().maxCoeff() <= 1e-10)
            break;
    }

    double reproj_thresh_scaled = reproj_thresh / 315;
    size_t n_deleted_refs = 0;
    int tNum = 0;
    for(auto it=frame->mvFeatures.begin(); it!=frame->mvFeatures.end(); ++it, ++tNum)
    {
        if(!((*it)->mbInitial))
            continue;

        Eigen::Vector3d tPose = frame->Get_Pose() * (*it)->mPoint;
        Eigen::Vector2d e = Eigen::Vector2d((*it)->mNormal(0)/(*it)->mNormal(2),(*it)->mNormal(1)/(*it)->mNormal(2)) -
                            Eigen::Vector2d(tPose(0)/tPose(2), tPose(1)/tPose(2));
        double sqrt_inv_cov = 1.0 / (1<<(*it)->mlevel);
        e *= sqrt_inv_cov;
        chi2_vec_final.push_back(e.squaredNorm());
        if(e.norm() > reproj_thresh_scaled)
        {
            // we don't need to delete a reference in the point since it was not created yet
            //(*it)->point = NULL;
            frame->mvMapPoints[tNum]->SetBadFlag();
            (*it)->mbInitial = false;
            ++n_deleted_refs;
        }
    }

    error_init=0.0;
    error_final=0.0;
    if(!chi2_vec_init.empty())
        error_init = sqrt(frame->mCamera->GetMedian(chi2_vec_init))*315;
    if(!chi2_vec_final.empty())
        error_final = sqrt(frame->mCamera->GetMedian(chi2_vec_final))*315;

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