//
// Created by buyi on 17-12-21.
//

#include "Sprase_ImageAlign.h"

namespace DSDTM
{

Sprase_ImgAlign::Sprase_ImgAlign(int tMaxLevel, int tMinLevel, int tMaxIterators):
        mnMaxLevel(tMaxLevel), mnMinLevel(tMinLevel), mnMaxIterators(tMaxIterators),
        mBoarder(2*mHalf_PatchSize), mPatchArea(mHalf_PatchSize*mHalf_PatchSize)
{
    mnMinfts = Config::Get<int>("Camera.Min_fts");
}

Sprase_ImgAlign::~Sprase_ImgAlign()
{

}

void Sprase_ImgAlign::Reset()
{
    mRefPatch.resize(0, 0);
    mJocabianPatch.resize(0, Eigen::NoChange);
    mRefNormals.resize(Eigen::NoChange, 0);
}

int Sprase_ImgAlign::Run(FramePtr tCurFrame, FramePtr tRefFrame)
{
    Reset();
    int mnPts;

    if(tRefFrame->mvFeatures.size() < mnMinfts)
    {
        DLOG(ERROR)<<"Too few features to track" << std::endl;
        return 0;
    }

    mCurFrame = tCurFrame;
    mRefFrame = tRefFrame;

    mT_c2r = tCurFrame->Get_Pose()*tRefFrame->Get_Pose().inverse();

    for (int i = mnMaxLevel-1; i >= mnMinLevel; --i)
    {
        GetJocabianMat(i);

        //TicToc tc;
        //CeresSolver(mT_c2r, i);
        GaussNewtonSolver(mT_c2r, i, mnPts);
        //std::cout <<"Cost "<< tc.toc() << " ms" << std::endl;

        Reset();
    }

    tCurFrame->Set_Pose(mT_c2r*tRefFrame->Get_Pose());

    return mnPts;
}

void Sprase_ImgAlign::GetJocabianMat(int tLevel)
{
    const cv::Mat tRefIMg = mRefFrame->mvImg_Pyr[tLevel];
    const float tScale = 1.0/(1<<tLevel);
    const int tRefStep = tRefIMg.step.p[0];
    const int boarder = 0.5*mHalf_PatchSize+1;

    const Eigen::Vector3d tRefCnt = mRefFrame->Get_CameraCnt();
    const float tFocalth = mRefFrame->mCamera->mf;

    Eigen::Matrix<double, 2, Eigen::Dynamic, Eigen::RowMajor> tRefPts;
    Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::RowMajor> tRefPoints;
    Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::RowMajor> tRefNormals;
    Eigen::Matrix<double, 1, Eigen::Dynamic, Eigen::RowMajor> tRefPtDepth;

    int tnPts = mRefFrame->mvFeatures.size();
    mVisible.resize(tnPts, true);
    tRefPts.resize(Eigen::NoChange, tnPts);
    tRefPoints.resize(Eigen::NoChange, tnPts);
    tRefNormals.resize(Eigen::NoChange, tnPts);

    int tNum = 0;
    for (int i = 0; i < tnPts; ++i)
    {
        if(!mRefFrame->mvFeatures[i]->mbInitial)
            continue;

        tRefPts.col(tNum) << mRefFrame->mvFeatures[i]->mpx.x,
                             mRefFrame->mvFeatures[i]->mpx.y;
        tRefPts.col(tNum) = tRefPts.col(tNum)*tScale;

        tRefPoints.col(tNum) = mRefFrame->mvFeatures[i]->mPoint;

        if( tRefPoints.col(tNum).isZero(0) || tRefPts(0, tNum) - boarder < 0 || tRefPts(1, tNum) - boarder < 0 ||
            tRefPts(0, tNum) + boarder >= tRefIMg.cols || tRefPts(1, tNum) + boarder >= tRefIMg.rows)
        {
            mVisible[i] = false;
            continue;
        }
        tRefNormals.col(tNum) = mRefFrame->mvFeatures[i]->mNormal;
        tNum++;
    }
    tRefPts.conservativeResize(Eigen::NoChange, tNum);
    tRefPoints.conservativeResize(Eigen::NoChange, tNum);
    tRefNormals.conservativeResize(Eigen::NoChange, tNum);

    mRefNormals.resize(3, tNum);
    tRefPtDepth.resize(1, tNum);

    //! Set the size fof refpatch and jacobian patch
    mRefPatch.resize(tNum, mHalf_PatchSize*mHalf_PatchSize);
    mJocabianPatch.resize(tNum*mHalf_PatchSize*mHalf_PatchSize, Eigen::NoChange);

    //! Calculate the points the reference frame
    // TODO this should be deal with independently for RGBD and monocular
    tRefPoints = tRefPoints.colwise() - tRefCnt;
    tRefPtDepth = tRefPoints.colwise().norm();
    mRefNormals = tRefNormals.array().rowwise()*tRefPtDepth.array();


    //! Calculate the coffient Bilinear difference
    Eigen::MatrixXi tRefPtsFloor = tRefPts.unaryExpr(std::ptr_fun(Eigenfloor));
    Eigen::MatrixXd tRefPtsSubpix = tRefPts - tRefPtsFloor.cast<double>();

    Eigen::MatrixXd tRefPtsSubpixX = Eigen::MatrixXd::Ones(1, tNum) - tRefPtsSubpix.row(0);
    Eigen::MatrixXd tRefPtsSubpixY = Eigen::MatrixXd::Ones(1, tNum) - tRefPtsSubpix.row(1);

    Eigen::MatrixXd tCofficientW00 = tRefPtsSubpixX.cwiseProduct(tRefPtsSubpixY);
    Eigen::MatrixXd tCofficientW01 = tRefPtsSubpix.row(0).cwiseProduct(tRefPtsSubpixY);
    Eigen::MatrixXd tCofficientW10 = tRefPtsSubpixX.cwiseProduct(tRefPtsSubpix.row(1));
    Eigen::MatrixXd tCofficientW11 = tRefPtsSubpix.row(0).cwiseProduct(tRefPtsSubpix.row(1));

    //! Calculate Jacobian matrix and reference patch
    for (int j = 0; j < tNum; ++j)
    {
        Eigen::Matrix<double, 2, 6> tJacTrans;
        tJacTrans = GetJocabianBA(mRefNormals.col(j));

        int tNum1 = 0;
        for (int i = 0; i < mHalf_PatchSize; ++i)
        {
            uchar *it = (uchar*)tRefIMg.data + (tRefPtsFloor(1, j) - 2 +i)*tRefStep + (tRefPtsFloor(0, j) - 2);

            for (int k = 0; k < mHalf_PatchSize; ++k, ++it, ++tNum1)
            {
                mRefPatch(j, tNum1) = tCofficientW00(j)*it[0] + tCofficientW01(j)*it[1]
                                   + tCofficientW10(j)*it[tRefStep] + tCofficientW11(j)*it[tRefStep+1];

                double dx = 0.5*((tCofficientW00(j)*it[1] + tCofficientW01(j)*it[2]
                                 + tCofficientW10(j)*it[tRefStep+1] + tCofficientW11(j)*it[tRefStep+2]) -
                                 (tCofficientW00(j)*it[-1] + tCofficientW01(j)*it[0]
                                 + tCofficientW10(j)*it[tRefStep-1] + tCofficientW11(j)*it[tRefStep]));

                double dy = 0.5*((tCofficientW00(j)*it[tRefStep] + tCofficientW01(j)*it[tRefStep+1]
                                 + tCofficientW10(j)*it[2*tRefStep] + tCofficientW11(j)*it[2*tRefStep+1]) -
                                 (tCofficientW00(j)*it[-tRefStep] + tCofficientW01(j)*it[-tRefStep+1]
                                 + tCofficientW10(j)*it[0] + tCofficientW11(j)*it[1]));

                mJocabianPatch.row(j*mPatchArea + tNum1) = (dx*tJacTrans.row(0) + dy*tJacTrans.row(1))*tFocalth*tScale;
            }
        }

    }

}

//! The projected plane is normalization plane
Eigen::Matrix<double, 2, 6> Sprase_ImgAlign::GetJocabianBA(Eigen::Vector3d tPoint)
{
    Eigen::Matrix<double, 2, 6> J;

    const double x = tPoint(0);
    const double y = tPoint(1);
    const double z_inv = 1.0/tPoint(2);
    const double z_inv2 = z_inv*z_inv;

    J(0,0) = -z_inv;
    J(0,1) = 0.0;
    J(0,2) = x*z_inv2;
    J(0,3) = y*J(0,2);
    J(0,4) = -(1.0 + x*J(0,2));
    J(0,5) = y*z_inv;

    J(1,0) = 0.0;
    J(1,1) = -z_inv;
    J(1,2) = y*z_inv2;
    J(1,3) = 1.0 + y*J(1,2);
    J(1,4) = -x*J(1,2);
    J(1,5) = -x*z_inv;

    return J;
}

void Sprase_ImgAlign::CeresSolver(Sophus::SE3 &tT_c2r, int tLevel)
{
    int tPatchArea = mHalf_PatchSize*mHalf_PatchSize;
    const cv::Mat tCurImg = mCurFrame->mvImg_Pyr[tLevel];
    const float tScale = 1.0/(1<<tLevel);


    Eigen::Matrix<double, 6, 1> tT_c2rArray;
    tT_c2rArray.block(0, 0, 3, 1) = tT_c2r.translation();
    tT_c2rArray.block(3, 0, 3, 1) = tT_c2r.so3().log();

    double *tRefPatch = (double*)mRefPatch.data();
    double *tJacobianPtr = (double*)mJocabianPatch.data();
    CameraPtr tCamera = mCurFrame->mCamera;

    ceres::Problem problem;
    ceres::LocalParameterization *local_Parameterization = new PoseLocalParameterization();
    problem.AddParameterBlock(tT_c2rArray.data(), 6, local_Parameterization);

    int tnPts = mRefPatch.rows();
    for (int i = 0; i < tnPts; ++i)
    {
        DirectSE3_Problem *p = new DirectSE3_Problem(mRefNormals.col(i), tRefPatch, tJacobianPtr, &tCurImg, tScale, tCamera.get());
        problem.AddResidualBlock(p, NULL, tT_c2rArray.data());

        tRefPatch += tPatchArea;
        tJacobianPtr += 6*tPatchArea;
    }

    ceres::Solver::Options options;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.linear_solver_type = ceres::DENSE_QR;
    //options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 5;

    ceres::Solver::Summary summary;
    //TicToc tc;
    ceres::Solve(options, &problem, &summary);
    //std::cout << tc.toc() << std::endl;
    //std::cout << summary.FullReport() << std::endl;

    Sophus::SE3 mT_c2r(Sophus::SO3::exp(tT_c2rArray.tail<3>()), tT_c2rArray.head<3>());
    tT_c2r = mT_c2r;
}

double Sprase_ImgAlign::ComputeResiduals(Sophus::SE3 &tT_c2r, int level, bool linearSystem, int & tnPts)
{
    int tPatchArea = mHalf_PatchSize*mHalf_PatchSize;
    const cv::Mat tCurImg = mCurFrame->mvImg_Pyr[level];
    const float tScale = 1.0/(1<<level);
    const int mnboarder = mHalf_PatchSize-1;

    double *mRefPatchAdd = (double*)mRefPatch.data();
    double chi2 = 0.0;
    int tResNum = 0;

    tnPts = 0;
    for (int n = 0; n < mRefPatch.rows(); ++n)
    {
        Eigen::Vector3d tCurPoint = tT_c2r*mRefNormals.col(n);
        Eigen::Vector2d tCurPix = mRefFrame->mCamera->Camera2Pixel(tCurPoint)*tScale;

        const double u = tCurPix(0);
        const double v = tCurPix(1);
        const int u_i = floor(u);
        const int v_i = floor(v);

        if(u_i < 0 || v_i < 0 || u_i - mnboarder < 0 || v_i - mnboarder < 0 || u_i + mnboarder >= tCurImg.cols || v_i + mnboarder >= tCurImg.rows)
            continue;

        const double tSubPixU = u - u_i;
        const double tSubPixV = v - v_i;
        const double tC_tl = (1.0 - tSubPixU)*(1.0 - tSubPixV);
        const double tC_tr = tSubPixU*(1.0 - tSubPixV);
        const double tC_bl = (1.0 - tSubPixU)*tSubPixV;
        const double tC_br = tSubPixU*tSubPixV;

        int tStep = tCurImg.step.p[0];

        int tPtnum = n*tPatchArea;
        int tNum = 0;
        for (int i = 0; i < mHalf_PatchSize; ++i)
        {
            uchar *it = tCurImg.data + (v_i + i - 2)*tCurImg.cols + u_i - 2;
            for (int j = 0; j < mHalf_PatchSize; ++j, ++it, ++tNum)
            {
                double tCurPx = tC_tl*it[0] + tC_tr*it[1] + tC_bl*it[tStep] + tC_br*it[tStep+1];
                double res = -(*(mRefPatchAdd+tNum+tPtnum) - tCurPx);

                chi2 += res*res;
                tResNum++;

                if(linearSystem)
                {
                    const Eigen::Matrix<double, 6, 1> J = mJocabianPatch.row(tPtnum + tNum).transpose();
                    H.noalias() += J * J.transpose();
                    JRes.noalias() += J * res;
                }
            }
        }
        tnPts++;
    }

    return chi2/tResNum;
}

void Sprase_ImgAlign::GaussNewtonSolver(Sophus::SE3 &tT_c2r, int level, int & tnPts)
{

    bool stop = false;
    const double eps = 1e-8;
    double chi2 = 0.0;

    Sophus::SE3 tT_c2rOld(tT_c2r);

    for (int i = 0; i < mnMaxIterators; ++i)
    {
        double chi2Diff = 0.0;

        H.setZero();
        JRes.setZero();

        double chi2New = ComputeResiduals(tT_c2r, level, true, tnPts);
        Eigen::Matrix<double, 6, 1> x = H.ldlt().solve(JRes);

        //! Failed solver
        if(bool(std::isnan(x(0))))
        {
            DLOG(ERROR) << "Matrix is close to singular!" << std::endl;
            std::cout<< "error" << std::endl;
            stop = true;
        }

        if((i > 0 && chi2New > chi2) || stop)
        {
            tT_c2r = tT_c2rOld;
            break;
        }

        //! Successful solver
        Sophus::SE3 tT_c2rNew = tT_c2r*Sophus::SE3::exp(x);
        tT_c2rOld = tT_c2r;
        tT_c2r = tT_c2rNew;

        chi2 = chi2New;

        if((x.cwiseAbs().maxCoeff() <= eps))
            break;
    }
}



}// namesapce DSDTM