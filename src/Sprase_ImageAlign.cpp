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

    if(tRefFrame->mvFeatures.size() < mnMinfts)
    {
        DLOG(ERROR)<<"Too few features to track" << std::endl;
        return 0;
    }

    mCurFrame = tCurFrame;
    mRefFrame = tRefFrame;

    mT_c2r = tCurFrame->Get_Pose()*tRefFrame->Get_Pose().inverse();

    for (int i = mnMaxLevel; i >= mnMinLevel; --i)
    {
        TicToc tc;
        GetJocabianMat(i);
        std::cout << tc.toc() << std::endl;

        DirectSolver(mT_c2r, i);
        Reset();
    }

    tCurFrame->Set_Pose(mT_c2r*tRefFrame->Get_Pose());
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
        tRefPts.col(tNum) << mRefFrame->mvFeatures[i].mpx.x,
                             mRefFrame->mvFeatures[i].mpx.y;
        tRefPts.col(tNum) = tRefPts.col(tNum)*tScale;

        tRefPoints.col(tNum) = mRefFrame->mvFeatures[i].mPoint;

        if( tRefPoints.col(tNum).isZero(0) || tRefPts(0, tNum) - boarder < 0 || tRefPts(1, tNum) - boarder < 0 ||
            tRefPts(0, tNum) + boarder >= tRefIMg.cols || tRefPts(1, tNum) + boarder >= tRefIMg.rows)
        {
            mVisible[i] = false;
            continue;
        }
        tRefNormals.col(tNum) = mRefFrame->mvFeatures[i].mNormal;
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

void Sprase_ImgAlign::DirectSolver(Sophus::SE3 &tT_c2r, int tLevel)
{
    int tPatchArea = mHalf_PatchSize*mHalf_PatchSize;
    const cv::Mat tCurImg = mCurFrame->mvImg_Pyr[tLevel];
    const float tScale = 1.0/(1<<tLevel);

    Sophus::SE3 mT_c2r = tT_c2r;

    Eigen::Matrix<double, 6, 1> tT_c2rArray;
    tT_c2rArray.block(0, 0, 3, 1) = tT_c2r.translation();
    tT_c2rArray.block(3, 0, 3, 1) = tT_c2r.so3().log();

    double *tRefPatch = (double*)mRefPatch.data();
    double *tJacobianPtr = (double*)mJocabianPatch.data();
    Camera *tCamera = mCurFrame->mCamera;

    ceres::Problem problem;
    ceres::LocalParameterization *local_Parameterization = new PoseLocalParameterization();
    problem.AddParameterBlock(tT_c2rArray.data(), 7, local_Parameterization);

    int tnPts = mRefPatch.rows();
    for (int i = 0; i < tnPts; ++i)
    {
        DirectSE3_Problem *p = new DirectSE3_Problem(mRefNormals.col(i), tRefPatch, tJacobianPtr, &tCurImg, tScale, tCamera);
        problem.AddResidualBlock(p, NULL, tT_c2rArray.data());

        tRefPatch += tPatchArea;
        tJacobianPtr += 6*tPatchArea;
    }

    ceres::Solver::Options options;
    //options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 100;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;

    tT_c2r = mT_c2r;
}

//! The projected plane is normalization plane
Eigen::Matrix<double, 2, 6> Sprase_ImgAlign::GetJocabianBA(Eigen::Vector3d tPoint)
{
    Eigen::Matrix<double, 2, 6> J;

    const double x = tPoint(0);
    const double y = tPoint(1);
    const double z_inv = 1.0/tPoint(2);
    const double z_inv2 = z_inv*z_inv;

    //! It is important to note that in sophus translation is ahead of so3

    /*
    J(0, 3) = -z_inv;
    J(0, 4) = 0.0;
    J(0, 5) = x*z_inv2;
    J(0, 0) = y*J(0, 5);
    J(0, 1) = -(1.0 + x*J(0, 5));
    J(0, 2) = y*z_inv;

    J(1, 3) = 0.0;
    J(1, 4) = -z_inv;
    J(1, 5) = y*z_inv2;
    J(1, 0) = 1.0 + y*J(1, 5);
    J(1, 1) = -J(0, 0);
    J(1, 2) = -x*z_inv;
    */

    J(0,0) = -z_inv;              // -1/z
    J(0,1) = 0.0;                 // 0
    J(0,2) = x*z_inv2;           // x/z^2
    J(0,3) = y*J(0,2);            // x*y/z^2
    J(0,4) = -(1.0 + x*J(0,2));   // -(1.0 + x^2/z^2)
    J(0,5) = y*z_inv;             // y/z

    J(1,0) = 0.0;                 // 0
    J(1,1) = -z_inv;              // -1/z
    J(1,2) = y*z_inv2;           // y/z^2
    J(1,3) = 1.0 + y*J(1,2);      // 1.0 + y^2/z^2
    J(1,4) = -x*J(1,2);             // -x*y/z^2
    J(1,5) = -x*z_inv;            // x/z


    return J;
}


}// namesapce DSDTM