//
// Created by buyi on 17-12-15.
//

#include "Feature_alignment.h"


namespace DSDTM
{

Feature_Alignment::Feature_Alignment(CameraPtr camera):
        mCam(camera)
{
    GridInitalization();
}

Feature_Alignment::~Feature_Alignment()
{

}

void Feature_Alignment::GridInitalization()
{
    mMax_pts = Config::Get<int>("Camera.Max_tkfts");
    mPyr_levels  = Config::Get<int>("Camera.MaxPyraLevels");
    //mHalf_PatchSize = Config::Get<int>("Camera.Half_PatchSize");

    mGrid.mCell_size   = Config::Get<int>("Camera.CellSize");
    mGrid.mGrid_Rows = ceil(static_cast<double>(1.0*mCam->mheight/mGrid.mCell_size));
    mGrid.mGrid_Cols = ceil(static_cast<double>(1.0*mCam->mwidth/mGrid.mCell_size));
    mGrid.mCells.resize(mGrid.mGrid_Rows*mGrid.mGrid_Cols);

    std::for_each(mGrid.mCells.begin(), mGrid.mCells.end(), [&](Cell*& c)
    {
        c = new Cell;
    });

    mGrid.mCellOrder.resize(mGrid.mCells.size());
    for (int i = 0; i < mGrid.mCells.size(); ++i)
    {
        mGrid.mCellOrder[i] = i;
    }
    std::random_shuffle(mGrid.mCellOrder.begin(), mGrid.mCellOrder.end());
}

void Feature_Alignment::ResetGrid()
{
    std::for_each(mGrid.mCells.begin(), mGrid.mCells.end(), [&](Cell*& c)
    {
       c->clear();
    });
}

bool Feature_Alignment::ReprojectPoint(FramePtr tFrame, MapPoint *tMPoint)
{
    Eigen::Vector2d tPx = tFrame->World2Pixel(tMPoint->Get_Pose());

    if(mCam->IsInImage(cv::Point2f(tPx(0), tPx(1)), 8))
    {
        const int index = static_cast<int>(tPx(1)/mGrid.mCell_size)*mGrid.mGrid_Cols
                          + static_cast<int>(tPx(0)/mGrid.mCell_size);

        mGrid.mCells[index]->push_back(Candidate(tMPoint, tPx));

        return true;
    }

    return false;
}

void Feature_Alignment::SearchLocalPoints(FramePtr tFrame)
{
    int mMatches = 0;

    for (int i = 0; i < mGrid.mCells.size(); ++i)
    {
        if(ReprojectCell(tFrame, mGrid.mCells[i]))
            mMatches++;

        if(mMatches >= mMax_pts)
            break;
    }
}

bool Feature_Alignment::ReprojectCell(FramePtr tFrame, Cell *tCell)
{
    //! Sort mappoint refer to their observation times
    tCell->sort(boost::bind(&Feature_Alignment::CellComparator, _1, _2));

    int N = tCell->size();
    for (auto iter = tCell->begin(); iter != tCell->end(); iter++)
    {
        if(iter->mMpPoint->IsBad())
            continue;

        if(tFrame->mImgMask.at<uchar>(cv::Point2f(iter->mPx[0], iter->mPx[1]))!=255)
            continue;

        bool tFindMatch = true;
        int tLevel = 0;
        tFindMatch = FindMatchDirect(iter->mMpPoint, tFrame, iter->mPx, tLevel);

        if(!tFindMatch)
            continue;

        iter->mMpPoint->IncreaseFound();

        Feature *tFeature = new Feature(tFrame.get(), cv::Point2f(iter->mPx[0], iter->mPx[1]), tLevel);
        tFeature->SetPose(iter->mMpPoint);

        cv::circle(tFrame->mImgMask, cv::Point2f(iter->mPx[0], iter->mPx[1]), mGrid.mCell_size, 0, -1);

        tFrame->Add_Feature(tFeature);
        tFrame->Add_MapPoint(iter->mMpPoint);
        //TODO Add Feature into Frame

        return true;
    }

    return false;
}

bool Feature_Alignment::CellComparator(Candidate &c1, Candidate &c2)
{
    return  c1.mMpPoint->Get_ObserveNums() > c2.mMpPoint->Get_ObserveNums();
}

bool Feature_Alignment::FindMatchDirect(const MapPoint *tMpPoint, const FramePtr tFrame, Eigen::Vector2d &tPt, int &tLevel)
{
    bool success = false;

    Feature *tReferFeature;
    KeyFrame *tRefKeyframe;

    if(!(tMpPoint->Get_ClosetObs(tFrame.get(), tReferFeature, tRefKeyframe)))
        return false;

    if(!(mCam->IsInImage(cv::Point2f(tReferFeature->mpx.x/(1<<tReferFeature->mlevel), tReferFeature->mpx.y/(1<<tReferFeature->mlevel)),
                       mHalf_PatchSize+1, tReferFeature->mlevel)))
        return false;

    Eigen::Matrix2d tA_c2r = SolveAffineMatrix(tRefKeyframe, tFrame, tReferFeature, tMpPoint);

    int tBestLevel = GetBestSearchLevel(tA_c2r, mPyr_levels-3);

    WarpAffine(tA_c2r, tRefKeyframe->mvImg_Pyr[tReferFeature->mlevel], tReferFeature, tBestLevel, mPatch_WithBoarder);

    GetPatchNoBoarder();

    Eigen::Vector2d tCurPx = tPt/(1<<tBestLevel);

    success = Align2DGaussNewton(tFrame->mvImg_Pyr[tBestLevel], mPatch_WithBoarder, mPatch, 10, tCurPx);

    tPt = tCurPx*(1<<tBestLevel);

    tLevel = tBestLevel;
    return success;
}

Eigen::Matrix2d Feature_Alignment::SolveAffineMatrix(KeyFrame *tReferKframe, const FramePtr tCurFrame, Feature *tReferFeature,
                                                     const MapPoint *tMpPoint)
{
    Eigen::Matrix2d tA_c2r;
    const int Half_PatchLarger = mHalf_PatchSize + 1;
    const int tLevel = tReferFeature->mlevel;

    Eigen::Vector3d tRefPoint = (tReferKframe->Get_CameraCnt() - tReferFeature->Mpt->Get_Pose()).norm()*tReferFeature->mNormal;

    cv::Point2f tRefPx = tReferFeature->mpx;

    Eigen::Vector2d tRefPxU(tRefPx.x + Half_PatchLarger*(1 << tLevel), tRefPx.y);
    Eigen::Vector2d tRefPxV(tRefPx.x, tRefPx.y + Half_PatchLarger*(1 << tLevel));

    Eigen::Vector3d tRefPointU = mCam->Pixel2Camera(tRefPxU, 1.0);
    Eigen::Vector3d tRefPointV = mCam->Pixel2Camera(tRefPxV, 1.0);
    tRefPointU.normalize();
    tRefPointV.normalize();
    tRefPointU = tRefPointU*(tRefPoint(2)/tRefPointU(2));
    tRefPointV = tRefPointV*(tRefPoint(2)/tRefPointV(2));

    Sophus::SE3 tT_c2r = tCurFrame->Get_Pose()*tReferKframe->Get_Pose().inverse();
    Eigen::Vector2d tCurPx = mCam->Camera2Pixel(tT_c2r * tRefPoint);
    Eigen::Vector2d tCurPxU = mCam->Camera2Pixel(tT_c2r * tRefPointU);
    Eigen::Vector2d tCurPxV = mCam->Camera2Pixel(tT_c2r * tRefPointV);

    tA_c2r.col(0) = (tCurPxU - tCurPx)/Half_PatchLarger;
    tA_c2r.col(1) = (tCurPxV - tCurPx)/Half_PatchLarger;

    return tA_c2r;
}

int Feature_Alignment::GetBestSearchLevel(Eigen::Matrix2d tAffineMat, int tMaxLevel)
{
    int tSearch_Level = 0;
    double D = tAffineMat.determinant();

    while(D > 3.0 && tSearch_Level < tMaxLevel)
    {
        tSearch_Level++;
        D = D*0.25;
    }

    return tSearch_Level;
}

void Feature_Alignment::WarpAffine(const Eigen::Matrix2d tA_c2r, const cv::Mat &tImg_ref, Feature *tRefFeature,
                                   const int tSearchLevel, uchar *tPatchLarger)
{

    int const tReferPh_Size = mHalf_PatchSize+2;
    Eigen::Matrix2f tA_r2c = tA_c2r.inverse().cast<float>();

    uchar *tRefPatch = tPatchLarger;
    Eigen::Vector2f tRefPx;
    tRefPx << tRefFeature->mpx.x/(1<<tRefFeature->mlevel),
              tRefFeature->mpx.y/(1<<tRefFeature->mlevel);


    //! generate the reference board patch
    Eigen::MatrixXf tWrapMat(2, 100);
    Eigen::MatrixXf tColBlock(1, 10);
    tColBlock << -5, -4, -3, -2, -1, 0, 1, 2, 3, 4;
    int tindex = 1;
    for (int i = -5; i < 5; ++i, ++tindex)
    {
        tWrapMat.block(0, (tindex-1)*10, 1, 10) = tColBlock;
        tWrapMat.block(1, (tindex-1)*10, 1, 10) = Eigen::MatrixXf::Ones(1, 10)*i;
    }

    //! Affine transform
    tWrapMat = tA_r2c*tWrapMat*(1/(1<<tSearchLevel));
    tWrapMat = tWrapMat.colwise() + tRefPx;

    //! Bilinear interpolation
    Eigen::MatrixXi tWrapMatFloor = tWrapMat.unaryExpr(std::ptr_fun(Eigenfloor));
    Eigen::MatrixXf tWrapMatSubpix = tWrapMat - tWrapMatFloor.cast<float>();

    Eigen::MatrixXf tWrapMatSubpixX = Eigen::MatrixXf::Ones(1, 100) - tWrapMatSubpix.row(0);
    Eigen::MatrixXf tWrapMatSubpixY = Eigen::MatrixXf::Ones(1, 100) - tWrapMatSubpix.row(1);

    Eigen::MatrixXf tCofficientW00 = tWrapMatSubpixX.cwiseProduct(tWrapMatSubpixY);
    Eigen::MatrixXf tCofficientW01 = tWrapMatSubpixX.cwiseProduct(tWrapMatSubpix.row(1));
    Eigen::MatrixXf tCofficientW10 = tWrapMatSubpix.row(0).cwiseProduct(tWrapMatSubpixY);
    Eigen::MatrixXf tCofficientW11 = Eigen::MatrixXf::Ones(1, 100) - tCofficientW00 - tCofficientW01 - tCofficientW10;

    const int tStep = tImg_ref.step.p[0];
    for (int j = 0; j < 100; ++j, tRefPatch++)
    {
        if(tWrapMat(0, j) < 0 || tWrapMat(1, j) < 0 || tWrapMat(0, j) > tImg_ref.cols - 1 || tWrapMat(1, j) > tImg_ref.rows - 1)
            *tRefPatch = 0;
        else
        {
            unsigned char *tPtr = tImg_ref.data + tStep * tWrapMatFloor(1, j) + tWrapMatFloor(0, j);
            *tRefPatch = tCofficientW00(j)*tPtr[0] + tCofficientW01(j)*tPtr[tStep] +
                         tCofficientW10(j)*tPtr[1] + tCofficientW11(j)*tPtr[tStep+1];
        }
    }

}

void Feature_Alignment::GetPatchNoBoarder()
{
    const int mPatchSize = 2*mHalf_PatchSize;
    uchar *RefPatch = mPatch;

    int tBoardPatchSize = mPatchSize +2;
    for (int i = 1; i < tBoardPatchSize-1; ++i, RefPatch +=mPatchSize)
    {
        uchar *tRowPtr = mPatch_WithBoarder + i*tBoardPatchSize + 1;
        for (int j = 0; j < mPatchSize; ++j)
        {
            RefPatch[j] = tRowPtr[j];
        }
    }
}

bool Feature_Alignment::Align2DCeres(const cv::Mat &tCurImg, uchar *tPatch_WithBoarder, uchar *tPatch,  int MaxIters, Eigen::Vector2d &tCurPx)
{
    const int tPatchSize = 2*mHalf_PatchSize;
    const int tLPatchSize = tPatchSize + 2;

    Eigen::Matrix<double, tPatchSize*tPatchSize, 3, Eigen::RowMajor> tJacobians;
    int tNum = 0;
    for (int i = 0; i < tPatchSize; ++i)
    {
        uchar* it = (uchar*) tPatch_WithBoarder + (i+1)*tLPatchSize + 1;
        for (int j = 0; j < tPatchSize; ++j, tNum++, ++it)
        {
            tJacobians.row(tNum) << 0.5*(it[1] - it[-1]),
                                    0.5*(it[tLPatchSize] - it[-tLPatchSize]),
                                    1;
        }
    }

    Eigen::Vector3d mCurpx;
    mCurpx << tCurPx(0), tCurPx(1), 0;

    ceres::Problem problem;
    problem.AddParameterBlock(mCurpx.data(), 3);

    FeatureAlign2DProblem *p = new FeatureAlign2DProblem(tPatch, &tCurImg, tJacobians.data());
    problem.AddResidualBlock(p, NULL, mCurpx.data());

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    //options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 5;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;

    tCurPx = mCurpx.head<2>();

    return true;
}

bool Feature_Alignment::Align2DGaussNewton(const cv::Mat &tCurImg, uchar *tPatch_WithBoarder, uchar *tPatch,  int MaxIters, Eigen::Vector2d &tCurPx)
{
    const int tPatchSize = 2*mHalf_PatchSize;
    const int tLPatchSize = tPatchSize + 2;


    Eigen::Matrix3f H, Hinv;
    H.setZero();
    float tRefdx[tPatchSize*tPatchSize], tRefdy[tPatchSize*tPatchSize];
    float *itdx = tRefdx;
    float *itdy = tRefdy;

    for (int l = 0; l < tPatchSize; ++l)
    {
        uchar *it = tPatch_WithBoarder + (l+1)*tLPatchSize + 1;
        for (int i = 0; i < tPatchSize; ++i, ++it, ++itdx, ++itdy)
        {
            Eigen::Vector3f J;
            J(0) = 0.5*(it[1] - it[-1]);
            J(1) = 0.5*(it[tLPatchSize] - it[-tLPatchSize]);
            J(2) = 1;
            *itdx = J(0);
            *itdy = J(1);
            H += J*J.transpose();
        }
    }

    Hinv = H.inverse();
    float mean_diff = 0;


    float u = tCurPx(0);
    float v = tCurPx(1);

    const float min_update_squared = 0.03*0.03;
    const int tCurImg_step = tCurImg.step.p[0];

    Eigen::Vector3f tUpdate;
    tUpdate.setZero();

    bool tConverged = false;
    for (int i = 0; i < MaxIters; ++i)
    {
        uchar *it_ref = tPatch;
        itdx = tRefdx;
        itdy = tRefdy;

        int u_r = floor(u);
        int v_r = floor(v);
        if(u_r < mHalf_PatchSize || v_r < mHalf_PatchSize || u_r > tCurImg.cols - mHalf_PatchSize ||
           v_r > tCurImg.rows - mHalf_PatchSize || isnan(u) || isnan(v))
            break;

        float subpix_x = u - u_r;
        float subpix_y = v - v_r;
        float wTL = (1.0 - subpix_x)*(1.0 - subpix_y);
        float wTR = subpix_x*(1 - subpix_y);
        float wBL = (1.0 - subpix_x)*subpix_y;
        float wBR = subpix_x*subpix_y;

        Eigen::Vector3f Jres;
        Jres.setZero();

        for (int j = 0; j < tPatchSize; ++j)
        {
            uchar *it = (uchar *)tCurImg.data + (v_r + j - mHalf_PatchSize)*tCurImg_step + u_r - mHalf_PatchSize;
            for (int k = 0; k < tPatchSize; ++k, ++it , ++it_ref, ++itdx, ++itdy)
            {
                float tSearchPx = wTL*it[0] + wTR*it[1] + wBL*it[tCurImg_step] + wBR*it[tCurImg_step+1];
                float tRes = tSearchPx - *it_ref + mean_diff;

                Jres[0] -= tRes*(*itdx);
                Jres[1] -= tRes*(*itdy);
                Jres[2] -= tRes;
            }
        }

        tUpdate = Hinv*Jres;
        u += tUpdate(0);
        v += tUpdate(1);
        mean_diff +=tUpdate(2);

        if(tUpdate(0)*tUpdate(0) + tUpdate(1)*tUpdate(1) < min_update_squared)
        {
            /*
            if(mean_diff > 3.0)
            {
                std::cout << "Alignment Error" << std::endl;
            }
             */

            tConverged = true;
            break;
        }
    }

    tCurPx << u, v;
    return tConverged;

}

} // namespace DSDTM