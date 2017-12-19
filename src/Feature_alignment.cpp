//
// Created by buyi on 17-12-15.
//

#include "Feature_alignment.h"


namespace DSDTM
{

Feature_Alignment::Feature_Alignment(Camera *camera):
        mCam(camera)
{
    GridInitalization();
}

Feature_Alignment::~Feature_Alignment()
{

}

void Feature_Alignment::GridInitalization()
{
    mMax_pts = Config::Get<int>("Camera.Max_fts");
    mPyr_levels  = Config::Get<int>("Camera.PyraLevels");
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

bool Feature_Alignment::ReprojectPoint(Frame tFrame, MapPoint *tMPoint)
{
    Eigen::Vector2d tPx = tFrame.World2Pixel(tMPoint->Get_Pose());

    if(mCam->IsInImage(cv::Point2f(tPx(0), tPx(1)), 8))
    {
        const int index = static_cast<int>(tPx(1)/mGrid.mCell_size)*mGrid.mGrid_Cols
                          + static_cast<int>(tPx(0)/mGrid.mCell_size);

        mGrid.mCells[index]->push_back(Candidate(tMPoint, tPx));

        return true;
    }

    return false;
}

void Feature_Alignment::SearchLocalPoints(Frame tFrame)
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

bool Feature_Alignment::ReprojectCell(Frame tFrame, Cell *tCell)
{
    //! Sort mappoint refer to their observation times
    tCell->sort(boost::bind(&Feature_Alignment::CellComparator, _1, _2));

    for (auto iter = tCell->begin(); iter != tCell->end(); iter++)
    {
        if(iter->mMpPoint->IsBad())
            continue;


    }
}

bool Feature_Alignment::CellComparator(Candidate &c1, Candidate &c2)
{
    return  c1.mMpPoint->Get_ObserveNums() > c2.mMpPoint->Get_ObserveNums();
}

bool Feature_Alignment::FindMatchDirect(MapPoint *tMpPoint, const Frame tFrame, Eigen::Vector2d &tPt)
{
    Feature tReferFeature;
    KeyFrame *tRefKeyframe;
    if(!(tMpPoint->Get_ClosetObs(tFrame, tReferFeature, tRefKeyframe)))
        return false;

    if(!(mCam->IsInImage(cv::Point2f(tReferFeature.mpx.x/(1<<tReferFeature.mlevel, tReferFeature.mpx.y/(1<<tReferFeature.mlevel))),
                       mHalf_PatchSize+1, tReferFeature.mlevel)))
        return false;

    Eigen::Matrix2d tA_c2r = SolveAffineMatrix(tRefKeyframe, tFrame, tReferFeature, tMpPoint);

    int tBestLevel = GetBestSearchLevel(tA_c2r, mPyr_levels);

    WarpAffine(tA_c2r, tRefKeyframe->mFrame->mvImg_Pyr[tReferFeature.mlevel], tReferFeature, tBestLevel, mPatch_WithBoarder);
}

Eigen::Matrix2d Feature_Alignment::SolveAffineMatrix(KeyFrame *tReferKframe, const Frame &tCurFrame, Feature tReferFeature,
                                                     MapPoint *tMpPoint)
{
    Eigen::Matrix2d tA_c2r;

    const int Half_PatchLarger = mHalf_PatchSize + 1;
    const int tLevel = tReferFeature.mlevel;

    const Eigen::Vector3d tRefPoint = tReferKframe->Get_Pose() * tMpPoint->Get_Pose();
    cv::Point2f tRefPx = tReferFeature.mpx;

    Eigen::Vector2d tRefPxU(tRefPx.x + Half_PatchLarger / (1 << tLevel), tRefPx.y);
    Eigen::Vector2d tRefPxV(tRefPx.x, tRefPx.y + Half_PatchLarger / (1 << tLevel));

    Eigen::Vector3d tRefPointU = mCam->Pixel2Camera(tRefPxU, tRefPoint(3));
    Eigen::Vector3d tRefPointV = mCam->Pixel2Camera(tRefPxV, tRefPoint(3));

    Sophus::SE3 tT_c2r = tReferKframe->Get_Pose().inverse() * tCurFrame.Get_Pose();
    Eigen::Vector2d tCurPxU = mCam->Camera2Pixel(tT_c2r * tRefPointU);
    Eigen::Vector2d tCurPxV = mCam->Camera2Pixel(tT_c2r * tRefPointV);

    tA_c2r.col(0) = (tRefPxU - tCurPxU)/Half_PatchLarger;
    tA_c2r.col(1) = (tRefPxV - tCurPxV)/Half_PatchLarger;

    return  tA_c2r;
}

int Feature_Alignment::GetBestSearchLevel(Eigen::Matrix2d tAffineMat, int tMaxLevel)
{
    int tSearch_Level = 0;
    double D = tAffineMat.determinant();

    while(D > 3.0 && tSearch_Level < tMaxLevel -1)
    {
        tSearch_Level++;
        D = D*0.25;
    }

    return tSearch_Level;
}

void Feature_Alignment::WarpAffine(const Eigen::Matrix2d tA_c2r, const cv::Mat &tImg_ref, Feature tRefFeature,
                                   const int tSearchLevel, uchar *tPatchLarger)
{
    int const tReferPh_Size = mHalf_PatchSize+2;
    Eigen::Matrix2f tA_r2c = tA_r2c.inverse().cast<float>();
    //Eigen::Map<Eigen::Matrix<double, 2*tReferPh_Size, 2*tReferPh_Size, Eigen::RowMajor> > tCurPatch_Mat(tPatchLarger);

    uchar *tRefPatch = tPatchLarger;
    Eigen::Vector2f tRefPx;
    tRefPx << tRefFeature.mpx.x/(1<<tRefFeature.mlevel),
            tRefFeature.mpx.y/(1<<tRefFeature.mlevel);

    //! generate the reference board patch
    Eigen::MatrixXf tWrapMat(2, 100);
    Eigen::MatrixXf tColBlock(1, 10);
    tColBlock << -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5;
    int tindex = 1;
    for (int i = -5; i < 5; ++i, ++tindex)
    {
        tWrapMat.block(0, (tindex-1)*10, 1, 10) = tColBlock;
        tWrapMat.block(1, (tindex-1)*10, 1, 10) = Eigen::MatrixXf::Ones(1, 10)*i;
    }

    //! Affine transform
    tWrapMat = tA_r2c*tWrapMat*(1/(1<<tSearchLevel));
    tWrapMat = tWrapMat.colwise() + tRefPx;

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
        if(tWrapMat(0, j) < 0 || tWrapMat(1, j) < 0 || tWrapMat(0, j) > tImg_ref.cols-1 || tWrapMat(0, j) > tImg_ref.rows-1)
            *tRefPatch = 0;
        else
        {
            unsigned char *tPtr = tImg_ref.data + tStep * tWrapMatFloor(1, j) + tWrapMatFloor(0, j);
            *tRefPatch = tCofficientW00(j)*tPtr[0] + tCofficientW01(j)*tPtr[tStep] +
                         tCofficientW10(j)*tPtr[1] + tCofficientW11(j)*tPtr[tStep+1];
        }
    }
}


} // namespace DSDTM