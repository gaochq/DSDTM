//
// Created by buyi on 17-10-19.
//

#include "Rarsac_base.h"

namespace DSDTM
{
    Rarsac_base::Rarsac_base(Frame *_frame):
            mFrame(_frame)
    {
        mGridSize_Row = mFrame->mCamera->mheight/10;
        mGridSize_Col = mFrame->mCamera->mwidth/10;

        mvGrid_probability = mFrame->mvGrid_probability;
        mMaxIteators = 1000;
    }

    void Rarsac_base::Set_GridOccupied(cv::Point2f _pt)
    {
        mvGrid_occupancy[Get_GridIndex(_pt)] = true;
    }

    int Rarsac_base::Get_GridIndex(cv::Point2f _pt)
    {
        return static_cast<int>(_pt.y/mGridSize_Row)*10
               + static_cast<int>(_pt.x/mGridSize_Col);
    }


    void Rarsac_base::Compute_Fmat(std::vector<cv::Point2f> Cur_pts, std::vector<cv::Point2f> Prev_pts)
    {
        for (int i = 0; i < Cur_pts.size(); ++i)
        {
            mvGrid_probability[Get_GridIndex(Cur_pts[i])] = 0.5;
        }

        std::vector<cv::Point2f> Fcur_pts, Fprev_pts;
        float mError = std::numeric_limits<float>::max();
        int Iterator_Num = 0;
        cv::Mat F_Mat;

        while(mError > mdeta || Iterator_Num < mMaxIteators)
        {
            cv::findFundamentalMat(Fprev_pts, Fcur_pts, F_Mat, CV_FM_8POINT);
        }
    }

    double Rarsac_base::Sampson_Distance(cv::Point2f PointA, cv::Point2f PointB, Eigen::Matrix3d F)
    {
        double Sampson_Distance;
        Eigen::Vector3d Pointx, Pointy;

        Pointx << PointA.x, PointA.y, 1;
        Pointy << PointB.x, PointB.y, 1;
        float a = static_cast<float>(Pointy.transpose()*F*Pointx);
        float b = static_cast<float>(F.row(0)*Pointx);
        float c = static_cast<float>(F.row(1)*Pointx);
        float d = static_cast<float>(F.transpose().row(0)*Pointy);
        float e = static_cast<float>(F.transpose().row(1)*Pointy);

        Sampson_Distance = a*a/(b*b + c*c + d*d + e*e);

        return Sampson_Distance;
    }

}// namespace DSDTM