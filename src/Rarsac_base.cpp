//
// Created by buyi on 17-10-19.
//

#include "Rarsac_base.h"

namespace DSDTM
{
    Rarsac_base::Rarsac_base(Frame *_frame, std::vector<cv::Point2f> &_pts1, std::vector<cv::Point2f> &_pts2):
            mFrame(_frame), mvCur_pts(_pts1), mvPrev_pts(_pts2)
    {
        mGridSize_Row = mFrame->mCamera->mheight/10;
        mGridSize_Col = mFrame->mCamera->mwidth/10;
        mHalf_GridHeight = mGridSize_Row/2;
        mHalf_GridWidth = mGridSize_Col/2;

        mdOutlierThreshold = Config::Get<double>("Rarsac.Threshold");
        mMaxIteators = Config::Get<int>("Rarsac.MaxIterators");

        //! compute the inlier probability of each bin
        mvBin_probability.resize(10*10, 0.0);
        double Proba_Sum = std::accumulate(mFrame->mvGrid_probability.begin(), mFrame->mvGrid_probability.end(), 0.0);
        for (int k = 0; k < 100; ++k)
        {
            mvBin_probability[k] = mFrame->mvGrid_probability[k]/Proba_Sum;

            std::pair<int, double> tpIndexProba = std::make_pair(k, mvBin_probability[k]);
            mvBinIdexProba.push_back(tpIndexProba);
        }
        mvGrid_probability.resize(10*10, 0.0);

        std::sort(mvBinIdexProba.begin(), mvBinIdexProba.end(), CompareBin);
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


    void Rarsac_base::RejectFundamental()
    {

        for (int i = 0; i < mvCur_pts.size(); ++i)
        {
            mvGrid_probability[Get_GridIndex(mvCur_pts[i])] = 0.5;
        }

        mvStatus.resize(mvCur_pts.size(), false);

        std::vector<cv::Point2f> Fcur_pts, Fprev_pts;
        float tError = std::numeric_limits<float>::max();
        int tIterator_Num = 0;
        int tBest_Iterators = 0;
        double tInlier_Proba = 0;
        cv::Mat F_Mat;
        double mScore = std::numeric_limits<double>::min();

        while(tError > mdeta || tIterator_Num < mMaxIteators)
        {

            std::vector<bool> tvStatus;

            //! Compute the F matrix and get inliers
            cv::findFundamentalMat(Fprev_pts, Fcur_pts, F_Mat, CV_FM_8POINT);
            Get_Inliers(F_Mat, tvStatus);

            //! Compute the score
            double Proba_Sum=0, tdScore=0, ProbaSqure_Sum=0;
            Eigen::Vector2d Wighted_pos(0.0, 0.0);
            Eigen::Vector2d WightedMean_pos(0.0, 0.0);
            Eigen::Matrix2d Cov_Matrix = Eigen::MatrixXd::Zero(2,2);


            //! (1) compute the mean position of inliers
            for (int i = 0; i < 100; ++i)
            {
                Proba_Sum += mvGrid_probability[i];
                ProbaSqure_Sum += mvGrid_probability[i]*mvGrid_probability[i];
                Wighted_pos += mvGrid_probability[i]*Eigen::Vector2d(mGridSize_Row*i/10 + mHalf_GridHeight,
                                                                     mGridSize_Col*i%10 + mHalf_GridWidth);
            }
            WightedMean_pos = Wighted_pos/Proba_Sum;

            //! (2) compute the covariance matrix
            for (int j = 0; j < 100; ++j)
            {
                Eigen::Vector2d Position_tmp;
                Position_tmp = Eigen::Vector2d(mGridSize_Row*j/10 + mHalf_GridHeight,
                                               mGridSize_Col*j%10 + mHalf_GridWidth)- WightedMean_pos;
                Cov_Matrix += mvGrid_probability[j]*Position_tmp*Position_tmp.transpose();
            }
            Cov_Matrix = Proba_Sum/(ProbaSqure_Sum - Proba_Sum*Proba_Sum)*Cov_Matrix;
            tdScore = Proba_Sum*M_PI*Cov_Matrix.determinant();

            //! (3) compute the final score
            if(tdScore>mScore)
            {
                mvGrid_probability.resize(10*10, 0);
                mScore = tdScore;
                mvStatus = tvStatus;
                tBest_Iterators = 0;
                tInlier_Proba = 0;

                for (int k = 0; k < 100; ++k)
                {
                    tInlier_Proba += mvGrid_probability[k]*mvBin_probability[k];
                }
                tInlier_Proba = pow(tInlier_Proba, 8);
            }

            tIterator_Num++;
            tBest_Iterators++;
            tError = pow((1-tInlier_Proba), tBest_Iterators);
        }
    }

    double Rarsac_base::Sampson_Distance(cv::Point2f PointA, cv::Point2f PointB, cv::Mat _F)
    {
        double Sampson_Distance;
        Eigen::Vector3d Pointx, Pointy;
        Eigen::Matrix3d F;
        cv::cv2eigen(_F, F);

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

    void Rarsac_base::Get_Inliers(const cv::Mat _F, std::vector<bool> _status)
    {
        int a[100] = {0};
        int b[100] = {0};

        //! Get inliers
        for (int i = 0; i <mvCur_pts.size(); ++i)
        {
            int Index = Get_GridIndex(mvCur_pts[i]);
            a[Index]++;
            if (Sampson_Distance(mvCur_pts[i], mvPrev_pts[i], _F) < mdOutlierThreshold)
            {
                _status[i] = true;
                b[Index]++;
            }
        }

        //! Set the inlier probality of the bin
        for (int j = 0; j < 100; ++j)
        {
            if(a[j]!=0)
                mvGrid_probability[j] = std::max(0.2, static_cast<double>(b[j])/a[j]);
            else
                mvGrid_probability[j] = 0.2;
        }

        /*
        double Proba_Sum = std::accumulate(mvGrid_probability.begin(), mvGrid_probability.end(), 0.0);
        for (int k = 0; k < 100; ++k)
        {
            mvBin_probability[k] = mvGrid_probability[k]/Proba_Sum;
        }
         */

    }

    void Rarsac_base::ComputeFundamental()
    {
        std::vector<cv::Point2f> Fcur_pts, Fprev_pts;
        int tIterator_Num = 0;

        for (int i = 0; i < mMaxIteators; ++i)
        {

        }

    }


}// namespace DSDTM