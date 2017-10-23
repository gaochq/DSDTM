//
// Created by buyi on 17-10-19.
//

#include "Rarsac_base.h"

namespace DSDTM
{
    Rarsac_base::Rarsac_base()
    {

    }

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
            mvBinIndexProba.push_back(tpIndexProba);
        }
        mvGrid_probability.resize(10*10, 0.0);

        std::sort(mvBinIndexProba.begin(), mvBinIndexProba.end(), CompareBin);

        //! Put features in different bin
        mvBinFeatures.resize(10*10);
        for (int i = 0; i < mvCur_pts.size(); ++i)
        {
            int Index = Get_GridIndex(mvCur_pts[i]);
            mvBinFeatures[Index].push_back(i);
        }
        for (int j = 0; j < mvBinFeatures.size(); ++j)
        {
            if(mvBinFeatures[j].size()==0)
            {
                //! Store a invalid feature index
                mvBinFeatures[j].push_back(mvCur_pts.size()+1);
            }
        }

        //! Initalize the state of features
        mvStatus.resize(mvCur_pts.size(), false);
    }

    Rarsac_base::~Rarsac_base()
    {

    }

    bool Rarsac_base::Get_Features(int Index, std::pair<cv::Point2f, cv::Point2f> &_featurePair)
    {
        if (mvBinFeatures[Index].size()>1)
        {
            std::random_shuffle(mvBinFeatures[Index].begin(), mvBinFeatures[Index].end());
        }

        if(mvBinFeatures[Index].back()< mvCur_pts.size())
        {
            _featurePair = std::make_pair(mvCur_pts[mvBinFeatures[Index].back()],
                                          mvPrev_pts[mvBinFeatures[Index].back()]);
            return true;
        }
        else
            return false;
    }


    void Rarsac_base::Set_GridOccupied(cv::Point2f _pt)
    {
        mvGrid_occupancy[Get_GridIndex(_pt)] = true;
    }

    int Rarsac_base::Get_GridIndex(cv::Point2f _pt)
    {
        return static_cast<int>(_pt.y/mGridSize_Col-1)*10
               + static_cast<int>(_pt.x/mGridSize_Row+1);
    }

    /*
    void Rarsac_base::RejectFundamental()
    {

        for (int i = 0; i < mvCur_pts.size(); ++i)
        {
            mvGrid_probability[Get_GridIndex(mvCur_pts[i])] = 0.5;
        }



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
     */

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

    void Rarsac_base::Get_Inliers(const cv::Mat _F, std::vector<bool> &_status)
    {
        _status.resize(mvCur_pts.size(), false);
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

    }

    std::vector<bool> Rarsac_base::RejectFundamental()
    {
        std::pair<cv::Point2f, cv::Point2f> tFeature_pair;
        std::vector< std::pair<int, double> > tv8binINdex;
        std::vector<cv::Point2f> Fcur_pts, Fprev_pts;
        int tIterator_Num = 0;
        int Erase_flag = 8;
        float tError = std::numeric_limits<float>::max();
        int tBest_Iterators = 0;
        double tInlier_Proba = 0;
        cv::Mat F_Mat;
        double mScore = std::numeric_limits<double>::min();

        for (int i = 0; i < 100; ++i)
        {
            int tFlagassign = 0;
            while(Fcur_pts.size()<8)
            {
                if(Get_Features(mvBinIndexProba[tFlagassign+i].first, tFeature_pair))
                {
                    Fcur_pts.push_back(tFeature_pair.first);
                    Fprev_pts.push_back(tFeature_pair.second);
                }
                tFlagassign++;
            }

            Erase_flag = Erase_flag+i;
            int Erase_flag_right = Erase_flag;
            int Erase_flag_left = 7;

            while (Erase_flag_left>=0)
            {
                if(tIterator_Num>=2000)
                    break;

                if(Erase_flag_right==100)
                {
                    Erase_flag_left--;
                    if(Erase_flag_left<0)
                        break;
                    Erase_flag_right = Erase_flag;
                    if(Get_Features(mvBinIndexProba[Erase_flag_left+1+i].first, tFeature_pair))
                    {
                        Fcur_pts[Erase_flag_left + 1] = tFeature_pair.first;
                        Fprev_pts[Erase_flag_left + 1] = tFeature_pair.second;
                    }
                }

                //! The main part of rarsac
                else
                {
                    std::vector<bool> tvStatus;

                    //! Compute the F matrix and get inliers
                    F_Mat = cv::findFundamentalMat(mvCur_pts, mvPrev_pts, CV_FM_RANSAC);
                    Get_Inliers(F_Mat, tvStatus);

                    //! Compute the score
                    double Proba_Sum=0, tdScore=0, ProbaSqure_Sum=0;
                    Eigen::Vector2d Wighted_pos(0.0, 0.0);
                    Eigen::Vector2d WightedMean_pos(0.0, 0.0);
                    Eigen::Matrix2d Cov_Matrix = Eigen::MatrixXd::Zero(2,2);


                    //! (1) compute the mean position of inliers
                    for (int v = 0; v < 100; ++v)
                    {
                        Proba_Sum += mvGrid_probability[v];
                        ProbaSqure_Sum += mvGrid_probability[v]*mvGrid_probability[v];
                        Wighted_pos += mvGrid_probability[v]*Eigen::Vector2d(mGridSize_Row*v/10 + mHalf_GridHeight,
                                                                             mGridSize_Col*v%10 + mHalf_GridWidth);
                    }
                    WightedMean_pos = Wighted_pos/Proba_Sum;

                    //! (2) compute the covariance matrix
                    for (int l = 0; l < 100; ++l)
                    {
                        Eigen::Vector2d Position_tmp;
                        Position_tmp = Eigen::Vector2d(mGridSize_Row*l/10 + mHalf_GridHeight,
                                                       mGridSize_Col*l%10 + mHalf_GridWidth)- WightedMean_pos;
                        Cov_Matrix += mvGrid_probability[l]*Position_tmp*Position_tmp.transpose();
                    }
                    Cov_Matrix = Proba_Sum/(Proba_Sum*Proba_Sum - ProbaSqure_Sum)*Cov_Matrix;
                    tdScore = Proba_Sum*M_PI*sqrt(Cov_Matrix.determinant());

                    //! (3) compute the final score
                    if(tdScore>mScore)
                    {
                        mvGrid_probability.resize(10*10, 0);
                        mScore = tdScore;
                        mvStatus = tvStatus;
                        tBest_Iterators = 0;
                        tInlier_Proba = 0;

                        for (int m = 0; m < 100; ++m)
                        {
                            tInlier_Proba += mvGrid_probability[m]*mvBin_probability[m];
                        }
                        tInlier_Proba = pow(tInlier_Proba, 8);
                    }

                    tIterator_Num++;
                    tBest_Iterators++;
                    tError = pow((1-tInlier_Proba), tBest_Iterators);
                    if(tError<mdeta)
                        break;

                    tIterator_Num++;
                }
                if(Get_Features(mvBinIndexProba[Erase_flag_right].first, tFeature_pair))
                {
                    Fcur_pts[Erase_flag_left] = tFeature_pair.first;
                    Fprev_pts[Erase_flag_left] = tFeature_pair.second;
                }
                Erase_flag_right++;
            }

        }

        return mvStatus;
    }

}// namespace DSDTM