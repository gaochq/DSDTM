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

    mdOutlierThreshold = Config::Get<double>("Rarsac.Sampson_Dist_threshold");
    mMaxIteators = Config::Get<int>("Rarsac.MaxIterators");
    mdTerminThreshold = Config::Get<double>("Rarsac.Terminate_threshold");

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

//! Get the inliners and calculate the inliner probability
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
            if(tIterator_Num>=1000)
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
                //! Too slow
                //F_Mat = cv::findFundamentalMat(Fcur_pts, Fprev_pts, CV_FM_RANSAC, 1.0);
                //F_Mat = cv::findFundamentalMat(Fcur_pts, Fprev_pts, CV_FM_8POINT);

                //

                F_Mat = ComputeF21(Fcur_pts, Fprev_pts);
                //Get_Inliers(F_Mat, tvStatus);
                CheckFundamental(F_Mat, tvStatus, 1.0);

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
                Cov_Matrix = Cov_Matrix*Proba_Sum/(Proba_Sum*Proba_Sum - ProbaSqure_Sum);
                tdScore = Proba_Sum*M_PI*sqrt(Cov_Matrix.determinant());

                //! (3) compute the final score
                if(tdScore>mScore)
                {
                    mScore = tdScore;
                    mvStatus = tvStatus;
                    tBest_Iterators = 0;
                    tInlier_Proba = 0;

                    for (int m = 0; m < 100; ++m)
                    {
                        tInlier_Proba += mvGrid_probability[m]*mvBin_probability[m];
                    }
                    tInlier_Proba = pow(tInlier_Proba, 8);
                    mFrame->mvGrid_probability = mvGrid_probability;
                }

                tIterator_Num++;
                tBest_Iterators++;
                tError = pow((1-tInlier_Proba), tBest_Iterators);
                if(tError < mdTerminThreshold)
                {
                    std::cout << tIterator_Num << "--" << tBest_Iterators << std::endl;
                    std::cout << tError <<"--"<< mScore <<std::endl;
                    double Proba_Sum = std::accumulate(mFrame->mvGrid_probability.begin(), mFrame->mvGrid_probability.end(), 0.0);
                    return mvStatus;
                }

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
    std::cout << tIterator_Num << "--" << tBest_Iterators << std::endl;
    std::cout << tError <<"--"<< mScore <<std::endl;
    double Proba_Sum = std::accumulate(mFrame->mvGrid_probability.begin(), mFrame->mvGrid_probability.end(), 0.0);
    return mvStatus;
}

float Rarsac_base::CheckFundamental(const cv::Mat &F21, std::vector<bool> &_status, float sigma)
{
    int a[100] = {0};
    int b[100] = {0};
    const int N = mvCur_pts.size();

    const float f11 = F21.at<float>(0,0);
    const float f12 = F21.at<float>(0,1);
    const float f13 = F21.at<float>(0,2);
    const float f21 = F21.at<float>(1,0);
    const float f22 = F21.at<float>(1,1);
    const float f23 = F21.at<float>(1,2);
    const float f31 = F21.at<float>(2,0);
    const float f32 = F21.at<float>(2,1);
    const float f33 = F21.at<float>(2,2);

    _status.resize(N);

    float score = 0;

    const float th = 3.841;
    const float thScore = 5.991;

    const float invSigmaSquare = 1.0/(sigma*sigma);

    for(int i=0; i<N; i++)
    {
        bool bIn = true;

        const cv::Point2f &kp1 = mvCur_pts[i];
        const cv::Point2f &kp2 = mvPrev_pts[i];

        const float u1 = kp1.x;
        const float v1 = kp1.y;
        const float u2 = kp2.x;
        const float v2 = kp2.y;

        // Reprojection error in second image
        // l2=F21x1=(a2,b2,c2)

        const float a2 = f11*u1+f12*v1+f13;
        const float b2 = f21*u1+f22*v1+f23;
        const float c2 = f31*u1+f32*v1+f33;

        const float num2 = a2*u2+b2*v2+c2;

        const float squareDist1 = num2*num2/(a2*a2+b2*b2);

        const float chiSquare1 = squareDist1*invSigmaSquare;

        if(chiSquare1>th)
            bIn = false;
        else
            score += thScore - chiSquare1;

        // Reprojection error in second image
        // l1 =x2tF21=(a1,b1,c1)

        const float a1 = f11*u2+f21*v2+f31;
        const float b1 = f12*u2+f22*v2+f32;
        const float c1 = f13*u2+f23*v2+f33;

        const float num1 = a1*u1+b1*v1+c1;

        const float squareDist2 = num1*num1/(a1*a1+b1*b1);

        const float chiSquare2 = squareDist2*invSigmaSquare;

        if(chiSquare2>th)
            bIn = false;
        else
            score += thScore - chiSquare2;

        int Index = Get_GridIndex(mvCur_pts[i]);
        a[Index]++;

        if(bIn)
        {
            _status[i] = true;
            b[Index]++;
        }
        else
            _status[i]=false;
    }

    //! Set the inlier probality of the bin
    for (int j = 0; j < 100; ++j)
    {
        if(a[j]!=0)
            mvGrid_probability[j] = std::max(0.2, static_cast<double>(b[j])/a[j]);

        else
            mvGrid_probability[j] = 0.2;
    }

    return score;
}

cv::Mat Rarsac_base::Normalize_Points(std::vector <cv::Point2f> Points, std::vector <cv::Point2f> &Norm_Points)
{
    cv::Point2d Point_mean, Point_absMean;
    for (int i = 0; i < Points.size(); ++i)
    {
        Point_mean.x += Points[i].x/Points.size();
        Point_mean.y += Points[i].y/Points.size();
    }

    for (int j = 0; j < Points.size(); ++j)
    {
        Point_absMean.x += abs(Points[j].x - Point_mean.x)/Points.size();
        Point_absMean.y += abs(Points[j].y - Point_mean.y)/Points.size();
    }

    float sx = 1.0/Point_absMean.x;
    float sy = 1.0/Point_absMean.y;

    Norm_Points.resize(Points.size());
    for (int k = 0; k < Points.size(); ++k)
    {
        float a = sx*(Points[k].x - Point_mean.x);
        Norm_Points[k].x = sx*(Points[k].x - Point_mean.x);
        Norm_Points[k].y = sy*(Points[k].y - Point_mean.y);
    }

    cv::Mat transform;
    transform = cv::Mat::eye(3,3,CV_32F);
    transform.at<float>(0,0) = sx;
    transform.at<float>(1,1) = sy;
    transform.at<float>(0,2) = -Point_mean.x*sx;
    transform.at<float>(1,2) = -Point_mean.y*sy;
    return transform;
}

cv::Mat Rarsac_base::ComputeF21(const std::vector<cv::Point2f> &vP1, const std::vector<cv::Point2f> &vP2)
{
    //! Normalize the points
    cv::Mat tT1, tT2;
    std::vector<cv::Point2f> vNP1, vNP2;

    tT1 = Normalize_Points(vP1, vNP1);
    tT2 = Normalize_Points(vP2, vNP2);


    const int N = vP1.size();

    cv::Mat A(N,9,CV_32F);

    for(int i=0; i<N; i++)
    {
        const float u1 = vNP1[i].x;
        const float v1 = vNP1[i].y;
        const float u2 = vNP2[i].x;
        const float v2 = vNP2[i].y;

        A.at<float>(i,0) = u2*u1;
        A.at<float>(i,1) = u2*v1;
        A.at<float>(i,2) = u2;
        A.at<float>(i,3) = v2*u1;
        A.at<float>(i,4) = v2*v1;
        A.at<float>(i,5) = v2;
        A.at<float>(i,6) = u1;
        A.at<float>(i,7) = v1;
        A.at<float>(i,8) = 1;
    }

    cv::Mat u,w,vt, tF_Mat;

    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    cv::Mat Fpre = vt.row(8).reshape(0, 3);

    cv::SVDecomp(Fpre,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    w.at<float>(2)=0;

    tF_Mat = u*cv::Mat::diag(w)*vt;

    return tT2.t()*tF_Mat*tT1;
}


}// namespace DSDTM