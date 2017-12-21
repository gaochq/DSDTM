//
// Created by buyi on 17-10-21.
//

#include "Camera.h"


double Eigenceil(double x)
{
    return ceil(x);
}

int main(int argc, char **argv)
{
    //google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();

    //! Have same meaning
    //FLAGS_logtostderr = true;
    //FLAGS_stderrthreshold = 0;
    FLAGS_log_dir = "./log";

    Eigen::Vector2d tTestCeil(1.1, 1.6);
    Eigen::Vector2d tTestFinal = tTestCeil.unaryExpr(std::ptr_fun(Eigenceil));

    double aa = ceil(tTestCeil(0));
    double bb = ceil(tTestCeil(1));

    /*
    Eigen::MatrixXd tMatTest(10000, 10000);
    for (int k = 0; k < 10000; ++k)
    {
        for (int i = 0; i < 10000; ++i)
        {
            tMatTest(k,i) = 1.0*k + 0.22;
        }
    }
    std::cout << tMatTest(999,100) << std::endl;

    DSDTM::TicToc tc;
    for (int k = 0; k < 10000; ++k)
    {
        for (int i = 0; i < 10000; ++i)
        {
            tMatTest(k,i) = ceil(tMatTest(k,i));
        }
    }
    std::cout << tc.toc() << std::endl;
    std::cout << tMatTest(999,100) << std::endl;

    for (int k = 0; k < 10000; ++k)
    {
        for (int i = 0; i < 10000; ++i)
        {
            tMatTest(k,i) = 1.0*k + 0.22;
        }
    }
    std::cout << tMatTest(999,100) << std::endl;

    //Eigen::MatrixXd tMatTestCeil(10000, 10000);
    DSDTM::TicToc tcc;
    Eigen::MatrixXd tMatTestCeil = tMatTest.unaryExpr(std::ptr_fun(Eigenceil));
    std::cout << tcc.toc() << std::endl;
    std::cout << tMatTestCeil(999,100) << std::endl;
    */

    Eigen::MatrixXf tWrapMat(2, 100);
    Eigen::MatrixXf tColBlock(1, 10);
    tColBlock << -5, -4, -3, -2, -1, 0, 1, 2, 3, 4;
    tWrapMat = Eigen::MatrixXf::Zero(2, 100);
    int tindex = 1;
    for (int i = -5; i < 5; ++i, ++tindex)
    {
        tWrapMat.block(0, (tindex-1)*10, 1, 10) = tColBlock;
        tWrapMat.block(1, (tindex-1)*10, 1, 10) = Eigen::MatrixXf::Ones(1, 10)*i;
    }
    std::cout << tWrapMat << std::endl;

    double tPt3[3];
    Eigen::Map<Eigen::Vector3d> tPtv3(tPt3);
    tPtv3 << 1, 2, 3;
    for (int k = 0; k < 3; ++k)
    {
        std::cout<< tPt3[k] << std::endl;
    }


    std::vector<int> a;
    for (int i = 0; i < 10; ++i)
    {
        a.push_back(i);
    }
    a.assign(10, 0);
    a.resize(10, 0);

    int b = a[a.size()-1];
    DLOG(INFO)<< "b = " << b;
    
    int c = static_cast<int>(5.10);
    DLOG(WARNING)<< "c = " << c;

    LOG(ERROR)<< "finished";
    //DLOG(FATAL)<< "finished";

    cv::Mat ModelIMg(480, 640, CV_8UC1, cv::Scalar(255, 255, 255));
    cv::cvtColor(ModelIMg, ModelIMg, CV_GRAY2BGR);
    for (int j = 0; j < 10; ++j)
    {
        cv::line(ModelIMg, cv::Point2f(64*j,0 ), cv::Point2f(64*j,479 ), cv::Scalar(0, 255, 0), 2, 8);
        cv::line(ModelIMg, cv::Point2f(0, 48*j), cv::Point2f(639, 48*j), cv::Scalar(0, 255, 0), 2, 8);
    }
    cv::imwrite("model.jpg", ModelIMg);
    cv::namedWindow("Model");
    cv::imshow("Model", ModelIMg);
    cv::waitKey(1);
    b++;

//! Release--NDEBUG     Debug--NDEBUG
#ifdef NDEBUG
    std::cout <<"aa" <<std::endl;
#else
    std::cout <<"bb" <<std::endl;
#endif
    return 0;
}