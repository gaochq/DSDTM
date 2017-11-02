//
// Created by buyi on 17-10-21.
//

#include "Camera.h"


int main(int argc, char **argv)
{
    std::vector<int> a;
    for (int i = 0; i < 10; ++i)
    {
        a.push_back(i);
    }
    a.assign(10,0);
    a.resize(10, 0);

    int b = a[a.size()-1];
    
    int c = static_cast<int>(5.10);

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
    return 0;
}