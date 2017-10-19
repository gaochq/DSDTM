//
// Created by buyi on 17-10-18.
//

#include "Camera.h"
#include "Frame.h"
#include "Feature_detection.h"

#include <fstream>

using namespace std;

void LoadImages(const string &strImageFilename, vector<string> &vstrImageFilenamesRGB, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strImageFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        //! read the first three lines of txt file
        getline(fAssociation,s);
        getline(fAssociation,s);
        getline(fAssociation,s);
        getline(fAssociation,s);

        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            //vstrImageFilenamesD.push_back(sD);

        }
    }
}

int main(int argc, char **argv)
{
    std::vector<long> a;
    std::vector<long> b;

    for (int r = 0; r < 10; ++r)
    {
        for (long i = 0; i < 100000; ++i) {
            a.push_back(i);
        }

        double start = static_cast<double>(cvGetTickCount());
        for (long j = 0; j < a.size(); ++j) {
            b.push_back(j);
        }
        double time = ((double) cvGetTickCount() - start) / cvGetTickFrequency();
        cout << time << "us" <<"  ";
        b.clear();

        long n = 0;
        double start1 = static_cast<double>(cvGetTickCount());
        std::for_each(a.begin(), a.end(), [&](int i) {
            ++n;
            b.push_back(n);
        });
        double time1 = ((double) cvGetTickCount() - start1) / cvGetTickFrequency();
        cout << time1 << "us" <<"  ";
        b.clear();

        double start2 = static_cast<double>(cvGetTickCount());
        for (std::vector<long>::iterator it = a.begin(); it != a.end(); it++) {
            b.push_back(*it);
        }
        double time2 = ((double) cvGetTickCount() - start2) / cvGetTickFrequency();
        cout << time1 << "us" <<"  "<< endl;
    }

    return  0;
}