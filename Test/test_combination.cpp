//
// Created by buyi on 17-10-21.
//

#include "Camera.h"


//! Compare two mvBinIdexProba from max to min

int mMaxIteators = 1000;

int main(int argc, char **argv)
{
    std::vector<int> b;
    b.resize(8,0);

    std::vector<int> a, c;
    for (int i = 1; i <= 100; ++i)
    {
            a.push_back(i);
    }

    int tIterator_Num = 0;
    int Erase_flag = 8;

    double start = static_cast<double>(cvGetTickCount());
    for (int i = 0; i < 100 ; ++i)
    {
        for (int j = 0; j < 8; ++j)
        {
            b[j] = a[j+i];
        }

        Erase_flag = Erase_flag+i;
        int Erase_flag_right = Erase_flag;
        int Erase_flag_left = 7;

        while (Erase_flag_left>=0)
        {
            if(tIterator_Num>=2000)
                break;

            if(Erase_flag_right==101)
            {
                Erase_flag_left--;
                if(Erase_flag_left<0)
                    break;
                Erase_flag_right = Erase_flag;
                b[Erase_flag_left+1] = a[Erase_flag_left+1+i];
            }
            else
            {

                for (int n = 0; n < 8; ++n)
                    std::cout << b[n] << " ";
                std::cout << "" << std::endl;
                tIterator_Num++;
            }

            b[Erase_flag_left] = a[Erase_flag_right];
            Erase_flag_right++;

        }
        if(tIterator_Num>=2000)
            break;
    }
    double time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
    std::cout << time << "us" << std::endl;
    std::cout<< tIterator_Num <<std::endl;

    return 0;
}