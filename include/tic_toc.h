//
// Created by buyi on 17-12-11.
//

#ifndef DSDTM_TIC_TOC_H
#define DSDTM_TIC_TOC_H

#include <ctime>
#include <cstdlib>
#include <chrono>

namespace DSDTM
{
class TicToc
{
public:
    TicToc()
    {
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    double toc()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }

private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};


} //namespace DSDTM


#endif //DSDTM_TIC_TOC_H
