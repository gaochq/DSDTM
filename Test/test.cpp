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

    int b = a[a.size()-1];

    b++;
    return 0;
}