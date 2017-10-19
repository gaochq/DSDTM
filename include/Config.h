//
// Created by buyi on 17-10-16.
//

#ifndef DSDTM_CONFIG_H
#define DSDTM_CONFIG_H

#include "Camera.h"

namespace DSDTM
{
class Config
{
private:
    static std::shared_ptr<Config> config;
    cv::FileStorage mfile;

    Config()
    {}

public:
    ~Config();

    //! Read the file
    static void setParameterFile(const std::string& filename);

    //! Read the parameter values
    template <typename T> static T Get(const std::string& key)
    {
        return T(Config::config->mfile[key]);
    }
};

}// namespace DSDTM


#endif //DSDTM_CONFIG_H
