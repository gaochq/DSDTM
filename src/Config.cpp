//
// Created by buyi on 17-10-16.
//


#include "Config.h"

namespace DSDTM
{

void Config::setParameterFile(const std::string& filename)
{
    if(config == nullptr)
        config = std::shared_ptr<Config>(new Config);

    config->mfile = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
    if(!config->mfile.isOpened())
    {
        std::cerr<< "Parameter file " << filename <<" does not exist."<<std::endl;
        config->mfile.release();
        return;
    }
}

Config::~Config()
{
    if (mfile.isOpened())
        mfile.release();
}

std::shared_ptr<Config> Config::config = nullptr;

}// namesapce DSDTM