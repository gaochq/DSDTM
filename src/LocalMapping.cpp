//
// Created by buyi on 17-11-20.
//
#include "LocalMapping.h"


namespace DSDTM
{
LocalMapping::LocalMapping(Map *tMap):
    mMap(tMap)
{

}

void LocalMapping::InsertKeyFrame(KeyFrame *tKFrame)
{
    mlNewKeyFrames.push_back(tKFrame);
}



}//namespace DSDTM