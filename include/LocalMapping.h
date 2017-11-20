//
// Created by buyi on 17-11-20.
//
#include "Camera.h"
#include "Map.h"
#include "Keyframe.h"

namespace DSDTM
{
class Map;
class KeyFrame;

class LocalMapping
{
public:
    LocalMapping(Map *tMap);
    ~LocalMapping();

    void InsertKeyFrame(KeyFrame *tKFrame);

public:

protected:
    Map         *mMap;
    std::list<KeyFrame*> mlNewKeyFrames;

};



}// namespace DSDTM