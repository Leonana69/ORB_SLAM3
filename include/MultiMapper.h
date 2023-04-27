#ifndef MULTIMAPPER_H
#define MULTIMAPPER_H

#include <string>
#include "System.h"
#include "Map.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "LoopClosing.h"
#include "LocalMapping.h"
#include "Viewer.h"
#include "MapDrawer.h"
#include "FrameDrawer.h"
#include "Converter.h"
#include <iomanip>

namespace ORB_SLAM3 {
class Map;
class LoopClosing;
class LocalMapping;
class KeyFrameDatabase;

class MultiMapper {
public:
    typedef pair<Map*, KeyFrameDatabase*> MapData;
    MultiMapper();

    void Run();

    void AddMap(Map* pMap, KeyFrameDatabase* pKFDB, ORBVocabulary* pVoc);

protected:
    std::mutex mMutexMapData;
    int mMapCount;
    std::vector<MapData> mMapData;
};
}

#endif