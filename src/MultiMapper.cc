#include "MultiMapper.h"
#include "ORBmatcher.h"
#include "Sim3Solver.h"
#include "Optimizer.h"

#include <mutex>

namespace ORB_SLAM3 {

MultiMapper::MultiMapper() {
	mMapCount = 0;
}

void MultiMapper::Run()
{
	std::cout << "MultiMapper Thread is Running" << std::endl;
	std::cout << "Initial Number of Maps: " << mMapData.size() << std::endl;
}

void MultiMapper::AddMap(Map* pMap, KeyFrameDatabase* pKFDB, ORBVocabulary* pVoc) {
	unique_lock<mutex> lock(mMutexMapData);


}
}