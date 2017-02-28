#pragma once
#include "SensorFusion.h"
//#include "vec.h"
#include<vector>
namespace android {
	struct DataTransaction {
		vec3_t TimeStamp;
		vec3_t AccData;
		vec3_t GyroData;
		vec3_t MagData;
	};
	class SensorData {
		std::vector<DataTransaction> dataTransactions;
	public:
		SensorData() {}
		bool LoadLogFile(const char* path);
	};
}