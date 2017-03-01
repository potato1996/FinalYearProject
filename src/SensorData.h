#pragma once
#include "SensorFusion.h"
//#include "vec.h"
#include<vector>
namespace android {
	struct DataTransaction {
		long long TimeStamp;
		vec3_t AccData;
		vec3_t GyroData;
		vec3_t MagData;
		vec3_t GravData;
		vec3_t PureAccData;
	};
	class SensorData {
		std::vector<DataTransaction> dataTransactions;
	public:
		SensorData() {}
		long long totalTransactions;
		bool LoadLogFile(const char* path);
		long long getTimeStamp(int n);
		vec3_t getAccData(int n);
		vec3_t getMagData(int n);
		vec3_t getGyroData(int n);
	};
}