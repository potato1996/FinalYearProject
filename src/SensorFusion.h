#pragma once
#include "Fusion.h"
#include "SensorData.h"
#include <stdint.h>
namespace android{

	enum MEASURE_TYPE{
		GYRO,
		ACC,
		MAG
	};

	struct Measurement{
		MEASURE_TYPE measure_type;
		vec3_t measure_val;
		long long timestamp;
	};

	class SensorFusion{
		Fusion mFusion;
		//vec4_t &mAttitude;
		long long mGryoTime;
		long long mAccTime;
		class SensorData* sensorData;
		vec3_t GryoDrift;
		int currTransactionNum;


	public:
		SensorFusion() :mGryoTime(0), mAccTime(0), sensorData(NULL)
		{
			mFusion.init(0);
		}
		void initStatus(SensorData* sensorData);
		bool updateOneCycle();
		void update(Measurement measurement);
		void dumpToEulerAngle(float& Pinch, float& Roll, float& Yaw);
		long long getCurrTimeStamp();

	};

}