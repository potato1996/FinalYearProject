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
		vec3_t Speed;
		mat33_t SpeedP;


		vec3_t Position;
		vec4_t Attitude;
		vec3_t GryoDrift;
		vec3_t AccDrift;
		int currTransactionNum;


	public:
		long long last_vision_timestamp;
		SensorFusion() :mGryoTime(0), mAccTime(0), sensorData(NULL)
		{
			mFusion.init(0);
		}
		void initStatus(SensorData* sensorData);
		bool SensorFusion::updateOneCycle(bool useGYRO = true, bool useMAG = false, bool useACC = false);
		void updateAttitude(bool useGYRO, bool useMAG, bool useACC);
		void updatePosition(vec3_t visiondata, long long curr_vision_timestamp);
		void getPosition(float& x,float& y, float& z){
			x = Position.x;
			y = Position.y;
			z = Position.z;
		}
		void getSpeed(float& x, float& y, float& z){
			x = Speed.x;
			y = Speed.y;
			z = Speed.z;
		}
		void update(Measurement measurement);
		void dumpToEulerAngle(float& Pinch, float& Roll, float& Yaw);
		mat33_t dumpToRotationMatrix();
		long long getCurrTimeStamp();
	private:
		void accumulateSpeed();
		void fuseVision(vec3_t z,float dT);

	};

}