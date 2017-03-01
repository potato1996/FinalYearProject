#include "SensorFusion.h"
#include <math.h>
#include<fstream>
namespace android{


	void SensorFusion::update(Measurement measurement){
		if (measurement.measure_type == GYRO){
			float dT;
			if ((measurement.timestamp - mGryoTime > 0) &&
				(measurement.timestamp - mGryoTime < (int64_t)(5e7))){
				dT = (measurement.timestamp - mGryoTime) / 1000000000.0f;
				
				mFusion.handleGyro(measurement.measure_val - GryoDrift, dT);				
			}
			mGryoTime = measurement.timestamp;
		}
		else if (measurement.measure_type == MAG){
			mFusion.handleMag(measurement.measure_val);
		}
		else if (measurement.measure_type == ACC){
			float dT;
			if ((measurement.timestamp - mAccTime > 0) &&
				(measurement.timestamp - mAccTime < (int64_t)(1e8))){
				dT = (measurement.timestamp - mAccTime) / 1000000000.0f;
				
				mFusion.handleAcc(measurement.measure_val, dT);
			}
			mAccTime = measurement.timestamp;
		}
	}
	void SensorFusion::initStatus(SensorData* data){
		sensorData = data;
		//Initialize using first K points
		int K = 1000;

		vec3_t calGryoDrift = 0;
		for (int i = 0; i < K; ++i){
			calGryoDrift += sensorData->getGyroData(i);
		}
		GryoDrift = calGryoDrift * (1.0f/K);

		//Initialize timeStamps
		mGryoTime = sensorData->getTimeStamp(0);
		mAccTime = sensorData->getTimeStamp(0);

		//Initialize fusion status
		for (int i = 0; i < K; ++i){
			Measurement currMeasurement;
			currMeasurement.measure_type = ACC;
			currMeasurement.measure_val = sensorData->getAccData(i);
			currMeasurement.timestamp = sensorData->getTimeStamp(i);
			update(currMeasurement);
		}

		for (int i = 0; i < K; ++i){
			Measurement currMeasurement;
			currMeasurement.measure_type = MAG;
			currMeasurement.measure_val = sensorData->getMagData(i);
			currMeasurement.timestamp = sensorData->getTimeStamp(i);
			update(currMeasurement);
		}

		for (int i = 0; i < K; ++i){
			Measurement currMeasurement;
			currMeasurement.measure_type = GYRO;
			currMeasurement.measure_val = sensorData->getGyroData(i);
			currMeasurement.timestamp = sensorData->getTimeStamp(i);
			update(currMeasurement);
		}

		mFusion.setInitFlagTrue();
		currTransactionNum = K;

	}
	bool SensorFusion::updateOneCycle(){
		if (currTransactionNum >= sensorData->totalTransactions -1)
			return false;
		Measurement currMeasurement;
		//update GRYO
		currMeasurement.measure_type = GYRO;
		currMeasurement.measure_val = sensorData->getGyroData(currTransactionNum);
		currMeasurement.timestamp = sensorData->getTimeStamp(currTransactionNum);
		update(currMeasurement);

		//update MAG
		currMeasurement.measure_type = MAG;
		currMeasurement.measure_val = sensorData->getMagData(currTransactionNum);
		currMeasurement.timestamp = sensorData->getTimeStamp(currTransactionNum);
		update(currMeasurement);

		//update ACC
		currMeasurement.measure_type = ACC;
		currMeasurement.measure_val = sensorData->getAccData(currTransactionNum);
		currMeasurement.timestamp = sensorData->getTimeStamp(currTransactionNum);
		update(currMeasurement);

		

		currTransactionNum++;
		return true;
	}
	void SensorFusion::dumpToEulerAngle(float& Pitch, float& Roll, float& Yaw){
		vec4_t q = mFusion.getAttitude();
		float Q[4];
		Q[0] = q.w;
		Q[1] = q.x;
		Q[2] = q.y;
		Q[3] = q.z;
		//Roll = atan2(2.f * (Q[2]*Q[3] + Q[0]*Q[1]), Q[0]*Q[0] - Q[1]*Q[1] - Q[2]*Q[2] + Q[3]*Q[3])*57.3;
		Roll = atan2(2.f * (Q[2] * Q[3] + Q[0] * Q[1]), 1-2.0f*(Q[1]*Q[1]+Q[2]*Q[2]))*57.3;
		Pitch = asin(2.f * (Q[0]*Q[2] - Q[1]*Q[3]))* 57.3;
		//Yaw = atan2(2.f * (Q[1]*Q[2] + Q[0]*Q[3]), Q[0]*Q[0] + Q[1]*Q[1] - Q[2]*Q[2] - Q[3]*Q[3])*57.3;
		Yaw = atan2(2.f * (Q[1] * Q[2] - Q[0] * Q[3]), 2.0f*(Q[0]*Q[0] -Q[1]*Q[1])-1)*57.3;
		
	}
	long long SensorFusion::getCurrTimeStamp(){
		if (sensorData){
			return sensorData->getTimeStamp(currTransactionNum);
		}
		else{
			return 0;
		}
	}
}

int main(){

	android::SensorData dataloader;
	dataloader.LoadLogFile("D:/cs/FinalYearProject/data/ANDROID1.log");
	std::ofstream pOutputFile("D:/cs/FinalYearProject/data/ANDROID1_answer.log");
	android::SensorFusion sensorFusion;
	sensorFusion.initStatus(&dataloader);
	while (sensorFusion.updateOneCycle()){
		float Pitch, Roll, Yaw;
		long long timeStamp;
		sensorFusion.dumpToEulerAngle(Pitch, Roll, Yaw);
		timeStamp = sensorFusion.getCurrTimeStamp();
		//printf("%I64d, Pitch=%f, Roll=%f, Yaw =%f\n", timeStamp, Pitch, Roll, Yaw);
		pOutputFile << " Time = " << timeStamp << " Pitch = " << Pitch << " Roll = " << Roll << " Yaw = " << Yaw <<std::endl;
	}
	pOutputFile.close();
	return 0;
}