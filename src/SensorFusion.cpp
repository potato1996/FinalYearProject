#include "SensorFusion.h"
#include <math.h>
#include<fstream>

static const float DEFAULT_ACC_STDEV = 0.015f;
static const float DUFAULT_VISION_STDEV = 0.005f;
static const float VISION_LOST_THRESHOLD = 0.06f;
namespace android{


	void SensorFusion::update(Measurement measurement){
		if (measurement.measure_type == GYRO){
			float dT;
			if ((measurement.timestamp - mGryoTime > 0) &&
				(measurement.timestamp - mGryoTime < (int64_t)(5e7))){
				dT = (measurement.timestamp - mGryoTime) / 1000000000.0f;
				
				mFusion.handleGyro(measurement.measure_val - GryoDrift, dT);				
			}
			//mGryoTime = measurement.timestamp;
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
			//mAccTime = measurement.timestamp;
		}
	}
	void SensorFusion::initStatus(SensorData* data){
		sensorData = data;
		//Initialize using first K points
		int K = 1000;

		//InitElements
		last_vision_timestamp = 0;
		vec3_t setInitPosition;
		setInitPosition[0] = 0;
		setInitPosition[1] = 0;
		setInitPosition[2] = 0;
		Position = setInitPosition;
		PureSensorPosition = setInitPosition;
		Speed = setInitPosition;
		SpeedP[0][0] = 0;
		SpeedP[0][1] = 0;
		SpeedP[0][2] = 0;
		SpeedP[1][0] = 0;
		SpeedP[1][1] = 0;
		SpeedP[1][2] = 0;
		SpeedP[2][0] = 0;
		SpeedP[2][1] = 0;
		SpeedP[2][2] = 0;


		vec3_t calGryoDrift = 0;
		vec3_t calAccDrift = 0;
		for (int i = 0; i < K; ++i){
			calGryoDrift += sensorData->getGyroData(i);
			calAccDrift += sensorData->getAccData(i);
		}
		GryoDrift = calGryoDrift * (1.0f/K);
		calAccDrift = calAccDrift * (1.0f / K);

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
			mAccTime = currMeasurement.timestamp;
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
			mGryoTime = currMeasurement.timestamp;
		}

		//mFusion.setInitFlagTrue();
		mFusion.doInitFusion();
		vec3_t grav = 0;
		grav.z = 9.8f;
		AccDrift = calAccDrift - mFusion.getRotationMatrix()* grav;
		//vec3_t testinvertback = mFusion.getRotationMatrix() * AccDrift;
		currTransactionNum = K;
		Attitude = mFusion.getAttitude();


	}
	void SensorFusion::updatePosition(vec3_t visiondata, long long curr_vision_timestamp){
		double dt = curr_vision_timestamp - last_vision_timestamp;
		fuseVision(visiondata*(1.0e6/dt), dt/1.0e6);
		last_vision_timestamp = curr_vision_timestamp;
	}
	void SensorFusion::fuseVision(vec3_t z, float dT){
		if (dT < VISION_LOST_THRESHOLD){
			mat33_t I;
			I[0][0] = 1;
			I[0][1] = 0;
			I[0][2] = 0;
			I[1][0] = 0;
			I[1][1] = 1;
			I[1][2] = 0;
			I[2][0] = 0;
			I[2][1] = 0;
			I[2][2] = 1;
			

			mat33_t A = I;
			mat33_t B = I*dT;
			mat33_t H = I;
			mat33_t Ht = transpose(H);
			mat33_t Q = I*DEFAULT_ACC_STDEV;
			mat33_t R = I*DUFAULT_VISION_STDEV;

			//predict...
			//pure IMU speed has already been calculated
			mat33_t P1 = A*SpeedP*transpose(A) + Q;

			//kalman gain
			mat33_t k = P1* Ht * invert(R + H*P1*Ht);

			//update 
			vec3_t x1 = Speed + k * (z - H*Speed);
			SpeedP = (I - k*H)*P1;
			Speed = x1;

			Position = Position + Speed* dT;
			PureSensorPosition = Position;
		}
		else{
			//Position = PureSensorPosition;
			Position = Position + z * dT;
		}
	}
	void SensorFusion::accumulateSpeed(){
		long long timestamp = sensorData->getTimeStamp(currTransactionNum);
		if ((timestamp - mAccTime > 0) &&
			(timestamp - mAccTime < (int64_t)(1e8))){
			double dT = (timestamp - mAccTime) / 1000000000.0f;
			mat33_t rotationMatrix = dumpToRotationMatrix();
			vec3_t testAcc = sensorData->getAccData(currTransactionNum);
			vec3_t accData = invert(rotationMatrix) * (sensorData->getAccData(currTransactionNum)- AccDrift);
			accData.z -= 9.8f;
			Speed = Speed + accData * dT;
			PureSensorPosition = PureSensorPosition + Speed* dT;
			//Speed = accData;
		}
	}
	void SensorFusion::updateAttitude(bool useGYRO, bool useMAG, bool useACC){
		Measurement currMeasurement;
		if (useGYRO){
			//update GRYO
			currMeasurement.measure_type = GYRO;
			currMeasurement.measure_val = sensorData->getGyroData(currTransactionNum);
			currMeasurement.timestamp = sensorData->getTimeStamp(currTransactionNum);
			update(currMeasurement);
		}
		if (useMAG){
			//update MAG
			currMeasurement.measure_type = MAG;
			currMeasurement.measure_val = sensorData->getMagData(currTransactionNum);
			currMeasurement.timestamp = sensorData->getTimeStamp(currTransactionNum);
			update(currMeasurement);
		}
		if (useACC){
			//update ACC
			currMeasurement.measure_type = ACC;
			currMeasurement.measure_val = sensorData->getAccData(currTransactionNum);
			currMeasurement.timestamp = sensorData->getTimeStamp(currTransactionNum);
			update(currMeasurement);
		}
		Attitude = mFusion.getAttitude();
	}
	bool SensorFusion::updateOneCycle(bool useGYRO, bool useMAG, bool useACC){
		if (currTransactionNum >= sensorData->totalTransactions - 1)
			return false;
		updateAttitude(useGYRO, useMAG, useACC);
		accumulateSpeed();
		mAccTime = sensorData->getTimeStamp(currTransactionNum);
		mGryoTime = sensorData->getTimeStamp(currTransactionNum);
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
		Roll = atan2(2.f * (Q[2]*Q[3] + Q[0]*Q[1]), Q[0]*Q[0] - Q[1]*Q[1] - Q[2]*Q[2] + Q[3]*Q[3])*57.3;
		//Roll = atan2(2.f * (Q[2] * Q[3] + Q[0] * Q[1]), 1-2.0f*(Q[1]*Q[1]+Q[2]*Q[2]))*57.3;
		Pitch = asin(2.f * (Q[0]*Q[2] - Q[1]*Q[3]))* 57.3;
		Yaw = atan2(2.f * (Q[1]*Q[2] + Q[0]*Q[3]), Q[0]*Q[0] + Q[1]*Q[1] - Q[2]*Q[2] - Q[3]*Q[3])*57.3;
		//Yaw = atan2(2.f * (Q[1] * Q[2] - Q[0] * Q[3]), 2.0f*(Q[0]*Q[0] -Q[1]*Q[1])-1)*57.3;
		//Yaw += 90;
		if (Yaw > 180) Yaw -= 360;
		
	}
	mat33_t SensorFusion::dumpToRotationMatrix(){
		return mFusion.getRotationMatrix();
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

	std::ifstream visionfile("D:/cs/FinalYearProject/data/yx2s.csv");

	long long vision_basetime = 1300000;
	long long curr_vision_time = 0;
	android::vec3_t init_vision_number;
	android::vec3_t curr_vision_number;

	char tempch;
	while (curr_vision_time < vision_basetime){
		visionfile >> curr_vision_time >> tempch >> init_vision_number.y >> tempch >> init_vision_number.x;
		curr_vision_number = init_vision_number;
	}

	android::SensorData dataloader;
	dataloader.LoadLogFile("D:/cs/FinalYearProject/data/curve1.log");
	std::ofstream pOutputFile("D:/cs/FinalYearProject/data/loss_2s.log");
	android::SensorFusion sensorFusion;
	sensorFusion.initStatus(&dataloader);
	long long init_timeStamp = sensorFusion.getCurrTimeStamp();
	long long last_timeStamp = -1;
	long long timeStamp;
	sensorFusion.last_vision_timestamp = vision_basetime;

	/*
	while (sensorFusion.updateOneCycle(true,false,false)){
		timeStamp = sensorFusion.getCurrTimeStamp() - init_timeStamp;
		if (timeStamp == last_timeStamp) continue;
		last_timeStamp = timeStamp;
		//printf("%I64d, Pitch=%f, Roll=%f, Yaw =%f\n", timeStamp, Pitch, Roll, Yaw);
		//pOutputFile << " Time = " << timeStamp << " Pitch = " << Pitch << " Roll = " << Roll << " Yaw = " << Yaw <<std::endl;
	
		//float Pitch, Roll, Yaw;
		//sensorFusion.dumpToEulerAngle(Pitch, Roll, Yaw);
		//pOutputFile << timeStamp << ',' << Roll << std::endl;

		float x, y, z;
		//sensorFusion.getPosition(x,y,z);
		sensorFusion.getSpeed(x, y, z);
		pOutputFile << timeStamp << ',' << x <<','<<y<<','<<z<< std::endl;
	}
	*/
	
	while (curr_vision_time < 15119490){
		sensorFusion.updateOneCycle(true, true, false);
		timeStamp = sensorFusion.getCurrTimeStamp() - init_timeStamp;
		if (timeStamp == last_timeStamp) continue;
		last_timeStamp = timeStamp;
		if (timeStamp > (curr_vision_time - vision_basetime)*1000){
			android::vec3_t vision_position;
			vision_position.x = -(curr_vision_number.y - init_vision_number.y);
			vision_position.y = -(curr_vision_number.x - init_vision_number.x);
			vision_position.z = 0;

			sensorFusion.updatePosition(vision_position*0.001, curr_vision_time);

			init_vision_number = curr_vision_number;
			visionfile >> curr_vision_time >> tempch >> curr_vision_number.y >> tempch >> curr_vision_number.x;

		}
		float x, y, z;
		//sensorFusion.getSpeed(x, y, z);
		sensorFusion.getPosition(x, y, z);
		pOutputFile << timeStamp << ',' << -x << ',' << y << ',' << z << ',' <<curr_vision_number.y/1000 << ','<<-curr_vision_number.x/1000<< std::endl;

	}
	pOutputFile.close();
	visionfile.close();
	return 0;
}