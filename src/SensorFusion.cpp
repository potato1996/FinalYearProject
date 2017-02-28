#include "SensorFusion.h"
namespace android{


	void SensorFusion::update(Measurement measurement){
		if (measurement.measure_type == GYRO){
			float dT;
			if ((measurement.timestamp - mGryoTime > 0) &&
				(measurement.timestamp - mGryoTime < (int64_t)(5e7))){
				dT = (measurement.timestamp - mGryoTime) / 1000000000.0f;
				
				mFusion.handleGyro(measurement.measure_val, dT);				
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
}

int main(){

	android::SensorData dataloader;
	dataloader.LoadLogFile("D:/cs/FinalYearProject/data/ANDROID1.log");
	system("pause");
	return 0;
}