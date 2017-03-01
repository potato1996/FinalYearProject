#include "SensorData.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
namespace android {
	bool SensorData::LoadLogFile(const char* path) {
		std::ifstream pLogFile(path);
		if (!pLogFile.is_open()) {
			std::cout << "Error opening file:" << path;
			return false;
		}

		char* lineBuffer = new char[1024];
		float uselessBuffer;
		while (!pLogFile.eof()) {
			DataTransaction currTransaction;
			pLogFile.getline(lineBuffer, 1000);
			sscanf_s(lineBuffer+48, "%I64d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
				&currTransaction.TimeStamp,
				&currTransaction.AccData[0], &currTransaction.AccData[1], &currTransaction.AccData[2],
				&currTransaction.GravData[0], &currTransaction.GravData[1], &currTransaction.GravData[2], 
				&currTransaction.PureAccData[0], &currTransaction.PureAccData[1], &currTransaction.PureAccData[2], 
				&currTransaction.MagData[0], &currTransaction.MagData[1], &currTransaction.MagData[2], 
				&currTransaction.GyroData[0], &currTransaction.GyroData[1], &currTransaction.GyroData[2]);
			dataTransactions.push_back(currTransaction);
		}
		totalTransactions = dataTransactions.size();
		pLogFile.close();
		

	}
	vec3_t SensorData::getAccData(int n){
		return dataTransactions[n].AccData;
	}
	vec3_t SensorData::getGyroData(int n){
		return dataTransactions[n].GyroData;
	}
	vec3_t SensorData::getMagData(int n){
		return dataTransactions[n].MagData;
	}
	long long SensorData::getTimeStamp(int n){
		return dataTransactions[n].TimeStamp;
	}
}