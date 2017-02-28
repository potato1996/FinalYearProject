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
		char* uselessBuffer = new char[1024];
		long long last_time_stamp = 0;
		while (pLogFile.eof()) {
			long long this_time_stamp = 0;
			pLogFile.getline(lineBuffer, 1000);
			sscanf_s(lineBuffer+48, "%I64d",
				&this_time_stamp);
			std::cout << this_time_stamp << std::endl;		
		}
		

	}
}