#include "Fusion.h"
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

	public:
		SensorFusion() :mGryoTime(0), mAccTime(0)
		{
			mFusion.init(3);
		}
		void update(Measurement measurement);
	};

}