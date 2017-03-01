/*
Modified Based On http://androidxref.com/7.0.0_r1/xref/frameworks/native/services/sensorservice/
 */

#ifndef ANDROID_FUSION_H
#define ANDROID_FUSION_H

#include "quat.h"
#include "mat.h"
#include "vec.h"

namespace android {

typedef mat<float, 3, 4> mat34_t;

enum FUSION_MODE{
    FUSION_9AXIS, // use accel gyro mag
    FUSION_NOMAG, // use accel gyro (game rotation, gravity)
    FUSION_NOGYRO, // use accel mag (geomag rotation)
    NUM_FUSION_MODE
};

class Fusion {
    /*
     * the state vector is made of two sub-vector containing respectively:
     * - modified Rodrigues parameters
     * - the estimated gyro bias
     */
    quat_t  x0;
    vec3_t  x1;

    /*
     * the predicated covariance matrix is made of 4 3x3 sub-matrices and it is
     * semi-definite positive.
     *
     * P = | P00  P10 | = | P00  P10 |
     *     | P01  P11 |   | P10t P11 |
     *
     * Since P01 = transpose(P10), the code below never calculates or
     * stores P01.
     */
    mat<mat33_t, 2, 2> P;

    /*
     * the process noise covariance matrix
     */
    mat<mat33_t, 2, 2> GQGt;

public:
    Fusion();
    void init(int mode = FUSION_9AXIS);
    void handleGyro(const vec3_t& w, float dT);
    bool handleAcc(const vec3_t& a, float dT);
    bool handleMag(const vec3_t& m);
    vec4_t getAttitude() const;
    vec3_t getBias() const;
    mat33_t getRotationMatrix() const;
    bool hasEstimate() const;
	void setInitFlagTrue(){
		initFlag = true;
	}

private:
    struct Parameter {
        float gyroVar;
        float gyroBiasVar;
        float accStdev;
        float magStdev;
    } mParam;

    mat<mat33_t, 2, 2> Phi;
    vec3_t Ba, Bm;
    uint32_t mInitState;
    float mGyroRate;
    vec<vec3_t, 3> mData;
    size_t mCount[3];
    int mMode;
	bool initFlag;
	bool initFinishFlag;

    enum { ACC=0x1, MAG=0x2, GYRO=0x4 };
    bool checkInitComplete(int, const vec3_t& w, float d = 0);
    void initFusion(const vec4_t& q0, float dT);
    void checkState();
    void predict(const vec3_t& w, float dT);
    void update(const vec3_t& z, const vec3_t& Bi, float sigma);
    static mat34_t getF(const vec4_t& p);
    static vec3_t getOrthogonal(const vec3_t &v);
};

}; // namespace android

#endif // ANDROID_FUSION_H
