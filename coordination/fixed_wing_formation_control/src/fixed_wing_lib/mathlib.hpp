#ifndef _MATHLIB_H_
#define _MATHLIB_H_
#include <math.h>
#include <iostream>

using namespace std;

#define PI 3.1415926535
#define CONSTANTS_RADIUS_OF_EARTH 6371000
#define EARTH_RADIUS 6378137
#define CONSTANTS_ONE_G 9.80665

//获取绝对值
float abs_num(float a)
{
    float result;
    if (a < 0)
        result = -a;

    else
        result = a;

    return result;
}

bool ISFINITE(float a)
{
    if ((abs_num(a) > 0.02) && (abs_num(a) < 1000))
    {
        return true;
    }
    else
        return false;
}

float constrain(float val, float min, float max)
{
    return (val < min) ? min : ((val > max) ? max : val);
}
float max(const float a, const float b)
{
    return (a > b) ? a : b;
}
float min(const float a, const float b)
{
    return (a < b) ? a : b;
}

void quaternion_2_euler(float quat[4], float angle[3])
{
    // 四元数转Euler
    // q0 q1 q2 q3
    // w x y z
    angle[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    angle[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    angle[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), -1.0 + 2.0 * (quat[0] * quat[0] + quat[1] * quat[1]));
}

void euler_2_quaternion(float angle[3], float quat[4])
{
    // Euler转四元数
    // q0 q1 q2 q3
    // w x y z
    double cosPhi_2 = cos(double(angle[0]) / 2.0);

    double sinPhi_2 = sin(double(angle[0]) / 2.0);

    double cosTheta_2 = cos(double(angle[1]) / 2.0);

    double sinTheta_2 = sin(double(angle[1]) / 2.0);

    double cosPsi_2 = cos(double(angle[2]) / 2.0);

    double sinPsi_2 = sin(double(angle[2]) / 2.0);

    quat[0] = float(cosPhi_2 * cosTheta_2 * cosPsi_2 + sinPhi_2 * sinTheta_2 * sinPsi_2);

    quat[1] = float(sinPhi_2 * cosTheta_2 * cosPsi_2 - cosPhi_2 * sinTheta_2 * sinPsi_2);

    quat[2] = float(cosPhi_2 * sinTheta_2 * cosPsi_2 + sinPhi_2 * cosTheta_2 * sinPsi_2);

    quat[3] = float(cosPhi_2 * cosTheta_2 * sinPsi_2 - sinPhi_2 * sinTheta_2 * cosPsi_2);
}

void matrix_plus_vector_3(float vector_a[3], float rotmax[3][3], float vector_b[3])
{
    vector_a[0] = rotmax[0][0] * vector_b[0] + rotmax[0][1] * vector_b[1] + rotmax[0][2] * vector_b[2];

    vector_a[1] = rotmax[1][0] * vector_b[0] + rotmax[1][1] * vector_b[1] + rotmax[1][2] * vector_b[2];

    vector_a[2] = rotmax[2][0] * vector_b[0] + rotmax[2][1] * vector_b[1] + rotmax[2][2] * vector_b[2];
}

/**
	 * 旋转矩阵
	 */
void quat_2_rotmax(float q[4], float R[3][3])
{
    float aSq = q[0] * q[0];
    float bSq = q[1] * q[1];
    float cSq = q[2] * q[2];
    float dSq = q[3] * q[3];
    R[0][0] = aSq + bSq - cSq - dSq;
    R[0][1] = 2.0f * (q[1] * q[2] - q[0] * q[3]);
    R[0][2] = 2.0f * (q[0] * q[2] + q[1] * q[3]);
    R[1][0] = 2.0f * (q[1] * q[2] + q[0] * q[3]);
    R[1][1] = aSq - bSq + cSq - dSq;
    R[1][2] = 2.0f * (q[2] * q[3] - q[0] * q[1]);
    R[2][0] = 2.0f * (q[1] * q[3] - q[0] * q[2]);
    R[2][1] = 2.0f * (q[0] * q[1] + q[2] * q[3]);
    R[2][2] = aSq - bSq - cSq + dSq;
}

//角度与弧度
float rad_2_deg(float rad)
{
    float deg;

    deg = rad * 180 / PI;

    return deg;
}

float deg_2_rad(float deg)
{
    float rad;

    rad = deg * PI / 180;

    return rad;
}

//ref,result---lat,long,alt
//转经纬度
void cov_m_2_lat_long_alt(double ref[3], float x, float y, float z, double result[3])
{

    if (x == 0 && y == 0)
    {
        result[0] = ref[0];
        result[1] = ref[1];
    }
    else
    {
        double local_radius = cos(deg_2_rad(ref[0])) * EARTH_RADIUS; //lat是

        result[0] = ref[0] + rad_2_deg(x / EARTH_RADIUS); //得到的是lat，x是北向位置，所以在大圆上

        result[1] = ref[1] + rad_2_deg(y / local_radius); //得到的是long，在维度圆上
    }

    result[2] = ref[2] + z; //高度
}

void cov_lat_long_2_m(double a_pos[2], double b_pos[2], double m[2])
{ //参考点是a点，lat，long，alt
    double lat1 = a_pos[0];
    double lon1 = a_pos[1];

    double lat2 = b_pos[0];
    double lon2 = b_pos[1];

    double n_distance = deg_2_rad(lat2 - lat1) * EARTH_RADIUS; //涉及到ned是向北增加，且纬度向北也增加

    double r_at_ref1 = cos(deg_2_rad(lat1)) * EARTH_RADIUS;

    double e_distance = deg_2_rad(lon2 - lon1) * r_at_ref1; //涉及到ned是向东增加，但是经度向东减少

    m[0] = n_distance;
    m[1] = e_distance;
}

//sat函数
float sat_function(float x, float b, float c)
{
    if (x < b)
    {
        return b;
    }
    else if (x > c)
    {
        return c;
    }
    else
    {
        return x;
    }
}

//sign符号函数
float sign_function(float s)
{
    if (s < 0)
    {
        return -1.0;
    }
    else if (s == 0)
    {
        return 0.0;
    }
    else
    {
        return 1.0;
    }    
}

//平滑符号函数所用，滑膜控制
float sign_replace_function(float s)
{
    float eplision, temp;
    eplision = 0.1;

    temp = s / (abs(s) + eplision);
    return temp;
}

#endif