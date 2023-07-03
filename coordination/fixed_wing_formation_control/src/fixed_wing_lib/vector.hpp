/*
 * @------------------------------------------1: 1------------------------------------------@
 * @修改自  lee-shun Email: 2015097272@qq.com
 * @Description:  向量类，用时候会方便一些
 */

#ifndef _VECTOR_HPP_
#define _VECTOR_HPP_

#include <math.h>
#include <iostream>
#include "mathlib.hpp"
using namespace std;

class Point;
typedef Point Vec;
class Point
{
private:
public:
    float x;
    float y;

    Point(double _x = 0, double _y = 0) : x(_x), y(_y) {}

    ~Point() {}

    //向量与常数 注意常数要放在后面
    Vec operator*(double p) { return Vec(x * p, y * p); }
    Vec operator/(double p) { return Vec(x / p, y / p); }
    Vec operator-() { return Vec(-x, -y); }
    Vec operator-(Vec obj) { return Vec(x - obj.x, y - obj.y); }
    Vec operator+(Vec obj) { return Vec(x + obj.x, y + obj.y); }

    //点积
    double operator*(Vec obj) { return x * obj.x + y * obj.y; }

    //叉积
    double operator^(Vec obj) { return x * obj.y - y * obj.x; }

    //两个向量的夹角 A*B=|A|*|B|*cos(th)
    double Angle(Vec B) { return acos((*this) * B / (*this).len() / B.len()); }

    //两条向量平行四边形的面积
    double Area(Vec B) { return fabs((*this) ^ B); }

    //向量旋转
    //旋转公式
    //  Nx     (cos  -sin) x
    //  Ny     (sin   cos) y
    Vec Rotate(double rad) { return Vec(x * cos(rad) - y * sin(rad), x * sin(rad) + y * cos(rad)); }

    //返回向量的法向量，即旋转pi/2
    Vec Normal() { return Vec(-y, x); }

    //返回向量的长度,或者点距离原点的距离
    double len() { return hypot(x, y); }

    double len2() { return x * x + y * y; }

    //归一化,如果是零向量，则返回零向量
    Vec normalized()
    {
        if ((*this).len() == 0)
        {
            return Vec(0, 0);
        }
        else
        {
            return Vec((*this).x, (*this).y) / (*this).len();
        }
    }

    //返回两点之间的距离
    double dis(Point obj) { return hypot(x - obj.x, y - obj.y); } //hypot 给定直角三角形的两条直角边，返回斜边边长

    //向量的极角 atan2(y,x)
    bool operator==(Point obj) { return dcmp(x - obj.x) == 0 && dcmp(y - obj.y) == 0; }

    bool operator<(Point obj) { return x < obj.x || (x == obj.x && y < obj.y); }

    /*
    1.精度  要学着用dcmp
    2.引用  不能交换两个引用swap（）,引用中间不能变
    */
    //三态函数比较;精度问题
    int dcmp(double x)
    {
        const double eps = 1e-9;
        if (fabs(x) < eps)
            return 0;
        return x < 0 ? -1 : 1;
    }

    void print()
    {
        cout << "my_x =" << x << "  "
             << "my_y =" << y << endl;
    }

    void set_vec_ele(float a, float b)
    {
        x = a;
        y = b;
    }
};

#endif
