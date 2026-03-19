
#ifndef TYPE_HPP
#define TYPE_HPP

#include <cmath>
#include <vector>
#include <numeric>
#include <algorithm>
#include <iostream>

namespace FSD
{

struct conePos
{
    float x;
    float y;
    conePos(float x_ = 0, float y_ = 0) {
        x = x_;
        y = y_;
    }
};


struct pathPoint
{
    int id;
    float x,y = 0;
    conePos left_cone;
    conePos right_cone;
    float wrong_cnt = 0;
    void calMidCone() {
        x = (left_cone.x + right_cone.x) / 2.0;
        y = (left_cone.y + right_cone.y) / 2.0;
    }
};

};


#endif