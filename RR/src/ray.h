#pragma once
#include "vec3.h"

struct Ray {
    Vec3 orig, dir;
    HYBRID Ray(){}
    HYBRID Ray(Vec3 o,Vec3 d):orig(o),dir(d){}
    HYBRID Vec3 at(float t)const{return orig+dir*t;}
};
