#pragma once
#include <math.h>

#ifdef __CUDACC__
#define HYBRID __host__ __device__
#else
#define HYBRID
#endif

struct Vec3 { float x,y,z; };

HYBRID inline Vec3 make_vec3(float x,float y,float z){return {x,y,z};}
HYBRID inline Vec3 operator+(Vec3 a,Vec3 b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
HYBRID inline Vec3 operator-(Vec3 a,Vec3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
HYBRID inline Vec3 operator*(Vec3 a,Vec3 b){return {a.x*b.x,a.y*b.y,a.z*b.z};}
HYBRID inline Vec3 operator*(Vec3 a,float t){return {a.x*t,a.y*t,a.z*t};}
HYBRID inline Vec3 operator*(float t,Vec3 a){return {a.x*t,a.y*t,a.z*t};}
HYBRID inline Vec3 operator/(Vec3 a,float t){return {a.x/t,a.y/t,a.z/t};}
HYBRID inline Vec3& operator+=(Vec3&a,Vec3 b){a.x+=b.x;a.y+=b.y;a.z+=b.z;return a;}
HYBRID inline Vec3& operator*=(Vec3&a,float t){a.x*=t;a.y*=t;a.z*=t;return a;}
HYBRID inline float dot(Vec3 a,Vec3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
HYBRID inline Vec3 cross(Vec3 a,Vec3 b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
HYBRID inline float length(Vec3 a){return sqrtf(dot(a,a));}
HYBRID inline float length2(Vec3 a){return dot(a,a);}
HYBRID inline Vec3 normalize(Vec3 a){float l=length(a);return l>1e-8f?a*(1.f/l):make_vec3(0,0,1);}
HYBRID inline Vec3 reflect(Vec3 v,Vec3 n){return v-2.f*dot(v,n)*n;}
HYBRID inline Vec3 clamp3(Vec3 c){
    return {c.x<0?0:c.x>1?1:c.x, c.y<0?0:c.y>1?1:c.y, c.z<0?0:c.z>1?1:c.z};
}
HYBRID inline float fmaxf3(float a,float b,float c){return a>b?(a>c?a:c):(b>c?b:c);}
