#pragma once
#include "vec3.h"
#include "material.h"
#include "mesh.h"
#include "camera.h"
#include <vector>
#include <string>

struct Light {
    Vec3  pos;
    Vec3  color={1,1,1};
    float intensity=1.f;
    float radius=0.f;  // >0 for soft shadows (area light)
};

// Flat GPU-uploadable triangle (no STL)
struct GPUTri {
    Vec3  v0,v1,v2;
    Vec3  n0,n1,n2;
    float u0,v0_,u1,v1_,u2,v2_;
    int   matId;
};

HYBRID inline GPUTri toGPUTri(const Triangle& t){
    GPUTri g;
    g.v0=t.v0.pos;g.v1=t.v1.pos;g.v2=t.v2.pos;
    g.n0=t.v0.normal;g.n1=t.v1.normal;g.n2=t.v2.normal;
    g.u0=t.v0.u;g.v0_=t.v0.v;
    g.u1=t.v1.u;g.v1_=t.v1.v;
    g.u2=t.v2.u;g.v2_=t.v2.v;
    g.matId=t.matId;
    return g;
}

struct SceneDesc {
    Camera cam;
    Vec3   missColor={0.5f,0.7f,1.f};
    int    spp=64, maxDepth=8;
    bool   diffuseBounce=true;
    std::vector<Material> materials;
    std::vector<Triangle> tris;
    std::vector<Light>    lights;
};
